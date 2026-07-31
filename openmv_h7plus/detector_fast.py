"""Two-level H7 detector: blob measurement every frame, Hough periodically."""
from detector import DTaskDetector, _invalid
from tracker import TrackState
from config import (
    MAX_CANDIDATE_REGIONS, MAX_STRONG_VERIFY_AGE, ROI_MAX_LOST_FRAMES,
    ROI_SCALE, SEARCH_FULL_VERIFY_INTERVAL, TRACK_FULL_VERIFY_INTERVAL,
    TARGET_RATIO_NOMINAL,
)


class FastV2Detector(DTaskDetector):
    def __init__(self, enable_timing=False):
        super().__init__(enable_timing)
        self.track = TrackState()
        self.mode = 'SEARCH'
        self.frame = 0
        self.last_verified_frame = None
        self.last_verified = None

    def _verification_age(self):
        if self.last_verified_frame is None:
            return MAX_STRONG_VERIFY_AGE + 1
        return max(0, self.frame - self.last_verified_frame)

    def _low_cost_measurement(self, image, region):
        """Return a measurement from this frame's blob/ROI only."""
        x, y, width, height = [int(v) for v in region]
        diameter = max(1, min(width, height))
        if diameter < 24:
            return None
        cx = max(0, min(image.width() - 1, x + width // 2))
        cy = max(0, min(image.height() - 1, y + height // 2))
        age = self._verification_age()
        verified_ok = (
            self.last_verified is not None and
            self.last_verified.get('valid', 0) and
            age <= MAX_STRONG_VERIFY_AGE)
        if verified_ok:
            status = 'TRACKING'
            angle = self.last_verified.get('angle_rad', 0.0)
            confidence = self.last_verified.get('confidence', 0)
        else:
            status = 'VERIFY_EXPIRED' if age > MAX_STRONG_VERIFY_AGE else (
                self.last_verified.get('status', 'CROSS_INVALID')
                if self.last_verified is not None else 'VERIFY_EXPIRED')
            angle = 0.0
            confidence = 0
            if status == 'VERIFY_EXPIRED':
                self.diagnostics.reject('VERIFY_EXPIRED')
        return {
            'valid': int(bool(verified_ok)),
            'measurement_valid': 1,
            'cx': cx, 'cy': cy,
            'outer_diameter_px': diameter,
            'inner_diameter_px': int(round(diameter * TARGET_RATIO_NOMINAL)),
            'angle_rad': angle, 'confidence': confidence,
            'status': status,
        }

    def _verify_due(self, roi):
        if self.mode == 'SEARCH' or roi is None:
            interval = SEARCH_FULL_VERIFY_INTERVAL
        elif self.mode == 'DROP_ALIGN':
            interval = max(1, TRACK_FULL_VERIFY_INTERVAL // 2)
        else:
            interval = TRACK_FULL_VERIFY_INTERVAL
        return self.last_verified_frame is None or self.frame % interval == 0

    def detect(self, image, mission_mode='SEARCH'):
        self.mode = mission_mode if mission_mode in (
            'MISSION_IDLE', 'SEARCH', 'ACQUIRE', 'FOLLOW', 'DROP_ALIGN') else 'SEARCH'
        self.frame += 1
        roi = self.track.roi(image, ROI_SCALE) if self.track.last else None
        verify_due = self._verify_due(roi)
        self.diagnostics.begin_frame(
            self.frame, self.mode, verify_due, self._verification_age())
        search_roi = roi or (0, 0, image.width(), image.height())
        started = self.timing.begin()
        regions = self._search_regions(image, search_roi, include_fallback=False)
        regions = regions[:MAX_CANDIDATE_REGIONS]
        self.timing.end('candidate_search', started)
        measurement = self._low_cost_measurement(image, regions[0]) if regions else None

        if verify_due and regions:
            pairs_cache = {}
            verified = None
            for region in regions:
                self.diagnostics.note_verify_attempt()
                started = self.timing.begin()
                pairs = self._circle_pairs(image, region)
                self.timing.end('strong_verify', started)
                pairs_cache[region] = pairs
                if pairs:
                    started = self.timing.begin()
                    verified = self._detect_roi(
                        image, region, regions=[region], pairs_cache=pairs_cache)
                    self.timing.end('roi_detect', started)
                    if verified is not None:
                        break
            self.last_verified_frame = self.frame
            self.last_verified = (
                verified if verified is not None else _invalid('CROSS_INVALID'))
            if verified is not None and measurement is not None:
                measurement['valid'] = int(bool(verified.get('valid')))
                measurement['status'] = verified.get('status', 'TRACKING')
                measurement['angle_rad'] = verified.get('angle_rad', 0.0)
                measurement['confidence'] = verified.get('confidence', 0)

        if measurement is None:
            self.track.update(None)
            if self.track.lost > ROI_MAX_LOST_FRAMES:
                self.track.last = None
                self.last = None
                self.mode = 'SEARCH'
            return _invalid('LOST')

        self.track.update(measurement)
        self.last = measurement
        if measurement.get('valid'):
            self.last_status = 'TRACKING'
            self.diagnostics.finish(True)
            return measurement
        self.last_status = measurement.get('status', 'CROSS_INVALID')
        invalid = dict(measurement)
        invalid['valid'] = 0
        return invalid
