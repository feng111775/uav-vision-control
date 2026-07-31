"""Two-level H7 detector: blob measurement every frame, Hough periodically."""
import math

from detector import DTaskDetector, _candidate_from_region, _invalid
from tracker import TrackState
from config import (
    MAX_CANDIDATE_REGIONS, MAX_CONSECUTIVE_VERIFY_FAILURES,
    MAX_STRONG_VERIFY_AGE, MAX_TRACK_JUMP_DIAMETER_RATIO,
    MAX_TRACK_SIZE_RATIO, MIN_TRACK_SIZE_RATIO, ROI_MAX_LOST_FRAMES,
    ROI_SCALE, SEARCH_FULL_VERIFY_INTERVAL, TARGET_RATIO_NOMINAL,
    TRACK_FULL_VERIFY_INTERVAL,
)


class FastV2Detector(DTaskDetector):
    def __init__(self, enable_timing=False):
        super().__init__(enable_timing)
        self.track = TrackState()
        self.mode = 'SEARCH'
        self.frame = 0
        self.last_verify_attempt_frame = None
        self.last_verified_success_frame = None
        self.last_verified_success_result = None
        self.last_verified_frame = None
        self.last_verified = None
        self.consecutive_verify_failures = 0
        self.last_selected_candidate = None

    def _verification_age(self):
        if self.last_verified_success_frame is None and self.last_verified_frame is not None:
            self.last_verified_success_frame = self.last_verified_frame
        if self.last_verified_success_result is None and self.last_verified is not None:
            self.last_verified_success_result = self.last_verified
        if self.last_verified_success_frame is None:
            return MAX_STRONG_VERIFY_AGE + 1
        return max(0, self.frame - self.last_verified_success_frame)

    def _verification_grace_active(self):
        return (
            self.last_verified_success_result is not None and
            self.last_verified_success_result.get('valid', 0) and
            self._verification_age() <= MAX_STRONG_VERIFY_AGE
        )

    def _verify_due(self, roi):
        if self.mode == 'SEARCH' or roi is None:
            interval = SEARCH_FULL_VERIFY_INTERVAL
        elif self.mode == 'DROP_ALIGN':
            interval = max(1, TRACK_FULL_VERIFY_INTERVAL // 2)
        else:
            interval = TRACK_FULL_VERIFY_INTERVAL
        reference = self.last_verify_attempt_frame
        if reference is None:
            reference = self.last_verified_success_frame
        return reference is None or self.frame % interval == 0

    def _continuity_metrics(self, candidate):
        if self.track.last is None:
            return 0.0, 1.0, 1.0
        jump_px = math.sqrt(
            (candidate['cx'] - self.track.last['cx']) ** 2 +
            (candidate['cy'] - self.track.last['cy']) ** 2)
        last_diameter = max(1.0, float(self.track.last['outer_diameter_px']))
        jump_ratio = jump_px / last_diameter
        size_ratio = candidate['diameter'] / last_diameter
        return jump_px, jump_ratio, size_ratio

    def _verified_position_error(self, candidate):
        if self.last_verified_success_result is None:
            return 0.0
        cx = self.last_verified_success_result.get('cx', candidate['cx'])
        cy = self.last_verified_success_result.get('cy', candidate['cy'])
        diameter = max(1.0, float(candidate['diameter']))
        jump = math.sqrt((candidate['cx'] - cx) ** 2 + (candidate['cy'] - cy) ** 2)
        return jump / diameter

    def _candidate_sort_key(self, candidate):
        jump_px, jump_ratio, size_ratio = self._continuity_metrics(candidate)
        position_consistency = self._verified_position_error(candidate)
        search_score = (
            -candidate['score'],
            position_consistency,
            -candidate['roundness'],
            -candidate['aspect'],
            jump_px,
        )
        if self.track.last is None:
            return search_score
        return (
            jump_ratio > MAX_TRACK_JUMP_DIAMETER_RATIO,
            size_ratio < MIN_TRACK_SIZE_RATIO or size_ratio > MAX_TRACK_SIZE_RATIO,
            jump_px,
            abs(1.0 - size_ratio),
            -candidate['roundness'],
            -candidate['aspect'],
            position_consistency,
            -candidate['score'],
        )

    def _select_candidate(self, candidates):
        if not candidates:
            return None, -1
        normalized = [(index, _candidate_from_region(candidate)) for index, candidate in enumerate(candidates)]
        ordered = sorted(normalized, key=lambda item: self._candidate_sort_key(item[1]))
        for rank, (index, candidate) in enumerate(ordered):
            jump_px, jump_ratio, size_ratio = self._continuity_metrics(candidate)
            if self.track.last is not None:
                if jump_ratio > MAX_TRACK_JUMP_DIAMETER_RATIO:
                    continue
                if size_ratio < MIN_TRACK_SIZE_RATIO or size_ratio > MAX_TRACK_SIZE_RATIO:
                    continue
            switched = (
                self.last_selected_candidate is not None and
                self.last_selected_candidate != (candidate['cx'], candidate['cy'], candidate['diameter'])
            )
            self.diagnostics.note_selected_candidate(rank, jump_px, jump_ratio, switched)
            self.last_selected_candidate = (candidate['cx'], candidate['cy'], candidate['diameter'])
            return candidate, rank
        if self.track.last is None:
            index, candidate = ordered[0]
            self.diagnostics.note_selected_candidate(0, 0.0, 0.0, False)
            self.last_selected_candidate = (candidate['cx'], candidate['cy'], candidate['diameter'])
            return candidate, 0
        return None, -1

    def _measurement_from_candidate(self, candidate):
        if candidate is None:
            return None
        age = self._verification_age()
        verified_ok = self._verification_grace_active()
        if verified_ok:
            status = 'TRACKING'
            angle = self.last_verified_success_result.get('angle_rad', 0.0)
            confidence = self.last_verified_success_result.get('confidence', 0)
            self.diagnostics.note_grace_frame()
        else:
            status = 'VERIFY_EXPIRED' if self.last_verified_success_result else 'CROSS_INVALID'
            angle = 0.0
            confidence = 0
            if age > MAX_STRONG_VERIFY_AGE:
                self.diagnostics.reject('VERIFY_EXPIRED')
        measurement = {
            'valid': int(bool(verified_ok)),
            'measurement_valid': 1,
            'cx': candidate['cx'],
            'cy': candidate['cy'],
            'outer_diameter_px': candidate['diameter'],
            'inner_diameter_px': int(round(candidate['diameter'] * TARGET_RATIO_NOMINAL)),
            'angle_rad': angle,
            'confidence': confidence,
            'status': status,
            'candidate': candidate,
        }
        self.diagnostics.note_current_measurement(candidate)
        return measurement

    def _apply_verified_result(self, measurement, verified):
        if measurement is None or verified is None:
            return measurement
        measurement['valid'] = int(bool(verified.get('valid')))
        measurement['status'] = verified.get('status', 'TRACKING')
        measurement['angle_rad'] = verified.get('angle_rad', 0.0)
        measurement['confidence'] = verified.get('confidence', 0)
        return measurement

    def detect(self, image, mission_mode='SEARCH'):
        self.mode = mission_mode if mission_mode in (
            'MISSION_IDLE', 'SEARCH', 'ACQUIRE', 'FOLLOW', 'DROP_ALIGN') else 'SEARCH'
        self.frame += 1
        roi = self.track.roi(image, ROI_SCALE) if self.track.last else None
        verify_due = self._verify_due(roi)
        self.diagnostics.begin_frame(self.frame, self.mode, verify_due, self._verification_age())
        search_roi = roi or (0, 0, image.width(), image.height())
        started = self.timing.begin()
        candidates = self._search_regions(image, search_roi, include_fallback=False)
        candidates = [_candidate_from_region(candidate) for candidate in candidates[:MAX_CANDIDATE_REGIONS]]
        self.timing.end('candidate_search', started)
        candidate, selected_rank = self._select_candidate(candidates)
        measurement = self._measurement_from_candidate(candidate)

        if verify_due and candidate is not None:
            self.diagnostics.note_verify_attempt()
            self.last_verify_attempt_frame = self.frame
            started = self.timing.begin()
            pairs = self._circle_pairs(image, candidate)
            self.timing.end('strong_verify', started)
            verified = None
            if pairs:
                started = self.timing.begin()
                verified = self._detect_candidate(image, candidate, pairs=pairs)
                self.timing.end('roi_detect', started)
            if verified is not None and verified.get('valid'):
                self.last_verified_success_frame = self.frame
                self.last_verified_success_result = dict(verified)
                self.last_verified_frame = self.frame
                self.last_verified = dict(verified)
                self.consecutive_verify_failures = 0
                self.diagnostics.note_verify_success()
                measurement = self._apply_verified_result(measurement, verified)
            else:
                self.consecutive_verify_failures += 1
                self.diagnostics.note_verify_failure()
                if self.consecutive_verify_failures >= MAX_CONSECUTIVE_VERIFY_FAILURES and not self._verification_grace_active():
                    self.last_verified_success_result = None
        if measurement is None:
            self.track.update(None)
            if self.track.lost > ROI_MAX_LOST_FRAMES:
                self.track.last = None
                self.last = None
                self.mode = 'SEARCH'
                self.last_selected_candidate = None
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
        if not invalid.get('measurement_valid'):
            invalid['cx'] = 0
            invalid['cy'] = 0
            invalid['outer_diameter_px'] = 0
            invalid['inner_diameter_px'] = 0
        return invalid
