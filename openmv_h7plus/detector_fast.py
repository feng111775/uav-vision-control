"""Two-level H7 detector: blob measurement every frame, Hough periodically."""
import math

from detector import DTaskDetector, _candidate_from_region, _invalid
from tracker import TrackState
from config import (
    ACQUIRE_FAILURE_LIMIT, ACQUIRE_MAX_AGE, ACQUIRE_VERIFY_INTERVAL,
    EXPECTED_OUTER_BLEND, MAX_CANDIDATE_REGIONS,
    MAX_CONSECUTIVE_VERIFY_FAILURES, MAX_STRONG_VERIFY_AGE,
    MAX_TRACK_JUMP_DIAMETER_RATIO, MAX_TRACK_SIZE_RATIO,
    MIN_TRACK_SIZE_RATIO, ROI_MAX_LOST_FRAMES, ROI_SCALE,
    SEARCH_FULL_VERIFY_INTERVAL, TARGET_RATIO_NOMINAL,
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
        self.last_selected_identity = None
        self.acquire_candidate = None
        self.acquire_age = 0
        self.acquire_failure_count = 0
        self.verified_track_active = False
        self.force_full_search = False

    def _state_name(self):
        if self.verified_track_active and self.track.last is not None:
            return 'VERIFIED_TRACK'
        if self.acquire_candidate is not None:
            return 'ACQUIRE'
        return 'SEARCH'

    def _clear_acquire(self, timed_out=False):
        self.acquire_candidate = None
        self.acquire_age = 0
        self.acquire_failure_count = 0
        if timed_out:
            self.diagnostics.note_acquire_timeout()

    def _clear_verified_track(self):
        if self.track.last is not None or self.verified_track_active:
            self.diagnostics.note_verified_track_reset()
        self.track.last = None
        self.track.lost = 0
        self.last = None
        self.verified_track_active = False
        self.last_selected_identity = None
        self.force_full_search = True

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

    def _search_roi(self, image):
        if self.force_full_search:
            return (0, 0, image.width(), image.height())
        if self.verified_track_active and self.track.last is not None:
            roi = self.track.roi(image, ROI_SCALE)
            if roi is not None:
                return roi
        if self.acquire_candidate is not None:
            return self.acquire_candidate['verify_roi']
        return (0, 0, image.width(), image.height())

    def _verify_due(self, roi):
        if self.acquire_candidate is not None:
            interval = ACQUIRE_VERIFY_INTERVAL
        elif not self.verified_track_active or self.track.last is None or not self._verification_grace_active():
            interval = SEARCH_FULL_VERIFY_INTERVAL
        elif self.mode == 'DROP_ALIGN':
            interval = max(1, TRACK_FULL_VERIFY_INTERVAL // 2)
        else:
            interval = TRACK_FULL_VERIFY_INTERVAL
        reference = self.last_verify_attempt_frame
        if reference is None:
            reference = self.last_verified_success_frame
        return reference is None or (self.frame - reference) >= interval

    def _expected_outer_diameter(self, candidate):
        current = float(candidate['diameter'])
        verified = None
        if self.last_verified_success_result is not None:
            verified = float(self.last_verified_success_result.get('outer_diameter_px', 0.0) or 0.0)
        if self.verified_track_active and verified and self.track.last is not None:
            last_diameter = max(1.0, float(self.track.last['outer_diameter_px']))
            size_ratio = current / last_diameter
            if size_ratio < MIN_TRACK_SIZE_RATIO or size_ratio > MAX_TRACK_SIZE_RATIO:
                expected = verified
            else:
                expected = verified * (1.0 - EXPECTED_OUTER_BLEND) + current * EXPECTED_OUTER_BLEND
            return max(verified * 0.85, min(verified * 1.15, expected))
        return current

    def _continuity_metrics(self, candidate, reference=None):
        reference = reference or self.track.last
        if reference is None:
            return 0.0, 0.0, 1.0
        jump_px = math.sqrt((candidate['cx'] - reference['cx']) ** 2 + (candidate['cy'] - reference['cy']) ** 2)
        last_diameter = max(1.0, float(reference['outer_diameter_px']))
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
        reference = self.track.last if self.verified_track_active else None
        if reference is None and self.acquire_candidate is not None:
            reference = {
                'cx': self.acquire_candidate['cx'],
                'cy': self.acquire_candidate['cy'],
                'outer_diameter_px': self.acquire_candidate['diameter'],
            }
        jump_px, jump_ratio, size_ratio = self._continuity_metrics(candidate, reference)
        position_consistency = self._verified_position_error(candidate)
        if reference is None:
            return (
                candidate.get('edge_clipped', False),
                -candidate['score'],
                position_consistency,
                -candidate['roundness'],
                -candidate['aspect'],
            )
        return (
            candidate.get('edge_clipped', False),
            jump_ratio > MAX_TRACK_JUMP_DIAMETER_RATIO,
            size_ratio < MIN_TRACK_SIZE_RATIO or size_ratio > MAX_TRACK_SIZE_RATIO,
            jump_px,
            abs(1.0 - size_ratio),
            position_consistency,
            -candidate['roundness'],
            -candidate['aspect'],
            -candidate['score'],
        )

    def _is_real_switch(self, candidate, reference):
        if reference is None:
            return False
        jump_px, jump_ratio, size_ratio = self._continuity_metrics(candidate, reference)
        if candidate.get('candidate_id') != reference.get('candidate_id'):
            if jump_ratio > MAX_TRACK_JUMP_DIAMETER_RATIO:
                return True
            if size_ratio < MIN_TRACK_SIZE_RATIO or size_ratio > MAX_TRACK_SIZE_RATIO:
                return True
        return False

    def _select_candidate(self, candidates):
        if not candidates:
            return None, -1
        normalized = [(index, _candidate_from_region(candidate)) for index, candidate in enumerate(candidates)]
        ordered = sorted(normalized, key=lambda item: self._candidate_sort_key(item[1]))
        reference = None
        if self.verified_track_active and self.track.last is not None:
            reference = dict(self.track.last)
            reference['candidate_id'] = self.last_selected_identity
        elif self.acquire_candidate is not None:
            reference = {
                'cx': self.acquire_candidate['cx'],
                'cy': self.acquire_candidate['cy'],
                'outer_diameter_px': self.acquire_candidate['diameter'],
                'candidate_id': self.acquire_candidate.get('candidate_id'),
            }
        for rank, (_, candidate) in enumerate(ordered):
            jump_px, jump_ratio, size_ratio = self._continuity_metrics(candidate, reference)
            if reference is not None:
                if jump_ratio > MAX_TRACK_JUMP_DIAMETER_RATIO:
                    continue
                if size_ratio < MIN_TRACK_SIZE_RATIO or size_ratio > MAX_TRACK_SIZE_RATIO:
                    continue
            switched = self._is_real_switch(candidate, reference)
            self.diagnostics.note_selected_candidate(rank, jump_px, jump_ratio, switched)
            return candidate, rank
        if reference is None:
            candidate = ordered[0][1]
            self.diagnostics.note_selected_candidate(0, 0.0, 0.0, False)
            return candidate, 0
        return None, -1

    def _measurement_from_candidate(self, candidate):
        if candidate is None:
            return None
        age = self._verification_age()
        verified_ok = self._verification_grace_active() and self.verified_track_active
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
        measurement['outer_diameter_px'] = verified.get('outer_diameter_px', measurement['outer_diameter_px'])
        measurement['inner_diameter_px'] = verified.get('inner_diameter_px', measurement['inner_diameter_px'])
        return measurement

    def _update_acquire(self, candidate):
        if candidate is None:
            if self.acquire_candidate is None:
                return
            self.acquire_failure_count += 1
            self.acquire_age += 1
        elif self.acquire_candidate is None:
            self.acquire_candidate = dict(candidate)
            self.acquire_age = 1
            self.acquire_failure_count = 0
        else:
            jump_px, jump_ratio, size_ratio = self._continuity_metrics(
                candidate,
                {'cx': self.acquire_candidate['cx'], 'cy': self.acquire_candidate['cy'], 'outer_diameter_px': self.acquire_candidate['diameter']},
            )
            if jump_ratio <= MAX_TRACK_JUMP_DIAMETER_RATIO and MIN_TRACK_SIZE_RATIO <= size_ratio <= MAX_TRACK_SIZE_RATIO:
                self.acquire_candidate = dict(candidate)
                self.acquire_age += 1
            else:
                self.acquire_candidate = dict(candidate)
                self.acquire_age = 1
                self.acquire_failure_count += 1
        if self.acquire_age > ACQUIRE_MAX_AGE or self.acquire_failure_count >= ACQUIRE_FAILURE_LIMIT:
            self._clear_acquire(timed_out=True)
            self.force_full_search = True
            self.diagnostics.note_full_search_reset()

    def detect(self, image, mission_mode='SEARCH'):
        started = self.timing.begin()
        try:
            return self._detect_impl(image, mission_mode)
        finally:
            self.timing.end('detector_total', started)

    def _detect_impl(self, image, mission_mode='SEARCH'):
        self.frame += 1
        self.mode = self._state_name()
        roi = self._search_roi(image)
        verify_due = self._verify_due(roi if self.verified_track_active else None)
        self.diagnostics.begin_frame(self.frame, self.mode, verify_due, self._verification_age())
        started = self.timing.begin()
        candidates = self._search_regions(image, roi, include_fallback=False)
        candidates = [_candidate_from_region(candidate) for candidate in candidates[:MAX_CANDIDATE_REGIONS]]
        self.timing.end('candidate_search', started)
        candidate, _ = self._select_candidate(candidates)
        measurement = self._measurement_from_candidate(candidate)

        if candidate is not None and candidate.get('edge_clipped'):
            self.diagnostics.reject('EDGE_CLIPPED')
            self.diagnostics.note_edge_clipped()
            if measurement is not None:
                measurement['valid'] = 0
                measurement['status'] = 'EDGE_CLIPPED'
            self.track.update(None)
            self.last = None
            self.diagnostics.finish(False)
            return measurement or _invalid('EDGE_CLIPPED')

        if self.verified_track_active and candidate is None:
            self.track.update(None)
            if self._verification_age() > MAX_STRONG_VERIFY_AGE:
                self._clear_verified_track()
            return _invalid('LOST')

        if self.verified_track_active and self._verification_age() > MAX_STRONG_VERIFY_AGE:
            self._clear_verified_track()
            self.diagnostics.note_full_search_reset()
            if measurement is not None:
                measurement['valid'] = 0
                measurement['status'] = 'VERIFY_EXPIRED'
            return measurement or _invalid('VERIFY_EXPIRED')

        if verify_due and candidate is not None:
            self.diagnostics.note_verify_attempt()
            self.last_verify_attempt_frame = self.frame
            candidate['expected_outer_diameter'] = self._expected_outer_diameter(candidate)
            started = self.timing.begin()
            pairs = self._circle_pairs(image, candidate)
            self.timing.end('strong_verify', started)
            verified = None
            if pairs and not candidate.get('edge_clipped'):
                started = self.timing.begin()
                verified = self._detect_candidate(image, candidate, pairs=pairs)
                self.timing.end('roi_detect', started)
            if verified is not None and verified.get('valid'):
                self.last_verified_success_frame = self.frame
                self.last_verified_success_result = dict(verified)
                self.last_verified_frame = self.frame
                self.last_verified = dict(verified)
                self.consecutive_verify_failures = 0
                self.verified_track_active = True
                self.force_full_search = False
                self.acquire_candidate = None
                self.acquire_age = 0
                self.acquire_failure_count = 0
                self.track.update(verified)
                self.last = verified
                self.last_selected_identity = candidate.get('candidate_id')
                self.diagnostics.note_verify_success()
                self.diagnostics.finish(True)
                return verified
            self.consecutive_verify_failures += 1
            if not self.verified_track_active and self.acquire_candidate is not None:
                self.acquire_failure_count += 1
            self.diagnostics.note_verify_failure()

        if self.verified_track_active:
            if measurement is not None and self._verification_grace_active():
                self.track.update(measurement)
                self.last = measurement
                self.last_selected_identity = candidate.get('candidate_id') if candidate is not None else self.last_selected_identity
                if measurement.get('valid'):
                    self.last_status = 'TRACKING'
                    self.diagnostics.finish(True)
                    return measurement
            if self.consecutive_verify_failures >= MAX_CONSECUTIVE_VERIFY_FAILURES or self._verification_age() > MAX_STRONG_VERIFY_AGE:
                self._clear_verified_track()
                self.diagnostics.note_full_search_reset()
            invalid = dict(measurement or _invalid('VERIFY_EXPIRED'))
            invalid['valid'] = 0
            invalid['status'] = invalid.get('status', 'VERIFY_EXPIRED')
            return invalid

        self._update_acquire(candidate)
        self.mode = self._state_name()
        self.track.update(None)
        self.last = None
        if measurement is None:
            if self.force_full_search:
                self.mode = 'SEARCH'
            return _invalid('LOST')
        invalid = dict(measurement)
        invalid['valid'] = 0
        invalid['status'] = self.mode
        return invalid
