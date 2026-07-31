"""Formal OpenMV H7 Plus detector for two concentric rings and a cross."""

import math

try:
    import pyb
except ImportError:
    pyb = None

from config import (
    BLOB_MARGIN_DIVISOR, BLOB_MARGIN_PIXELS, BLOB_MERGE,
    BLOB_MERGED_MARGIN, CIRCLE_R_MARGIN, CIRCLE_R_MIN, CIRCLE_R_STEP,
    CIRCLE_THRESHOLD, CIRCLE_X_MARGIN, CIRCLE_X_STRIDE, CIRCLE_Y_MARGIN,
    CIRCLE_Y_STRIDE, CROSS_ANGLE_TOLERANCE_DEG, CROSS_DISTANCE_PENALTY_RATIO,
    CROSS_LINE_SUM_RATIO, CROSS_ROI_RADIUS_RATIO, DEBUG_CAPTURE_ONCE,
    DEBUG_CAPTURE_RAW_PATH, DEBUG_CAPTURE_THRESHOLD_PATH, DIAGNOSTIC_PERIOD_MS,
    ENABLE_MERGED_BLOB_FALLBACK, FULL_SEARCH_INTERVAL, INNER_OUTER_RATIO_MAX,
    INNER_OUTER_RATIO_MIN, LINE_CENTER_DISTANCE_MIN,
    LINE_CENTER_DISTANCE_RATIO, LINE_RHO_MARGIN, LINE_THETA_MARGIN,
    LINE_THRESHOLD, LINE_X_STRIDE, LINE_Y_STRIDE, MAX_CONCENTRIC_ERROR,
    MAX_OUTER_DIAMETER, MAX_SEARCH_REGIONS, MIN_BLOB_AREA, MIN_BLOB_ASPECT,
    MIN_BLOB_PIXELS, MIN_BLOB_ROUNDNESS, MIN_CIRCLE_DIAMETER,
    MIN_CONFIDENCE, MIN_CROSS_SCORE, MIN_LINE_LENGTH_RATIO,
    MIN_OUTER_DIAMETER, ROI_FAILURE_LIMIT, ROI_SCALE_PERCENT,
    TARGET_RATIO_NOMINAL,
)

HALF_PI = math.pi / 2
REJECT_REASONS = (
    'NO_BLOB', 'BLOB_TOO_SMALL', 'BLOB_TOO_LARGE', 'BLOB_ASPECT_REJECT',
    'BLOB_ROUNDNESS_REJECT', 'NO_CIRCLE', 'NO_CONCENTRIC_PAIR',
    'RATIO_REJECT', 'CONCENTRIC_ERROR', 'NO_CROSS_LINES',
    'CROSS_SCORE_LOW', 'CONFIDENCE_LOW', 'VERIFY_EXPIRED', 'VALID'
)


class StageTiming:
    """Bounded timing samples; disabled instances add no clock calls."""

    def __init__(self, enabled=False, window=64):
        self.enabled = bool(enabled and pyb is not None)
        self.window = max(8, int(window))
        self.samples = {}

    def begin(self):
        return pyb.micros() if self.enabled else None

    def end(self, name, start):
        if start is None:
            return
        values = self.samples.setdefault(name, [])
        values.append(pyb.elapsed_micros(start) / 1000.0)
        if len(values) > self.window:
            del values[0]

    def summary(self):
        result = {}
        for name, values in self.samples.items():
            ordered = sorted(values)
            p50 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.50))]
            p95 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))]
            result[name] = (
                sum(values) / len(values), p50, p95, max(values), len(values))
        return result


class DetectionStatsReporter:
    """Aggregate detector-stage diagnostics and emit one line per second."""

    def __init__(self, period_ms=DIAGNOSTIC_PERIOD_MS):
        self.period_ms = max(250, int(period_ms))
        self.writer = None
        self.last_emit_ms = None
        self.reset()

    def reset(self):
        self.frame_sequence = 0
        self.mode = 'SEARCH'
        self.blob_count = 0
        self.region_count = 0
        self.verify_due_count = 0
        self.verify_attempt_count = 0
        self.circle_count = 0
        self.circle_pair_count = 0
        self.cross_line_count = 0
        self.cross_candidate_count = 0
        self.best_cross_score = 0.0
        self.best_confidence = 0.0
        self.best_ratio = 0.0
        self.best_ratio_error = None
        self.last_verified_age = -1
        self.valid_count = 0
        self.reject_counts = {reason: 0 for reason in REJECT_REASONS}

    def set_writer(self, writer):
        self.writer = writer

    def begin_frame(self, frame_sequence, mode, verify_due, last_verified_age):
        self.frame_sequence = int(frame_sequence)
        self.mode = mode or 'SEARCH'
        self.verify_due_count += int(bool(verify_due))
        self.last_verified_age = int(last_verified_age)

    def reject(self, reason, count=1):
        if reason in self.reject_counts:
            self.reject_counts[reason] += int(max(0, count))

    def note_blobs(self, count):
        self.blob_count += int(max(0, count))

    def note_regions(self, count):
        self.region_count += int(max(0, count))

    def note_verify_attempt(self):
        self.verify_attempt_count += 1

    def note_circles(self, count):
        self.circle_count += int(max(0, count))

    def note_circle_pairs(self, count):
        self.circle_pair_count += int(max(0, count))

    def note_cross_lines(self, count):
        self.cross_line_count += int(max(0, count))

    def note_cross_candidates(self, count):
        self.cross_candidate_count += int(max(0, count))

    def note_best_cross_score(self, score):
        self.best_cross_score = max(self.best_cross_score, float(score or 0.0))

    def note_best_confidence(self, confidence):
        self.best_confidence = max(self.best_confidence, float(confidence or 0.0))

    def note_ratio(self, ratio):
        ratio = float(ratio or 0.0)
        error = abs(ratio - TARGET_RATIO_NOMINAL)
        if self.best_ratio_error is None or error < self.best_ratio_error:
            self.best_ratio_error = error
            self.best_ratio = ratio

    def finish(self, valid):
        if valid:
            self.valid_count += 1
            self.reject('VALID')

    def _reject_payload(self):
        items = []
        for reason in REJECT_REASONS:
            count = self.reject_counts.get(reason, 0)
            if count:
                items.append('%s:%d' % (reason, count))
        return '|'.join(items) if items else 'NONE:0'

    def emit_if_due(self, now_ms):
        if self.writer is None or now_ms is None:
            return
        if self.last_emit_ms is None:
            self.last_emit_ms = now_ms
            return
        if now_ms - self.last_emit_ms < self.period_ms:
            return
        line = (
            'D_DETECT_STATS,%d,%s,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f,%d,%d,%d,%s\n'
            % (self.frame_sequence, self.mode, self.blob_count, self.region_count,
               self.verify_due_count, self.verify_attempt_count,
               self.circle_count, self.circle_pair_count, self.cross_line_count,
               self.best_cross_score, self.best_confidence, self.best_ratio,
               self.cross_candidate_count, self.last_verified_age,
               self.valid_count, self._reject_payload()))
        try:
            self.writer.write(line)
        except Exception:
            pass
        self.last_emit_ms = now_ms
        self.reset()


def _value(item, name, index):
    value = getattr(item, name, None)
    if value is None:
        value = item[index]
    return value() if callable(value) else value


def _clamp(value, lower=0.0, upper=1.0):
    if value < lower:
        return lower
    if value > upper:
        return upper
    return value


def _periodic_error(first, second):
    return (first - second + HALF_PI / 2) % HALF_PI - HALF_PI / 2


def _invalid(status='LOST'):
    return {
        'valid': 0, 'cx': 0, 'cy': 0, 'outer_diameter_px': 0,
        'inner_diameter_px': 0, 'angle_rad': 0.0, 'confidence': 0,
        'status': status,
    }


class DTaskDetector:
    """Layered adaptive full-frame/ROI detector with finite recovery."""

    def __init__(self, enable_timing=False):
        self.last = None
        self.roi_failures = 0
        self.last_status = 'LOST'
        self.full_search_countdown = 0
        self.timing = StageTiming(enable_timing)
        self.diagnostics = DetectionStatsReporter()
        self._debug_capture_saved = False

    def set_diagnostic_writer(self, writer):
        self.diagnostics.set_writer(writer)

    def emit_diagnostics(self, now_ms=None):
        self.diagnostics.emit_if_due(now_ms)

    def _tracking_roi(self, image):
        if self.last is None:
            return None
        diameter = self.last['outer_diameter_px']
        half = max(16, diameter * ROI_SCALE_PERCENT // 200)
        cx, cy = self.last['cx'], self.last['cy']
        x0, y0 = max(0, cx - half), max(0, cy - half)
        x1 = min(image.width(), cx + half)
        y1 = min(image.height(), cy + half)
        if x1 - x0 < 24 or y1 - y0 < 24:
            return None
        return (x0, y0, x1 - x0, y1 - y0)

    def _adaptive_dark_threshold(self, image, roi):
        started = self.timing.begin()
        histogram = image.get_histogram(roi=roi)
        threshold = histogram.get_threshold()
        self.timing.end('histogram_otsu', started)
        value = int(_value(threshold, 'value', 0))
        return [(0, min(255, max(12, value + 8)))]

    def _maybe_capture_debug_once(self, image, thresholds):
        if self._debug_capture_saved or not DEBUG_CAPTURE_ONCE:
            return
        try:
            image.save(DEBUG_CAPTURE_RAW_PATH)
            copy = image.copy()
            copy.binary(thresholds)
            copy.save(DEBUG_CAPTURE_THRESHOLD_PATH)
            self._debug_capture_saved = True
        except Exception:
            self._debug_capture_saved = True

    def _regions_from_blobs(self, image, blobs):
        regions = []
        for blob in blobs:
            x = int(_value(blob, 'x', 0))
            y = int(_value(blob, 'y', 1))
            width = int(_value(blob, 'w', 2))
            height = int(_value(blob, 'h', 3))
            diameter = max(width, height)
            if diameter < MIN_OUTER_DIAMETER:
                self.diagnostics.reject('BLOB_TOO_SMALL')
                continue
            if diameter > MAX_OUTER_DIAMETER:
                self.diagnostics.reject('BLOB_TOO_LARGE')
                continue
            aspect = min(width, height) / max(1.0, max(width, height))
            if aspect < MIN_BLOB_ASPECT:
                self.diagnostics.reject('BLOB_ASPECT_REJECT')
                continue
            circularity = float(_value(blob, 'roundness', 18))
            if circularity < MIN_BLOB_ROUNDNESS:
                self.diagnostics.reject('BLOB_ROUNDNESS_REJECT')
                continue
            margin = max(BLOB_MARGIN_PIXELS, diameter // BLOB_MARGIN_DIVISOR)
            x0, y0 = max(0, x - margin), max(0, y - margin)
            x1 = min(image.width(), x + width + margin)
            y1 = min(image.height(), y + height + margin)
            region = (x0, y0, x1 - x0, y1 - y0)
            if region[2] >= 24 and region[3] >= 24:
                regions.append(region)
        return regions[:MAX_SEARCH_REGIONS]

    def _search_regions(self, image, roi, include_fallback=True):
        thresholds = self._adaptive_dark_threshold(image, roi)
        self._maybe_capture_debug_once(image, thresholds)
        started = self.timing.begin()
        blobs = image.find_blobs(
            thresholds, roi=roi, pixels_threshold=MIN_BLOB_PIXELS,
            area_threshold=MIN_BLOB_AREA, merge=BLOB_MERGE,
            margin=BLOB_MERGED_MARGIN)
        self.timing.end('find_blobs', started)
        self.diagnostics.note_blobs(len(blobs))
        regions = self._regions_from_blobs(image, blobs)
        if not blobs:
            self.diagnostics.reject('NO_BLOB')
        if not regions and ENABLE_MERGED_BLOB_FALLBACK and not BLOB_MERGE:
            started = self.timing.begin()
            merged = image.find_blobs(
                thresholds, roi=roi, pixels_threshold=MIN_BLOB_PIXELS,
                area_threshold=MIN_BLOB_AREA, merge=True,
                margin=BLOB_MERGED_MARGIN)
            self.timing.end('find_blobs_merged', started)
            self.diagnostics.note_blobs(len(merged))
            regions = self._regions_from_blobs(image, merged)
        if not regions and include_fallback:
            regions.append(roi)
        self.diagnostics.note_regions(len(regions))
        return regions[:MAX_SEARCH_REGIONS]

    def _circle_pairs(self, image, region):
        max_radius = min(region[2], region[3]) // 2
        started = self.timing.begin()
        circles = image.find_circles(
            roi=region, x_stride=CIRCLE_X_STRIDE, y_stride=CIRCLE_Y_STRIDE,
            threshold=CIRCLE_THRESHOLD, x_margin=CIRCLE_X_MARGIN,
            y_margin=CIRCLE_Y_MARGIN, r_margin=CIRCLE_R_MARGIN,
            r_min=CIRCLE_R_MIN, r_max=max_radius, r_step=CIRCLE_R_STEP)
        self.timing.end('find_circles', started)
        self.diagnostics.note_circles(len(circles))
        if not circles:
            self.diagnostics.reject('NO_CIRCLE')
        started = self.timing.begin()
        values = []
        for circle in circles:
            radius = int(_value(circle, 'r', 2))
            diameter = radius * 2
            if MIN_CIRCLE_DIAMETER <= diameter <= MAX_OUTER_DIAMETER:
                magnitude = float(_value(circle, 'magnitude', 3))
                values.append((
                    int(_value(circle, 'x', 0)),
                    int(_value(circle, 'y', 1)), diameter,
                    _clamp(magnitude / 6000.0)))
        pairs = []
        ratio_rejects = 0
        concentric_rejects = 0
        for outer in values:
            if outer[2] < MIN_OUTER_DIAMETER:
                continue
            for inner in values:
                if inner[2] >= outer[2]:
                    continue
                ratio = inner[2] / outer[2]
                self.diagnostics.note_ratio(ratio)
                concentric = math.sqrt(
                    (inner[0] - outer[0]) ** 2 +
                    (inner[1] - outer[1]) ** 2) / outer[2]
                if not (INNER_OUTER_RATIO_MIN <= ratio <= INNER_OUTER_RATIO_MAX):
                    ratio_rejects += 1
                    continue
                if concentric > MAX_CONCENTRIC_ERROR:
                    concentric_rejects += 1
                    continue
                pairs.append((outer, inner, ratio, concentric))
        self.timing.end('circle_pair_scoring', started)
        self.diagnostics.note_circle_pairs(len(pairs))
        if not pairs and circles:
            self.diagnostics.reject('NO_CONCENTRIC_PAIR')
            if ratio_rejects:
                self.diagnostics.reject('RATIO_REJECT', ratio_rejects)
            if concentric_rejects:
                self.diagnostics.reject('CONCENTRIC_ERROR', concentric_rejects)
        return pairs

    def _cross(self, image, cx, cy, inner_diameter):
        radius = max(8, int(inner_diameter * CROSS_ROI_RADIUS_RATIO))
        roi = (max(0, cx - radius), max(0, cy - radius),
               min(image.width(), cx + radius) - max(0, cx - radius),
               min(image.height(), cy + radius) - max(0, cy - radius))
        started = self.timing.begin()
        lines = image.find_lines(
            roi=roi, x_stride=LINE_X_STRIDE, y_stride=LINE_Y_STRIDE,
            threshold=LINE_THRESHOLD, theta_margin=LINE_THETA_MARGIN,
            rho_margin=LINE_RHO_MARGIN)
        self.timing.end('find_lines', started)
        self.diagnostics.note_cross_lines(len(lines))
        started = self.timing.begin()
        usable = []
        for line in lines:
            x1, y1 = _value(line, 'x1', 0), _value(line, 'y1', 1)
            x2, y2 = _value(line, 'x2', 2), _value(line, 'y2', 3)
            dx, dy = x2 - x1, y2 - y1
            length = math.sqrt(dx * dx + dy * dy)
            if length < inner_diameter * MIN_LINE_LENGTH_RATIO:
                continue
            distance = abs(dy * cx - dx * cy + x2 * y1 - y2 * x1)
            distance /= max(1.0, length)
            if distance <= max(LINE_CENTER_DISTANCE_MIN,
                               inner_diameter * LINE_CENTER_DISTANCE_RATIO):
                usable.append((math.atan2(dy, dx), length, distance))
        if len(usable) < 2:
            self.timing.end('cross_scoring', started)
            self.diagnostics.reject('NO_CROSS_LINES')
            return None, 0.0
        best = None
        candidate_count = 0
        for index in range(len(usable)):
            for second_index in range(index + 1, len(usable)):
                candidate_count += 1
                first, second = usable[index], usable[second_index]
                raw_difference = abs(
                    (first[0] - second[0] + math.pi / 2) %
                    math.pi - math.pi / 2)
                orthogonal = 1.0 - abs(
                    raw_difference - math.pi / 2) / math.radians(CROSS_ANGLE_TOLERANCE_DEG)
                score = _clamp(orthogonal) * _clamp(
                    (first[1] + second[1]) /
                    max(1.0, inner_diameter * CROSS_LINE_SUM_RATIO))
                score *= _clamp(
                    1.0 - (first[2] + second[2]) /
                    max(1.0, inner_diameter * CROSS_DISTANCE_PENALTY_RATIO))
                self.diagnostics.note_best_cross_score(score)
                if best is None or score > best[0]:
                    best = (score, first[0])
        self.diagnostics.note_cross_candidates(candidate_count)
        if best is None or best[0] < MIN_CROSS_SCORE:
            self.timing.end('cross_scoring', started)
            self.diagnostics.reject('CROSS_SCORE_LOW')
            return None, 0.0
        angle = (best[1] + HALF_PI / 2) % HALF_PI - HALF_PI / 2
        self.timing.end('cross_scoring', started)
        return angle, _clamp(best[0])

    def _detect_roi(self, image, roi, regions=None, pairs_cache=None):
        best = None
        if regions is None:
            regions = self._search_regions(image, roi)
        pairs_cache = pairs_cache or {}
        for region in regions:
            pairs = pairs_cache.get(region)
            if pairs is None:
                pairs = self._circle_pairs(image, region)
            for outer, inner, ratio, concentric in pairs:
                cx = (outer[0] + inner[0]) // 2
                cy = (outer[1] + inner[1]) // 2
                angle, cross_score = self._cross(image, cx, cy, inner[2])
                if angle is None:
                    continue
                ratio_score = _clamp(1.0 - abs(ratio - TARGET_RATIO_NOMINAL) / 0.08)
                concentric_score = _clamp(1.0 - concentric / MAX_CONCENTRIC_ERROR)
                border = min(cx, cy, image.width() - cx, image.height() - cy)
                border_score = _clamp(
                    (border - outer[2] / 2) / max(1.0, outer[2] * 0.08))
                continuity = 0.5
                if self.last is not None:
                    jump = math.sqrt(
                        (cx - self.last['cx']) ** 2 +
                        (cy - self.last['cy']) ** 2)
                    continuity = _clamp(1.0 - jump / outer[2])
                confidence = 100 * (
                    0.19 * outer[3] + 0.14 * inner[3] +
                    0.18 * ratio_score + 0.18 * concentric_score +
                    0.22 * cross_score + 0.05 * border_score +
                    0.04 * continuity)
                self.diagnostics.note_best_confidence(confidence)
                candidate = {
                    'valid': int(confidence >= MIN_CONFIDENCE),
                    'cx': cx, 'cy': cy,
                    'outer_diameter_px': outer[2],
                    'inner_diameter_px': inner[2],
                    'angle_rad': angle,
                    'confidence': int(_clamp(confidence / 100) * 100),
                    'status': 'TRACKING' if confidence >= MIN_CONFIDENCE else 'CONFIDENCE_LOW',
                }
                if best is None or candidate['confidence'] > best['confidence']:
                    best = candidate
        if best is not None and not best['valid']:
            self.diagnostics.reject('CONFIDENCE_LOW')
        return best

    def detect(self, image):
        frame_started = self.timing.begin()
        self.diagnostics.begin_frame(0, 'SEARCH', False, -1)
        roi = self._tracking_roi(image)
        result = None
        if roi is None and self.full_search_countdown > 0:
            self.full_search_countdown -= 1
            self.last_status = 'CROSS_INVALID'
            self.timing.end('detector_total', frame_started)
            return _invalid(self.last_status)
        if roi is not None and self.roi_failures < ROI_FAILURE_LIMIT:
            result = self._detect_roi(image, roi)
            if result is None or not result['valid']:
                self.roi_failures += 1
        if result is None and (roi is None or self.roi_failures >= ROI_FAILURE_LIMIT):
            result = self._detect_roi(image, (0, 0, image.width(), image.height()))
            if roi is None:
                self.full_search_countdown = FULL_SEARCH_INTERVAL - 1
        if result is None:
            self.last_status = 'CROSS_INVALID'
            self.timing.end('detector_total', frame_started)
            return _invalid(self.last_status)
        if result['valid']:
            self.last = result
            self.roi_failures = 0
            self.last_status = 'TRACKING'
            self.diagnostics.finish(True)
        else:
            self.last_status = result['status']
        self.timing.end('detector_total', frame_started)
        return result
