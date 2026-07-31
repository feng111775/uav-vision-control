"""Formal OpenMV H7 Plus detector for two concentric rings and a cross.

Only APIs documented for OpenMV firmware 5.0.0 are used: get_histogram,
find_blobs, find_circles and find_lines. PC-side accuracy is evaluated by the
OpenCV reference tool; this implementation still requires IDE/hardware tuning.
"""

import math

try:
    import pyb
except ImportError:
    pyb = None

RATIO_MIN = 0.52
RATIO_MAX = 0.68
MAX_CONCENTRIC_ERROR = 0.10
MIN_OUTER_DIAMETER = 24
MAX_OUTER_DIAMETER = 220
MIN_CIRCLE_SCORE = 0.35
MIN_CROSS_SCORE = 0.45
MIN_CONFIDENCE = 55
ROI_SCALE_PERCENT = 165
ROI_FAILURE_LIMIT = 3
# Full-frame Hough search dominates a no-target frame on H7 Plus. Keep the
# heartbeat responsive by scanning every eighth frame until a target is first
# acquired. Tracking and recovery ROI searches remain frame-by-frame.
FULL_SEARCH_INTERVAL = 8
MIN_BLOB_PIXELS = 30
MIN_BLOB_AREA = 80
HALF_PI = math.pi / 2


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


def _invalid(status="LOST"):
    return {
        "valid": 0, "cx": 0, "cy": 0, "outer_diameter_px": 0,
        "inner_diameter_px": 0, "angle_rad": 0.0, "confidence": 0,
        "status": status,
    }


class DTaskDetector:
    """Layered adaptive full-frame/ROI detector with finite recovery."""

    def __init__(self, enable_timing=False):
        self.last = None
        self.roi_failures = 0
        self.last_status = "LOST"
        self.full_search_countdown = 0
        self.timing = StageTiming(enable_timing)

    def _tracking_roi(self, image):
        if self.last is None:
            return None
        diameter = self.last["outer_diameter_px"]
        half = max(16, diameter * ROI_SCALE_PERCENT // 200)
        cx, cy = self.last["cx"], self.last["cy"]
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
        self.timing.end("histogram_otsu", started)
        value = int(_value(threshold, "value", 0))
        # Otsu is frame/ROI adaptive. The bounded margin tolerates dark ring
        # antialiasing without becoming a fixed scene threshold.
        return [(0, min(255, max(12, value + 8)))]

    def _search_regions(self, image, roi):
        thresholds = self._adaptive_dark_threshold(image, roi)
        started = self.timing.begin()
        blobs = image.find_blobs(
            thresholds, roi=roi, pixels_threshold=MIN_BLOB_PIXELS,
            area_threshold=MIN_BLOB_AREA, merge=True, margin=4)
        self.timing.end("find_blobs", started)
        regions = []
        for blob in blobs:
            x = int(_value(blob, "x", 0))
            y = int(_value(blob, "y", 1))
            width = int(_value(blob, "w", 2))
            height = int(_value(blob, "h", 3))
            diameter = max(width, height)
            if diameter < MIN_OUTER_DIAMETER or diameter > MAX_OUTER_DIAMETER:
                continue
            aspect = min(width, height) / max(1.0, max(width, height))
            circularity = float(_value(blob, "roundness", 18))
            if aspect < 0.65 or circularity < 0.20:
                continue
            margin = max(6, diameter // 6)
            x0, y0 = max(0, x - margin), max(0, y - margin)
            x1 = min(image.width(), x + width + margin)
            y1 = min(image.height(), y + height + margin)
            regions.append((x0, y0, x1 - x0, y1 - y0))
        if not regions:
            regions.append(roi)
        return regions[:4]

    def _circle_pairs(self, image, region):
        max_radius = min(region[2], region[3]) // 2
        started = self.timing.begin()
        circles = image.find_circles(
            roi=region, x_stride=2, y_stride=2, threshold=1800,
            x_margin=6, y_margin=6, r_margin=6, r_min=6,
            r_max=max_radius, r_step=2)
        self.timing.end("find_circles", started)
        started = self.timing.begin()
        values = []
        for circle in circles:
            radius = int(_value(circle, "r", 2))
            diameter = radius * 2
            if 10 <= diameter <= MAX_OUTER_DIAMETER:
                magnitude = float(_value(circle, "magnitude", 3))
                values.append((
                    int(_value(circle, "x", 0)),
                    int(_value(circle, "y", 1)), diameter,
                    _clamp(magnitude / 6000.0)))
        pairs = []
        for outer in values:
            if outer[2] < MIN_OUTER_DIAMETER:
                continue
            for inner in values:
                if inner[2] >= outer[2]:
                    continue
                ratio = inner[2] / outer[2]
                concentric = math.sqrt(
                    (inner[0] - outer[0]) ** 2 +
                    (inner[1] - outer[1]) ** 2) / outer[2]
                if RATIO_MIN <= ratio <= RATIO_MAX and (
                        concentric <= MAX_CONCENTRIC_ERROR):
                    pairs.append((outer, inner, ratio, concentric))
        self.timing.end("circle_pair_scoring", started)
        return pairs

    def _cross(self, image, cx, cy, inner_diameter):
        radius = max(8, int(inner_diameter * 0.42))
        roi = (max(0, cx - radius), max(0, cy - radius),
               min(image.width(), cx + radius) - max(0, cx - radius),
               min(image.height(), cy + radius) - max(0, cy - radius))
        started = self.timing.begin()
        lines = image.find_lines(
            roi=roi, x_stride=2, y_stride=1, threshold=700,
            theta_margin=8, rho_margin=8)
        self.timing.end("find_lines", started)
        started = self.timing.begin()
        usable = []
        for line in lines:
            x1, y1 = _value(line, "x1", 0), _value(line, "y1", 1)
            x2, y2 = _value(line, "x2", 2), _value(line, "y2", 3)
            dx, dy = x2 - x1, y2 - y1
            length = math.sqrt(dx * dx + dy * dy)
            if length < inner_diameter * 0.45:
                continue
            distance = abs(dy * cx - dx * cy + x2 * y1 - y2 * x1)
            distance /= max(1.0, length)
            if distance <= max(3.0, inner_diameter * 0.10):
                usable.append((math.atan2(dy, dx), length, distance))
        best = None
        for index in range(len(usable)):
            for second_index in range(index + 1, len(usable)):
                first, second = usable[index], usable[second_index]
                raw_difference = abs(
                    (first[0] - second[0] + math.pi / 2) %
                    math.pi - math.pi / 2)
                orthogonal = 1.0 - abs(
                    raw_difference - math.pi / 2) / math.radians(18)
                score = _clamp(orthogonal) * _clamp(
                    (first[1] + second[1]) /
                    max(1.0, inner_diameter * 1.2))
                score *= _clamp(
                    1.0 - (first[2] + second[2]) /
                    max(1.0, inner_diameter * 0.20))
                if best is None or score > best[0]:
                    best = (score, first[0])
        if best is None or best[0] < MIN_CROSS_SCORE:
            self.timing.end("cross_scoring", started)
            return None, 0.0
        angle = (best[1] + HALF_PI / 2) % HALF_PI - HALF_PI / 2
        self.timing.end("cross_scoring", started)
        return angle, _clamp(best[0])

    def _detect_roi(self, image, roi):
        best = None
        for region in self._search_regions(image, roi):
            for outer, inner, ratio, concentric in self._circle_pairs(
                    image, region):
                cx = (outer[0] + inner[0]) // 2
                cy = (outer[1] + inner[1]) // 2
                angle, cross_score = self._cross(
                    image, cx, cy, inner[2])
                if angle is None:
                    continue
                ratio_score = _clamp(1.0 - abs(ratio - 0.6) / 0.08)
                concentric_score = _clamp(
                    1.0 - concentric / MAX_CONCENTRIC_ERROR)
                border = min(
                    cx, cy, image.width() - cx, image.height() - cy)
                border_score = _clamp(
                    (border - outer[2] / 2) / max(1.0, outer[2] * 0.08))
                continuity = 0.5
                if self.last is not None:
                    jump = math.sqrt(
                        (cx - self.last["cx"]) ** 2 +
                        (cy - self.last["cy"]) ** 2)
                    continuity = _clamp(1.0 - jump / outer[2])
                confidence = 100 * (
                    0.19 * outer[3] + 0.14 * inner[3] +
                    0.18 * ratio_score + 0.18 * concentric_score +
                    0.22 * cross_score + 0.05 * border_score +
                    0.04 * continuity)
                candidate = {
                    "valid": int(confidence >= MIN_CONFIDENCE),
                    "cx": cx, "cy": cy,
                    "outer_diameter_px": outer[2],
                    "inner_diameter_px": inner[2],
                    "angle_rad": angle,
                    "confidence": int(_clamp(confidence / 100) * 100),
                    "status": "TRACKING",
                }
                if best is None or (
                        candidate["confidence"] > best["confidence"]):
                    best = candidate
        return best

    def detect(self, image):
        frame_started = self.timing.begin()
        roi = self._tracking_roi(image)
        result = None
        if roi is None and self.full_search_countdown > 0:
            self.full_search_countdown -= 1
            self.last_status = "CROSS_INVALID"
            self.timing.end("detector_total", frame_started)
            return _invalid(self.last_status)
        if roi is not None and self.roi_failures < ROI_FAILURE_LIMIT:
            result = self._detect_roi(image, roi)
            if result is None or not result["valid"]:
                self.roi_failures += 1
        if result is None and (
                roi is None or self.roi_failures >= ROI_FAILURE_LIMIT):
            result = self._detect_roi(
                image, (0, 0, image.width(), image.height()))
            if roi is None:
                self.full_search_countdown = FULL_SEARCH_INTERVAL - 1
        if result is None:
            self.last_status = "CROSS_INVALID"
            self.timing.end("detector_total", frame_started)
            return _invalid(self.last_status)
        if result["valid"]:
            self.last = result
            self.roi_failures = 0
            self.last_status = "TRACKING"
        else:
            self.last_status = result["status"]
        self.timing.end("detector_total", frame_started)
        return result
