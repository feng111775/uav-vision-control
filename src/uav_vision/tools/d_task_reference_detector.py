#!/usr/bin/env python3
"""OpenCV reference detector for the D-task concentric rings and cross.

This is an offline tuning/evaluation implementation, not the OpenMV runtime.
"""

from dataclasses import dataclass
import math
import time

import cv2
import numpy as np


HALF_PI = math.pi / 2.0


def periodic_angle_error(a, b):
    """Return signed smallest angle difference for a pi/2-periodic cross."""
    return (float(a) - float(b) + HALF_PI / 2.0) % HALF_PI - HALF_PI / 2.0


@dataclass
class DetectorConfig:
    ratio_min: float = 0.52
    ratio_max: float = 0.68
    max_concentric_error: float = 0.10
    min_outer_diameter: float = 24.0
    max_outer_fraction: float = 0.95
    min_circularity: float = 0.58
    max_aspect_error: float = 0.30
    min_cross_score: float = 0.48
    min_confidence: float = 55.0
    roi_scale: float = 1.65
    roi_failures_before_full: int = 3


def invalid_result(status='LOST', processing_ms=0.0):
    return {
        'valid': 0, 'center_x_px': 0.0, 'center_y_px': 0.0,
        'outer_diameter_px': 0.0, 'inner_diameter_px': 0.0,
        'angle_rad': 0.0, 'confidence': 0.0, 'status': status,
        'processing_ms': float(processing_ms), 'scores': {},
    }


def geometry_score(outer, inner, config=None):
    """Score already extracted circle/cross geometry without image access."""
    cfg = config or DetectorConfig()
    if outer is None or inner is None:
        return invalid_result('RING_INCOMPLETE')
    ox, oy, od, oc = map(float, outer)
    ix, iy, inner_diameter, ic = map(float, inner)
    if od <= 0.0 or inner_diameter <= 0.0:
        return invalid_result('RING_INCOMPLETE')
    ratio = inner_diameter / od
    concentric = math.hypot(ix - ox, iy - oy) / od
    if not cfg.ratio_min <= ratio <= cfg.ratio_max:
        return invalid_result('RATIO_INVALID')
    if concentric > cfg.max_concentric_error:
        return invalid_result('CONCENTRIC_INVALID')
    ratio_score = max(0.0, 1.0 - abs(ratio - 0.6) / 0.08)
    concentric_score = max(0.0, 1.0 - concentric /
                           cfg.max_concentric_error)
    confidence = 100.0 * (
        0.30 * max(0.0, min(1.0, oc)) +
        0.25 * max(0.0, min(1.0, ic)) +
        0.25 * ratio_score + 0.20 * concentric_score)
    return {
        'valid': 1, 'center_x_px': (ox + ix) / 2.0,
        'center_y_px': (oy + iy) / 2.0, 'outer_diameter_px': od,
        'inner_diameter_px': inner_diameter, 'angle_rad': 0.0,
        'confidence': min(100.0, max(0.0, confidence)),
        'status': 'RINGS_ONLY', 'processing_ms': 0.0,
        'scores': {'ratio': ratio_score, 'concentric': concentric_score},
    }


class DTaskReferenceDetector:
    """Stateful full-frame/ROI reference detector."""

    def __init__(self, config=None):
        self.config = config or DetectorConfig()
        self.last = None
        self.roi_failures = 0
        self.last_search = 'FULL'

    def reset(self):
        self.last = None
        self.roi_failures = 0
        self.last_search = 'FULL'

    def _tracking_roi(self, shape):
        if self.last is None:
            return None
        height, width = shape[:2]
        radius = self.last['outer_diameter_px'] * self.config.roi_scale / 2
        cx, cy = self.last['center_x_px'], self.last['center_y_px']
        x0, y0 = max(0, int(cx - radius)), max(0, int(cy - radius))
        x1, y1 = min(width, int(cx + radius)), min(height, int(cy + radius))
        if x1 - x0 < 16 or y1 - y0 < 16:
            return None
        return x0, y0, x1 - x0, y1 - y0

    @staticmethod
    def _threshold(gray):
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, otsu = cv2.threshold(
            blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        adaptive = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV, 31, 5)
        return cv2.bitwise_or(otsu, adaptive), blur

    def _circle_candidates(self, mask, offset, full_shape):
        contours, _ = cv2.findContours(
            mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        max_diameter = min(full_shape[:2]) * self.config.max_outer_fraction
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if area < 60 or perimeter <= 0:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            diameter = (w + h) / 2.0
            if diameter < self.config.min_outer_diameter * 0.45:
                continue
            if diameter > max_diameter:
                continue
            aspect_score = 1.0 - abs(w - h) / max(w, h)
            circularity = min(1.0, 4 * math.pi * area /
                              (perimeter * perimeter))
            if aspect_score < 1.0 - self.config.max_aspect_error:
                continue
            if circularity < self.config.min_circularity:
                continue
            (cx, cy), enclosing_r = cv2.minEnclosingCircle(contour)
            edge_coverage = min(1.0, perimeter /
                                max(1.0, 2 * math.pi * enclosing_r))
            candidates.append({
                'cx': cx + offset[0], 'cy': cy + offset[1],
                'diameter': diameter,
                'circle': 0.65 * circularity + 0.35 * aspect_score,
                'coverage': edge_coverage,
                'bbox': (x + offset[0], y + offset[1], w, h),
            })
        return candidates

    def _cross(self, gray, cx, cy, inner_diameter, offset):
        radius = max(8, int(inner_diameter * 0.42))
        local_cx, local_cy = int(cx - offset[0]), int(cy - offset[1])
        x0, y0 = max(0, local_cx - radius), max(0, local_cy - radius)
        x1 = min(gray.shape[1], local_cx + radius)
        y1 = min(gray.shape[0], local_cy + radius)
        patch = gray[y0:y1, x0:x1]
        if patch.size == 0:
            return None, 0.0, 1.0
        edges = cv2.Canny(patch, 40, 120)
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, threshold=max(8, radius // 2),
            minLineLength=max(6, int(radius * 0.65)),
            maxLineGap=max(2, radius // 4))
        if lines is None:
            return None, 0.0, 1.0
        usable = []
        center = np.array([local_cx - x0, local_cy - y0], dtype=float)
        for item in lines[:, 0]:
            x2, y2, x3, y3 = map(float, item)
            vector = np.array([x3 - x2, y3 - y2])
            length = float(np.linalg.norm(vector))
            if length < 1.0:
                continue
            distance = abs(np.cross(
                vector, center - np.array([x2, y2]))) / length
            if distance <= max(3.0, inner_diameter * 0.10):
                usable.append((math.atan2(vector[1], vector[0]), length,
                               distance))
        best = None
        for index, first in enumerate(usable):
            for second in usable[index + 1:]:
                raw_difference = abs(
                    (first[0] - second[0] + math.pi / 2) %
                    math.pi - math.pi / 2)
                orthogonal_error = abs(raw_difference - math.pi / 2)
                score = (
                    max(0.0, 1.0 - orthogonal_error / math.radians(18)) *
                    min(1.0, (first[1] + second[1]) /
                        max(1.0, inner_diameter * 1.2)) *
                    max(0.0, 1.0 - (first[2] + second[2]) /
                        max(1.0, inner_diameter * 0.20)))
                if best is None or score > best[0]:
                    best = score, first[0]
        if best is None:
            return None, 0.0, 1.0
        angle = (best[1] + HALF_PI / 2.0) % HALF_PI - HALF_PI / 2.0
        return angle, min(1.0, best[0]), 0.0

    def _detect_in(self, image, roi):
        x, y, w, h = roi
        crop = image[y:y + h, x:x + w]
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) if crop.ndim == 3 else crop
        mask, blur = self._threshold(gray)
        candidates = self._circle_candidates(mask, (x, y), image.shape)
        best = None
        for outer in candidates:
            if outer['diameter'] < self.config.min_outer_diameter:
                continue
            for inner in candidates:
                if inner is outer or inner['diameter'] >= outer['diameter']:
                    continue
                base = geometry_score(
                    (outer['cx'], outer['cy'], outer['diameter'],
                     outer['circle']),
                    (inner['cx'], inner['cy'], inner['diameter'],
                     inner['circle']), self.config)
                if not base['valid']:
                    continue
                angle, cross_score, cross_error = self._cross(
                    blur, base['center_x_px'], base['center_y_px'],
                    base['inner_diameter_px'], (x, y))
                if angle is None or cross_score < self.config.min_cross_score:
                    continue
                cx, cy = base['center_x_px'], base['center_y_px']
                radius = base['outer_diameter_px'] / 2.0
                border_margin = min(cx, cy, image.shape[1] - cx,
                                    image.shape[0] - cy) - radius
                border_score = max(0.0, min(1.0, border_margin /
                                           max(1.0, radius * 0.15)))
                continuity = 0.5
                if self.last is not None:
                    jump = math.hypot(
                        cx - self.last['center_x_px'],
                        cy - self.last['center_y_px'])
                    continuity = max(
                        0.0, 1.0 - jump /
                        max(1.0, base['outer_diameter_px']))
                confidence = 100.0 * (
                    0.18 * outer['circle'] +
                    0.13 * inner['circle'] +
                    0.14 * base['scores']['ratio'] +
                    0.14 * base['scores']['concentric'] +
                    0.10 * ((outer['coverage'] + inner['coverage']) / 2) +
                    0.20 * cross_score +
                    0.04 * (1.0 - cross_error) +
                    0.04 * border_score + 0.03 * continuity)
                result = dict(base)
                result.update(
                    angle_rad=angle, confidence=max(0.0, min(100.0, confidence)),
                    status='TRACKING', scores={
                        **base['scores'], 'cross': cross_score,
                        'border': border_score, 'continuity': continuity,
                    })
                result['valid'] = int(
                    result['confidence'] >= self.config.min_confidence)
                if best is None or result['confidence'] > best['confidence']:
                    best = result
        return best

    def detect(self, image):
        started = time.perf_counter()
        if image is None or not isinstance(image, np.ndarray) or image.size == 0:
            return invalid_result('IMAGE_INVALID')
        roi = self._tracking_roi(image.shape)
        result = None
        if roi is not None and self.roi_failures < self.config.roi_failures_before_full:
            self.last_search = 'ROI'
            result = self._detect_in(image, roi)
            if result is None or not result['valid']:
                self.roi_failures += 1
        if result is None and (
                roi is None or
                self.roi_failures >= self.config.roi_failures_before_full):
            self.last_search = 'FULL'
            result = self._detect_in(
                image, (0, 0, image.shape[1], image.shape[0]))
        elapsed = (time.perf_counter() - started) * 1000.0
        if result is None:
            return invalid_result('CROSS_INVALID', elapsed)
        result['processing_ms'] = elapsed
        if result['valid']:
            self.last = dict(result)
            self.roi_failures = 0
        return result


def draw_debug(image, result):
    output = image.copy()
    status = result.get('status', 'LOST')
    if result.get('valid'):
        center = (round(result['center_x_px']), round(result['center_y_px']))
        cv2.circle(output, center, round(result['outer_diameter_px'] / 2),
                   (0, 255, 255), 2)
        cv2.circle(output, center, round(result['inner_diameter_px'] / 2),
                   (0, 255, 0), 2)
        length = max(10, round(result['inner_diameter_px'] * 0.4))
        angle = result['angle_rad']
        end = (round(center[0] + length * math.cos(angle)),
               round(center[1] + length * math.sin(angle)))
        cv2.line(output, center, end, (255, 0, 255), 2)
    cv2.putText(
        output, '%s conf=%.1f' % (status, result.get('confidence', 0.0)),
        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    return output
