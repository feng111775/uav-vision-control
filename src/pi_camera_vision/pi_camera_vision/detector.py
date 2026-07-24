"""ROS-independent red-target detection."""

import cv2
import numpy as np


INVALID_DETECTION = [0.0] * 7


class RedTargetDetector:
    """Detect the largest red region and return the H7-compatible fields."""

    def __init__(self, min_area=100.0, morphology_kernel=5,
                 red1_h_min=0, red1_h_max=10,
                 red2_h_min=170, red2_h_max=179,
                 saturation_min=100, value_min=80):
        if min_area < 0:
            raise ValueError('min_area must be non-negative')
        if morphology_kernel < 1 or morphology_kernel % 2 == 0:
            raise ValueError('morphology_kernel must be a positive odd number')
        if not 0 <= red1_h_min <= red1_h_max <= 179:
            raise ValueError('first hue range must be inside [0, 179]')
        if not 0 <= red2_h_min <= red2_h_max <= 179:
            raise ValueError('second hue range must be inside [0, 179]')
        if not 0 <= saturation_min <= 255 or not 0 <= value_min <= 255:
            raise ValueError('saturation/value thresholds must be in [0, 255]')
        self.min_area = float(min_area)
        self.kernel = np.ones(
            (morphology_kernel, morphology_kernel), dtype=np.uint8)
        self.lower_red1 = np.array(
            [red1_h_min, saturation_min, value_min], dtype=np.uint8)
        self.upper_red1 = np.array(
            [red1_h_max, 255, 255], dtype=np.uint8)
        self.lower_red2 = np.array(
            [red2_h_min, saturation_min, value_min], dtype=np.uint8)
        self.upper_red2 = np.array(
            [red2_h_max, 255, 255], dtype=np.uint8)

    def detect(self, image):
        """Return (seven-value detection, annotated BGR image, mask)."""
        if (not isinstance(image, np.ndarray) or image.ndim != 3
                or image.shape[2] != 3 or image.size == 0):
            return list(INVALID_DETECTION), None, None

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.bitwise_or(
            cv2.inRange(hsv, self.lower_red1, self.upper_red1),
            cv2.inRange(hsv, self.lower_red2, self.upper_red2),
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [
            contour for contour in contours
            if cv2.contourArea(contour) >= self.min_area
        ]
        annotated = image.copy()
        if not contours:
            return list(INVALID_DETECTION), annotated, mask

        target = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(target))
        x, y, width, height = cv2.boundingRect(target)
        moments = cv2.moments(target)
        if moments['m00'] > 0:
            cx = moments['m10'] / moments['m00']
            cy = moments['m01'] / moments['m00']
        else:
            cx = x + width / 2.0
            cy = y + height / 2.0

        bounding_area = max(1.0, float(width * height))
        fill_score = min(1.0, area / bounding_area)
        frame_area = float(image.shape[0] * image.shape[1])
        size_score = min(1.0, area / max(1.0, frame_area * 0.05))
        confidence = 100.0 * (0.7 * fill_score + 0.3 * size_score)

        cv2.rectangle(
            annotated, (x, y), (x + width - 1, y + height - 1),
            (0, 255, 0), 2)
        cv2.circle(
            annotated, (int(round(cx)), int(round(cy))), 4, (255, 0, 0), -1)
        result = [
            1.0, float(cx), float(cy), float(width), float(height),
            area, float(max(0.0, min(100.0, confidence))),
        ]
        return result, annotated, mask
