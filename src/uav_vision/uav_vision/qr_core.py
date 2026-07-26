"""QR shelf detection, decoding, inventory and mission logic without ROS."""

from collections import Counter, deque
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

import cv2
import numpy as np


VALID_IDS = set(range(1, 25))


@dataclass
class QRObservation:
    """One decoded QR observation."""

    qr_id: int
    bbox: tuple
    center: tuple
    confidence: float
    timestamp: float
    row: int = -1
    column: int = -1

    def detection_array(self):
        """Return the established seven-field detection representation."""
        x, y, width, height = self.bbox
        return [1.0, float(self.center[0]), float(self.center[1]),
                float(width), float(height), float(width * height),
                float(np.clip(self.confidence * 100.0, 0.0, 100.0))]


class QRDecoder:
    """Decode numeric shelf QR codes with guarded preprocessing fallbacks."""

    def __init__(self):
        self.detector = cv2.QRCodeDetector()

    @staticmethod
    def parse(payload):
        """Only accept canonical decimal identifiers 1 through 24."""
        text = str(payload).strip()
        if not text.isdigit() or str(int(text)) != text:
            return None
        value = int(text)
        return value if value in VALID_IDS else None

    @staticmethod
    def safe_crop(image, bbox, padding=4):
        """Clip a bounding box to the image; return an empty array if invalid."""
        if image is None or not hasattr(image, 'shape') or image.size == 0:
            return np.empty((0, 0), dtype=np.uint8)
        x, y, width, height = [int(round(v)) for v in bbox]
        x0, y0 = max(0, x - padding), max(0, y - padding)
        x1 = min(image.shape[1], x + width + padding)
        y1 = min(image.shape[0], y + height + padding)
        return image[y0:y1, x0:x1] if x1 > x0 and y1 > y0 else \
            np.empty((0, 0), dtype=np.uint8)

    @staticmethod
    def variants(crop):
        """Generate bounded enhancement variants for difficult QR crops."""
        if crop is None or crop.size == 0:
            return []
        gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY) \
            if crop.ndim == 3 else crop
        scale = max(1.0, 320.0 / max(gray.shape))
        large = cv2.resize(gray, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_CUBIC)
        clahe = cv2.createCLAHE(2.0, (8, 8)).apply(large)
        binary = cv2.adaptiveThreshold(
            clahe, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, 31, 5)
        return [crop, large, clahe, binary]

    def decode_crop(self, crop):
        """Try original, enlarged, CLAHE and adaptive-threshold variants."""
        for variant in self.variants(crop):
            try:
                text, points, _ = self.detector.detectAndDecode(variant)
            except cv2.error:
                continue
            qr_id = self.parse(text)
            if qr_id is not None:
                return qr_id, points
        return None, None

    def decode_full(self, image):
        """Decode all valid QR codes in an image, protecting corrupt inputs."""
        if image is None or not hasattr(image, 'shape') or image.size == 0:
            return []
        results = []
        try:
            ok, texts, points, _ = self.detector.detectAndDecodeMulti(image)
        except (cv2.error, ValueError):
            ok, texts, points = False, (), None
        if ok and points is not None:
            for text, corners in zip(texts, points):
                qr_id = self.parse(text)
                if qr_id is None:
                    continue
                x, y, width, height = cv2.boundingRect(
                    np.asarray(corners, dtype=np.float32))
                results.append((qr_id, (x, y, width, height), corners))
        if results:
            return results
        try:
            text, corners, _ = self.detector.detectAndDecode(image)
        except cv2.error:
            return []
        qr_id = self.parse(text)
        if qr_id is not None and corners is not None:
            corners = np.asarray(corners).reshape(-1, 2)
            x, y, width, height = cv2.boundingRect(
                corners.astype(np.float32))
            results.append((qr_id, (x, y, width, height), corners))
        if not results and max(image.shape[:2]) <= 800:
            # Gazebo and Raspberry Pi preview streams often contain
            # physically small but sharp codes. Upscaling the whole frame
            # is bounded and considerably cheaper than learned sliding
            # windows.
            enlarged = cv2.resize(
                image, None, fx=2.0, fy=2.0,
                interpolation=cv2.INTER_CUBIC)
            try:
                ok, texts, points, _ = \
                    self.detector.detectAndDecodeMulti(enlarged)
            except (cv2.error, ValueError):
                ok, texts, points = False, (), None
            if ok and points is not None:
                for text, corners in zip(texts, points):
                    qr_id = self.parse(text)
                    if qr_id is None:
                        continue
                    corners = np.asarray(corners, np.float32) / 2.0
                    x, y, width, height = cv2.boundingRect(corners)
                    results.append(
                        (qr_id, (x, y, width, height), corners))
        return results


class LearnedRegionDetector:
    """Sliding-window HOG+SVM QR locator trained by the bundled tool."""

    def __init__(self, model_path='', threshold=0.0):
        self.threshold = float(threshold)
        self.model = None
        if model_path and Path(model_path).is_file():
            try:
                self.model = cv2.ml.SVM_load(str(model_path))
            except cv2.error:
                self.model = None
        self.hog = cv2.HOGDescriptor(
            (64, 64), (16, 16), (8, 8), (8, 8), 9)

    @property
    def available(self):
        return self.model is not None

    def detect(self, image):
        """Return NMS-filtered boxes scored by a compact learned classifier."""
        if not self.available or image is None or image.size == 0:
            return []
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) \
            if image.ndim == 3 else image
        boxes, scores = [], []
        shortest = min(gray.shape)
        for size in (48, 64, 96, 128, 160, 224):
            if size > shortest:
                continue
            stride = max(12, size // 3)
            for y in range(0, gray.shape[0] - size + 1, stride):
                for x in range(0, gray.shape[1] - size + 1, stride):
                    patch = cv2.resize(
                        gray[y:y + size, x:x + size], (64, 64))
                    feature = self.hog.compute(patch).reshape(1, -1)
                    _, raw = self.model.predict(
                        feature, flags=cv2.ml.STAT_MODEL_RAW_OUTPUT)
                    score = -float(raw[0, 0])
                    if score >= self.threshold:
                        boxes.append([x, y, size, size])
                        scores.append(float(1.0 / (1.0 + math.exp(-score))))
        keep = cv2.dnn.NMSBoxes(boxes, scores, 0.35, 0.25)
        return [(tuple(boxes[int(i)]), scores[int(i)])
                for i in np.asarray(keep).reshape(-1)] if len(keep) else []


class HybridQRDetector:
    """Traditional decoder plus optional learned localization."""

    def __init__(self, backend='hybrid', model_path='',
                 confidence_threshold=0.35):
        if backend not in ('opencv', 'model', 'hybrid'):
            raise ValueError('backend must be opencv, model, or hybrid')
        self.backend = backend
        self.decoder = QRDecoder()
        self.learned = LearnedRegionDetector(model_path)
        self.confidence_threshold = float(confidence_threshold)
        self.fallback_reason = '' if self.learned.available else \
            'model unavailable; OpenCV fallback active'

    def detect(self, image, timestamp=0.0):
        """Locate and decode, de-duplicating IDs and rejecting bad geometry."""
        if image is None or not hasattr(image, 'shape') or image.size == 0:
            return []
        found = {}
        if self.backend in ('model', 'hybrid') and self.learned.available:
            for bbox, confidence in self.learned.detect(image):
                if confidence < self.confidence_threshold:
                    continue
                qr_id, _ = self.decoder.decode_crop(
                    self.decoder.safe_crop(image, bbox, padding=12))
                if qr_id is not None:
                    found[qr_id] = self._observation(
                        qr_id, bbox, confidence, timestamp)
        # A model backend must remain operational when weights fail to load.
        if self.backend in ('opencv', 'hybrid') or not self.learned.available:
            for qr_id, bbox, _ in self.decoder.decode_full(image):
                found[qr_id] = self._observation(
                    qr_id, bbox, 1.0, timestamp)
        return list(found.values())

    @staticmethod
    def _observation(qr_id, bbox, confidence, timestamp):
        x, y, width, height = bbox
        return QRObservation(
            qr_id, (int(x), int(y), int(width), int(height)),
            (float(x + width / 2), float(y + height / 2)),
            float(confidence), float(timestamp))


class QRInventory:
    """Multi-frame confirmation and de-duplicated shelf inventory."""

    def __init__(self, layout=None, confirm_frames=3, timeout=30.0,
                 image_timeout=0.5, target_qr_id=1,
                 inventory_mode='target'):
        if target_qr_id not in VALID_IDS:
            raise ValueError('target_qr_id must be 1..24')
        if inventory_mode not in ('target', 'full', 'timeout'):
            raise ValueError('inventory_mode must be target, full, or timeout')
        self.layout = layout or {}
        self.confirm_frames = max(1, int(confirm_frames))
        self.timeout = float(timeout)
        self.image_timeout = float(image_timeout)
        self.target_qr_id = int(target_qr_id)
        self.inventory_mode = inventory_mode
        self.pending = {}
        self.records = {}
        self.start_time = None
        self.last_image_time = None
        self.image_size = None

    def update(self, observations, now):
        """Merge one frame and confirm only consecutive repeated results."""
        now = float(now)
        self.start_time = now if self.start_time is None else self.start_time
        self.last_image_time = now
        seen = set()
        for obs in observations:
            if obs.qr_id not in VALID_IDS:
                continue
            seen.add(obs.qr_id)
            history = self.pending.setdefault(obs.qr_id, deque(
                maxlen=self.confirm_frames))
            history.append(obs)
            if len(history) < self.confirm_frames:
                continue
            entry = self.records.get(obs.qr_id)
            position = self.layout.get(obs.qr_id, {})
            if entry is None:
                entry = {
                    'qr_id': obs.qr_id, 'first_seen': now,
                    'observations': 0, 'confirmed_frames': 0}
                self.records[obs.qr_id] = entry
            entry.update({
                'last_seen': now,
                'confirmed_frames': len(history),
                'image_center': list(obs.center),
                'bbox': list(obs.bbox),
                'area': obs.bbox[2] * obs.bbox[3],
                'row': position.get('row', -1),
                'column': position.get('column', -1),
                'available': True,
                'is_target': obs.qr_id == self.target_qr_id,
            })
            entry['observations'] += 1
        for qr_id in list(self.pending):
            if qr_id not in seen:
                self.pending[qr_id].clear()
        return self.records

    def image_stale(self, now):
        return self.last_image_time is None or \
            float(now) - self.last_image_time > self.image_timeout

    def complete(self, now):
        if self.inventory_mode == 'target':
            return self.target_qr_id in self.records
        if self.inventory_mode == 'full':
            return len(self.records) == 24
        return len(self.records) == 24 or (
            self.start_time is not None and
            float(now) - self.start_time >= self.timeout)

    def to_json(self):
        return json.dumps(
            {'count': len(self.records),
             'target_qr_id': self.target_qr_id,
             'complete': self.complete(
                 self.last_image_time if self.last_image_time is not None
                 else 0.0),
             'image_size': list(self.image_size) if self.image_size else [],
             'records': list(self.records.values())},
            ensure_ascii=False, sort_keys=True)


class LaserAlignment:
    """Software-only laser alignment confirmation."""

    def __init__(self, threshold_px=12.0, confirm_frames=5):
        self.threshold_px = float(threshold_px)
        self.confirm_frames = max(1, int(confirm_frames))
        self.count = 0

    def update(self, error_x, error_y, valid=True):
        error = math.hypot(float(error_x), float(error_y))
        self.count = self.count + 1 if valid and \
            error <= self.threshold_px else 0
        return self.count >= self.confirm_frames


def load_layout(path):
    """Load the small YAML layout without making PyYAML a runtime hard-fail."""
    import yaml
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    return {int(item['id']): item for item in data['qr_codes']}
