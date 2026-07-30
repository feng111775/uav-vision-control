"""CPython tests for the standalone OpenMV LAB red-assist interface."""

import pathlib
import sys
import unittest


MODULE_DIR = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))

from red_detector import RED_LAB_THRESHOLDS, detect_red_center  # noqa: E402


class FakeBlob:
    def __init__(self, x, y, width, height, pixels, area=None):
        self._items = (x, y, width, height, 0, pixels,
                       width * height if area is None else area)

    def __getitem__(self, index):
        return self._items[index]

    def x(self):
        return self._items[0]

    def y(self):
        return self._items[1]

    def w(self):
        return self._items[2]

    def h(self):
        return self._items[3]

    def pixels(self):
        return self._items[5]

    def area(self):
        return self._items[6]


class FakeImage:
    def __init__(self, blobs=(), width=320, height=240):
        self.blobs = list(blobs)
        self.width_value = width
        self.height_value = height
        self.thresholds = None

    def width(self):
        return self.width_value

    def height(self):
        return self.height_value

    def find_blobs(self, thresholds, **kwargs):
        self.thresholds = thresholds
        return self.blobs


class RedDetectorTest(unittest.TestCase):
    def test_returns_invalid_without_red_blob(self):
        result = detect_red_center(FakeImage())
        self.assertEqual(result, {
            "valid": 0, "cx": 0, "cy": 0, "area": 0, "confidence": 0,
        })

    def test_returns_red_region_centre_and_lab_thresholds(self):
        image = FakeImage([FakeBlob(120, 80, 30, 28, 760, 840)])
        result = detect_red_center(image)
        self.assertEqual(image.thresholds, RED_LAB_THRESHOLDS)
        self.assertEqual((result["cx"], result["cy"]), (135, 94))
        self.assertEqual(result["area"], 840)
        self.assertEqual(result["valid"], 1)
        self.assertGreaterEqual(result["confidence"], 45)
        self.assertLessEqual(result["confidence"], 100)

    def test_rejects_thin_background_interference(self):
        image = FakeImage([
            FakeBlob(5, 5, 90, 6, 450, 540),
            FakeBlob(150, 100, 20, 20, 360, 400),
        ])
        result = detect_red_center(image)
        self.assertEqual((result["cx"], result["cy"]), (160, 110))
        self.assertEqual(result["valid"], 1)

    def test_malformed_image_is_safe(self):
        self.assertEqual(detect_red_center(None)["valid"], 0)


if __name__ == "__main__":
    unittest.main()
