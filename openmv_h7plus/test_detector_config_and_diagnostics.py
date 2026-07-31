import ast
import importlib
import io
import sys
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).parent))
import config
import detector
from detector_fast import FastV2Detector


class Image:
    def width(self): return 320
    def height(self): return 240


class Blob:
    def __init__(self, x, y, w, h, roundness=0.5):
        self._values = (x, y, w, h) + (0,) * 14 + (roundness,)
    def __getitem__(self, index): return self._values[index]


class Circle:
    def __init__(self, x, y, r, magnitude=4000):
        self._values = (x, y, r, magnitude)
    def __getitem__(self, index): return self._values[index]


class Threshold:
    def value(self): return 80


class Histogram:
    def get_threshold(self): return Threshold()


class SearchImage(Image):
    def __init__(self, blobs): self._blobs = blobs
    def get_histogram(self, roi=None): return Histogram()
    def find_blobs(self, *args, **kwargs): return self._blobs


class CircleImage(Image):
    def __init__(self, circles): self._circles = circles
    def find_circles(self, **kwargs): return self._circles


def test_config_value_changes_tracking_roi(monkeypatch):
    original = config.ROI_SCALE_PERCENT
    monkeypatch.setattr(config, 'ROI_SCALE_PERCENT', 300)
    reloaded = importlib.reload(detector)
    instance = reloaded.DTaskDetector()
    instance.last = {'cx': 100, 'cy': 100, 'outer_diameter_px': 40}
    roi = instance._tracking_roi(Image())
    assert roi[2] > 60 and roi[3] > 60
    monkeypatch.setattr(config, 'ROI_SCALE_PERCENT', original)
    importlib.reload(detector)


def test_detector_has_no_duplicate_threshold_assignments():
    source = Path(__file__).with_name('detector.py').read_text(encoding='utf-8')
    tree = ast.parse(source)
    duplicated = {
        'RATIO_MIN', 'RATIO_MAX', 'MIN_OUTER_DIAMETER', 'MAX_OUTER_DIAMETER',
        'MIN_CROSS_SCORE', 'MIN_CONFIDENCE', 'MIN_BLOB_PIXELS', 'MIN_BLOB_AREA'
    }
    for node in tree.body:
        if isinstance(node, ast.Assign):
            names = {target.id for target in node.targets if isinstance(target, ast.Name)}
            assert not (names & duplicated)


def test_blob_filters_report_no_blob_and_size_rejects():
    instance = detector.DTaskDetector()
    image = SearchImage([Blob(1, 1, 10, 10)])
    regions = instance._search_regions(image, (0, 0, 320, 240), include_fallback=False)
    assert regions == []
    assert instance.diagnostics.reject_counts['BLOB_TOO_SMALL'] >= 1


def test_wrong_ratio_and_non_concentric_fail_pairs():
    instance = detector.DTaskDetector()
    image = CircleImage([
        Circle(100, 100, 40), Circle(100, 100, 10),
        Circle(150, 150, 35), Circle(170, 170, 20),
    ])
    candidate = {'verify_roi': (0, 0, 240, 240), 'diameter': 80}
    pairs = instance._circle_pairs(image, candidate)
    assert pairs == []
    assert instance.diagnostics.reject_counts['NO_CONCENTRIC_PAIR'] >= 1
    assert instance.diagnostics.reject_counts['RATIO_REJECT'] >= 1 or instance.diagnostics.reject_counts['CONCENTRIC_ERROR'] >= 1


def test_generated_target_geometry_matches_algorithm_window():
    import importlib.util
    tool_path = Path(__file__).resolve().parents[1] / 'tools' / 'generate_openmv_test_target.py'
    spec = importlib.util.spec_from_file_location('generate_openmv_test_target', tool_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    ratio = module.INNER_DIAMETER_MM / module.OUTER_DIAMETER_MM
    assert config.INNER_OUTER_RATIO_MIN <= ratio <= config.INNER_OUTER_RATIO_MAX
    assert module.CROSS_LENGTH_MM < module.INNER_DIAMETER_MM


def test_fast_detector_valid_and_verify_expired_paths():
    region = {
        'blob_bbox': (100, 70, 60, 60), 'verify_roi': (92, 62, 76, 76),
        'cx': 130, 'cy': 100, 'diameter': 60, 'area': 3600,
        'aspect': 1.0, 'roundness': 0.9, 'score': 1.9,
    }
    verified = {'valid': 1, 'cx': 130, 'cy': 100, 'outer_diameter_px': 50,
                'inner_diameter_px': 30, 'angle_rad': .1, 'confidence': 80,
                'status': 'TRACKING'}
    detector_fast = FastV2Detector()
    detector_fast.track.last = {'valid': 1, 'measurement_valid': 1, 'cx': 130,
                                'cy': 100, 'outer_diameter_px': 50}
    detector_fast.last_verified_success_frame = 1
    detector_fast.last_verified_success_result = verified
    detector_fast.verified_track_active = True
    detector_fast._search_regions = Mock(return_value=[region])
    detector_fast.frame = 2
    result = detector_fast.detect(Image(), 'FOLLOW')
    assert result['valid'] == 1
    detector_fast.last_verified_success_frame = 0
    detector_fast.frame = 30
    result = detector_fast.detect(Image(), 'FOLLOW')
    assert result['valid'] == 0
    assert result['status'] in ('VERIFY_EXPIRED', 'CROSS_INVALID')


def test_diagnostics_success_rate_counters_are_bounded():
    report = detector.DetectionStatsReporter()
    stream = io.StringIO()
    report.set_writer(stream)
    report.begin_frame(10, 'SEARCH', True, 1)
    report.note_verify_attempt()
    report.note_verify_success()
    report.note_current_measurement({'diameter': 60, 'cx': 10, 'cy': 20})
    report.finish(True)
    report.emit_if_due(0)
    report.emit_if_due(1200)
    line = stream.getvalue().strip()
    assert 'D_DETECT_STATS' in line
