"""Tests for deterministic dual-camera detection selection."""

import pytest

from uav_vision.camera_selector_node import CameraSelector
from uav_vision.camera_selector_node import INVALID_DETECTION


def detection(valid=True, area=1000.0, width=40.0, height=40.0,
              image_width=320.0, image_height=240.0):
    """Build one valid nine-field detection."""
    if not valid:
        result = list(INVALID_DETECTION)
        result[7:] = [image_width, image_height]
        return result
    return [
        1.0, image_width / 2.0, image_height / 2.0,
        width, height, area, 90.0, image_width, image_height]


@pytest.mark.parametrize(('mode', 'expected_camera'), [
    ('front', 'front'),
    ('down', 'down'),
])
def test_fixed_modes_select_only_requested_camera(mode, expected_camera):
    """Fixed modes must ignore the other fresh source."""
    selector = CameraSelector(mode=mode)
    selector.update_front(detection(area=1111.0), 0.0)
    selector.update_down(detection(area=2222.0), 0.0)
    camera, _, selected = selector.selected(0.1)
    assert camera == expected_camera
    expected_area = 1111.0 if mode == 'front' else 2222.0
    assert selected[5] == expected_area


def test_auto_starts_with_front_camera():
    """Auto mode must begin in SEARCH using front detection."""
    selector = CameraSelector()
    selector.update_front(detection(area=1000.0), 0.0)
    selector.update_down(detection(area=2000.0), 0.0)
    camera, state, selected = selector.selected(0.1)
    assert camera == 'front'
    assert state == selector.SEARCH
    assert selected[5] == 1000.0


def test_front_confirmation_switches_to_down_then_aligns():
    """A close front target then confirmed down target enters ALIGN."""
    selector = CameraSelector(
        front_confirm_frames=3, front_area_ratio_threshold=0.03,
        down_confirm_frames=2)
    for index in range(3):
        selector.update_front(detection(area=3000.0), index * 0.01)
    assert selector.state == selector.DOWN_ACQUIRE
    assert selector.selected_camera == 'front'

    selector.update_down(detection(area=800.0), 0.04)
    assert selector.state == selector.DOWN_ACQUIRE
    selector.update_down(detection(area=900.0), 0.05)
    assert selector.state == selector.ALIGN
    assert selector.selected(0.06)[0] == 'down'


def test_down_acquire_keeps_fresh_front_control_until_down_confirmed():
    """DOWN_ACQUIRE must not forward an invalid down stream to control."""
    selector = CameraSelector(
        front_confirm_frames=1, front_area_ratio_threshold=0.03,
        down_confirm_frames=2, down_lost_frames=4)
    selector.update_front(detection(area=3000.0), 0.0)
    assert selector.state == selector.DOWN_ACQUIRE
    for index in range(8):
        now = 0.01 + index * 0.01
        selector.update_down(detection(valid=False), now)
        selector.update_front(detection(area=3100.0), now)
        camera, state, selected = selector.selected(now)
        assert camera == 'front'
        assert state == selector.DOWN_ACQUIRE
        assert selected[0] == 1.0


def test_down_acquire_waits_safely_through_front_loss():
    """Front loss during handoff must stop motion without cancelling acquire."""
    selector = CameraSelector(
        front_confirm_frames=1, front_area_ratio_threshold=0.03,
        down_confirm_frames=2)
    selector.update_front(detection(area=3000.0), 0.0)
    selector.update_front(detection(valid=False), 0.01)
    camera, state, selected = selector.selected(0.01)
    assert camera == 'front'
    assert state == selector.DOWN_ACQUIRE
    assert selected == INVALID_DETECTION

    selector.update_down(detection(area=800.0), 0.02)
    selector.update_down(detection(area=900.0), 0.03)
    assert selector.state == selector.ALIGN
    assert selector.selected(0.03)[0] == 'down'


def test_sustained_down_loss_returns_to_front_search():
    """Down loss hysteresis must eventually return to front SEARCH."""
    selector = CameraSelector(
        front_confirm_frames=1, down_confirm_frames=1,
        down_hold_frames=2, down_lost_frames=4,
        front_area_ratio_threshold=0.03)
    selector.update_front(detection(area=3000.0), 0.0)
    selector.update_down(detection(area=900.0), 0.01)
    assert selector.state == selector.ALIGN
    for index in range(4):
        selector.update_down(detection(valid=False), 0.02 + index * 0.01)
    assert selector.state == selector.SEARCH
    assert selector.selected_camera == 'front'


def test_down_failure_cooldown_prevents_immediate_reswitch():
    """Persistent close front frames must respect down retry cooldown."""
    selector = CameraSelector(
        front_confirm_frames=1, down_confirm_frames=1,
        down_hold_frames=1, down_lost_frames=2,
        front_area_ratio_threshold=0.03, switch_cooldown=2.0)
    selector.update_front(detection(area=3000.0), 0.0)
    selector.update_down(detection(area=900.0), 0.05)
    selector.update_down(detection(valid=False), 0.1)
    selector.update_down(detection(valid=False), 0.2)
    assert selector.state == selector.SEARCH

    selector.update_front(detection(area=3000.0), 1.0)
    assert selector.state == selector.SEARCH
    selector.update_front(detection(area=3000.0), 2.21)
    assert selector.state == selector.DOWN_ACQUIRE


def test_short_down_loss_holds_without_camera_chatter():
    """Short loss and repeated close front frames cannot flap selection."""
    selector = CameraSelector(
        front_confirm_frames=2, down_confirm_frames=1,
        down_hold_frames=2, down_lost_frames=5,
        front_area_ratio_threshold=0.03)
    selector.update_front(detection(area=3000.0), 0.0)
    selector.update_front(detection(area=3000.0), 0.01)
    selector.update_down(detection(area=900.0), 0.02)
    original = selector.selected(0.02)[2]

    selector.update_down(detection(valid=False), 0.03)
    selector.update_front(detection(area=4000.0), 0.03)
    assert selector.selected(0.03)[0] == 'down'
    assert selector.selected(0.03)[2] == original
    selector.update_down(detection(valid=False), 0.04)
    assert selector.selected(0.04)[0] == 'down'
    assert selector.state == selector.ALIGN


def test_both_sources_stale_output_invalid():
    """No stale detection may reach downstream control."""
    selector = CameraSelector(mode='front', source_timeout=0.3)
    selector.update_front(detection(area=1200.0), 1.0)
    selector.update_down(detection(area=2200.0), 1.0)
    assert selector.selected(1.31)[2] == INVALID_DETECTION

    down = CameraSelector(mode='down', source_timeout=0.3)
    down.update_down(detection(area=2200.0), 1.0)
    assert down.selected(1.31)[2] == INVALID_DETECTION


def test_stale_down_source_returns_auto_mode_to_front():
    """A timed-out down stream must fall back to fresh front search."""
    selector = CameraSelector(
        front_confirm_frames=1, down_confirm_frames=1,
        source_timeout=0.3)
    selector.update_front(detection(area=20000.0, width=150.0), 1.0)
    selector.update_down(detection(area=900.0), 1.0)
    selector.update_front(detection(area=1000.0), 1.25)
    camera, state, selected = selector.selected(1.31)
    assert camera == 'front'
    assert state == selector.SEARCH
    assert selected[0] == 1.0


def test_only_selected_detection_enters_output():
    """Output must contain one source, never a merged pair."""
    selector = CameraSelector(mode='front')
    selector.update_front(detection(area=1111.0), 0.0)
    selector.update_down(detection(area=9999.0), 0.0)
    selected = selector.selected(0.1)[2]
    assert len(selected) == 9
    assert selected[5] == 1111.0
    assert 9999.0 not in selected


@pytest.mark.parametrize('mode', ['invalid', '', 'AUTO'])
def test_invalid_mode_is_rejected(mode):
    """Only the documented selection modes are accepted."""
    with pytest.raises(ValueError, match='mode'):
        CameraSelector(mode=mode)


def test_switch_threshold_is_resolution_independent():
    """Equivalent target ratios must switch identically at both sizes."""
    small = CameraSelector(
        front_confirm_frames=1, front_area_ratio_threshold=0.08,
        front_width_ratio_threshold=0.5,
        front_height_ratio_threshold=0.5)
    large = CameraSelector(
        front_confirm_frames=1, front_area_ratio_threshold=0.08,
        front_width_ratio_threshold=0.5,
        front_height_ratio_threshold=0.5)
    small.update_front(detection(
        area=8000.0, width=80.0, height=100.0,
        image_width=320.0, image_height=240.0), 0.0)
    large.update_front(detection(
        area=32000.0, width=160.0, height=200.0,
        image_width=640.0, image_height=480.0), 0.0)
    assert small.state == small.DOWN_ACQUIRE
    assert large.state == large.DOWN_ACQUIRE
