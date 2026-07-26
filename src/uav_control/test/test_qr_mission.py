"""Tests for the sequential sensor-event-driven QR mission."""

import math

import pytest

from uav_control.qr_mission import QRMission


def airborne(mission):
    mission.start(0.0)
    mission.step(0.1, 'TAKEOFF')
    mission.step(0.2, 'VISION_CONTROL')
    assert mission.state == mission.QR_SCAN_MOVE


def test_scan_order_and_generated_points():
    mission = QRMission()
    assert mission.scan_order == (
        1, 2, 3, 4, 5, 6, 12, 11, 10, 9, 8, 7,
        13, 14, 15, 16, 17, 18, 24, 23, 22, 21, 20, 19)
    assert mission.scan_point() == pytest.approx((-1.25, .78, -2.5))


def test_scan_requires_arrival_hold_and_inventory_confirmation():
    mission = QRMission(scan_hold_seconds=.5)
    airborne(mission)
    mission.update_position(*mission.scan_point())
    mission.step(.3, 'VISION_CONTROL')
    assert mission.state == mission.QR_SCAN_HOLD
    mission.step(.7, 'VISION_CONTROL')
    assert mission.state == mission.QR_SCAN_HOLD
    mission.step(.81, 'VISION_CONTROL')
    assert mission.state == mission.QR_SCAN_CONFIRM
    mission.update_inventory(False, False, 0, 0, .82, scan_index=1)
    mission.step(.82, 'VISION_CONTROL')
    assert mission.state == mission.QR_SCAN_NEXT
    mission.step(.83, 'VISION_CONTROL')
    assert mission.state == mission.QR_SCAN_MOVE


def test_timeout_retries_then_failsafe_without_skipping():
    mission = QRMission(
        scan_hold_seconds=.1, scan_timeout=.2, scan_max_retries=1)
    airborne(mission)
    mission.update_position(*mission.scan_point())
    mission.step(.3, 'VISION_CONTROL')
    mission.step(.41, 'VISION_CONTROL')
    mission.step(.62, 'VISION_CONTROL')
    assert mission.state == mission.QR_SCAN_MOVE
    assert mission.scan_index == 0 and mission.retry_count == 1
    mission.update_position(*mission.scan_point())
    mission.step(.7, 'VISION_CONTROL')
    mission.step(.81, 'VISION_CONTROL')
    mission.step(1.02, 'VISION_CONTROL')
    assert mission.state == mission.FAILSAFE
    assert mission.scan_index == 0


def test_camera_silence_during_confirmation_retries_then_failsafe():
    mission = QRMission(
        scan_hold_seconds=.1, scan_timeout=.2, scan_max_retries=0)
    airborne(mission)
    mission.update_position(*mission.scan_point())
    mission.step(.3, 'VISION_CONTROL')
    mission.step(.41, 'VISION_CONTROL')
    assert mission.state == mission.QR_SCAN_CONFIRM
    # No inventory callback follows: a stopped camera cannot confirm.
    mission.step(.62, 'VISION_CONTROL')
    assert mission.state == mission.FAILSAFE
    assert mission.scan_index == 0


def test_incomplete_inventory_cannot_enter_target_approach():
    mission = QRMission()
    airborne(mission)
    mission.update_inventory(False, True, 1000, 0, .3, scan_index=23)
    mission.step(.3, 'VISION_CONTROL')
    assert mission.state in (mission.QR_SCAN_MOVE, mission.QR_SCAN_HOLD)


def test_inventory_progress_is_accepted_before_target_is_seen():
    mission = QRMission(target_qr_id=10)
    mission.update_inventory(
        False, False, 0, math.inf, .3, scan_index=1,
        target_visible=False)
    assert mission.inventory_count == 1


def test_complete_inventory_selects_configured_target():
    mission = QRMission(target_qr_id=10)
    airborne(mission)
    mission.scan_index = 23
    mission.inventory_count = 23
    mission.state = mission.QR_SCAN_CONFIRM
    mission.state_since = .2
    mission.update_inventory(True, True, 1000, 0, .3, scan_index=24)
    mission.step(.3, 'VISION_CONTROL')
    assert mission.state == mission.QR_INVENTORY_COMPLETE
    mission.step(.4, 'VISION_CONTROL')
    assert mission.state == mission.TARGET_ACQUIRE
    assert mission.target_point() == pytest.approx((.25, .78, -2.0))


def test_missing_target_after_complete_is_safe_failure():
    mission = QRMission(target_qr_id=10)
    mission.start(0)
    mission.state = mission.QR_INVENTORY_COMPLETE
    mission.state_since = 0
    mission.inventory_complete = True
    mission.inventory_count = 24
    mission.target_seen = False
    mission.step(.1, 'VISION_CONTROL')
    assert mission.state == mission.FAILSAFE


@pytest.mark.parametrize('target', [0, 25])
def test_invalid_target_id(target):
    with pytest.raises(ValueError):
        QRMission(target_qr_id=target)
