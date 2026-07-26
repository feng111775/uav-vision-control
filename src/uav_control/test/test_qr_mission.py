"""Tests for the sensor-event-driven QR mission."""

import math

import pytest

from uav_control.qr_mission import QRMission


def reach_approach(mission):
    mission.start(0.0)
    mission.update_flight(True, False, 0.1)
    mission.step(0.1, 'TAKEOFF')
    mission.step(0.2, 'VISION_CONTROL')
    mission.update_inventory(True, True, 1000, 20, 0.3)
    mission.step(0.3, 'VISION_CONTROL')
    mission.step(0.4, 'VISION_CONTROL')
    mission.step(0.5, 'VISION_CONTROL')
    assert mission.state == mission.TARGET_APPROACH


def test_complete_event_driven_mission():
    mission = QRMission(
        event_timeout=2, transit_seconds=1, align_error=10)
    reach_approach(mission)
    mission.update_inventory(True, True, 20000, 5, 0.6)
    mission.step(0.6, 'VISION_CONTROL')
    mission.update_laser(True, 0.7)
    mission.step(0.7, 'VISION_CONTROL')
    mission.step(0.8, 'VISION_CONTROL')
    mission.step(1.9, 'VISION_CONTROL')
    mission.update_camera('down', 2.0)
    mission.step(2.0, 'VISION_CONTROL')
    mission.update_down(True, 5, 2.1)
    output = mission.step(2.1, 'VISION_CONTROL')
    assert mission.state == mission.LAND and output.request_land
    mission.update_flight(True, True, 2.2)
    output = mission.step(2.2, 'VISION_CONTROL')
    assert mission.state == mission.DISARM and output.request_disarm
    mission.update_flight(False, True, 2.3)
    assert not mission.step(2.3, 'VISION_CONTROL').request_disarm


def test_target_loss_returns_to_acquire():
    mission = QRMission(event_timeout=.2)
    reach_approach(mission)
    mission.step(1.0, 'VISION_CONTROL')
    assert mission.state == mission.TARGET_ACQUIRE


def test_laser_jitter_does_not_advance():
    mission = QRMission(event_timeout=1)
    reach_approach(mission)
    mission.update_inventory(True, True, 20000, 5, .6)
    mission.step(.6, 'VISION_CONTROL')
    mission.update_laser(False, .7)
    mission.step(.7, 'VISION_CONTROL')
    assert mission.state == mission.LASER_ALIGN


def test_front_to_down_requires_fresh_feedback():
    mission = QRMission(event_timeout=.5, transit_seconds=.1)
    reach_approach(mission)
    mission.update_inventory(True, True, 20000, 1, .6)
    mission.step(.6, 'VISION_CONTROL')
    mission.update_laser(True, .7)
    mission.step(.7, 'VISION_CONTROL')
    mission.step(.8, 'VISION_CONTROL')
    mission.step(1.0, 'VISION_CONTROL')
    mission.update_camera('down', .1)
    mission.step(1.0, 'VISION_CONTROL')
    assert mission.state == mission.DOWN_ACQUIRE


def test_down_alignment_requires_valid_threshold():
    mission = QRMission(event_timeout=1, align_error=10)
    mission.state = mission.ALIGN
    mission.state_since = mission.started = 0
    mission.update_down(False, 1, .1)
    mission.step(.1, 'VISION_CONTROL')
    mission.update_down(True, 11, .2)
    mission.step(.2, 'VISION_CONTROL')
    assert mission.state == mission.ALIGN


def test_state_and_total_timeout():
    state = QRMission(state_timeout=1, mission_timeout=20)
    state.start(0)
    state.step(2, 'PRESTREAM')
    assert state.state == state.PRESTREAM
    state.step(21, 'PRESTREAM')
    assert state.state == state.FAILSAFE
    total = QRMission(state_timeout=20, mission_timeout=1)
    total.start(0)
    total.step(2, 'PRESTREAM')
    assert total.state == total.FAILSAFE


def test_out_of_order_and_nonfinite_events_are_ignored():
    mission = QRMission()
    mission.update_camera('down', 2)
    mission.update_camera('front', 1)
    assert mission.selected_camera == 'down'
    mission.update_down(True, math.nan, 3)
    assert mission.down_time is None
    mission.update_inventory(True, True, math.inf, 1, 3)
    assert mission.qr_time is None


@pytest.mark.parametrize('target', [0, 25])
def test_invalid_target_id(target):
    with pytest.raises(ValueError):
        QRMission(target_qr_id=target)
