from pathlib import Path


def test_frozen_px4_topics():
    source = (Path(__file__).parents[1] / 'uav_control' /
              'mission_controller_node.py').read_text()
    expected = (
        "VEHICLE_STATUS_TOPIC = '/fmu/out/vehicle_status_v1'",
        "VEHICLE_LOCAL_POSITION_TOPIC = '/fmu/out/vehicle_local_position'",
        "VEHICLE_ATTITUDE_TOPIC = '/fmu/out/vehicle_attitude'",
        "VEHICLE_COMMAND_ACK_TOPIC = '/fmu/out/vehicle_command_ack'",
        "OFFBOARD_CONTROL_MODE_TOPIC = '/fmu/in/offboard_control_mode'",
        "TRAJECTORY_SETPOINT_TOPIC = '/fmu/in/trajectory_setpoint'",
        "VEHICLE_COMMAND_TOPIC = '/fmu/in/vehicle_command'",
    )
    for declaration in expected:
        assert declaration in source
    assert "'/fmu/out/vehicle_local_position_v1'" not in source
