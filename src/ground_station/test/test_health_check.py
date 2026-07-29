from ground_station.health_check_node import health_status


def test_health_status_is_ready_read_only_and_offline():
    status = health_status()

    assert status["status"] == "READY"
    assert status["mode"] == "READ_ONLY"
    assert status["offline"] is True


def test_health_status_has_no_control_commands():
    status = health_status()

    assert status["control_commands"] == ()
    assert not any(key.startswith(("send_", "publish_", "command_")) for key in status)
