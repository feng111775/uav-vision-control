from d_system_sim.system_check_node import system_status


def test_jazzy_distribution_is_ready():
    assert system_status({"ROS_DISTRO": "jazzy"}) == {
        "status": "READY",
        "ros_distro": "jazzy",
    }


def test_missing_distribution_is_explicit_mismatch():
    assert system_status({}) == {
        "status": "MISMATCH",
        "ros_distro": "unknown",
    }


def test_wrong_distribution_is_explicit_mismatch():
    assert system_status({"ROS_DISTRO": "rolling"}) == {
        "status": "MISMATCH",
        "ros_distro": "rolling",
    }
