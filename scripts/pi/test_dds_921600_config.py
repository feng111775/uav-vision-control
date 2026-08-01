from pathlib import Path


ROOT = Path(__file__).parents[2]


def test_dds_921600_profile_is_explicit_and_separate_from_openmv():
    env = (ROOT / 'scripts/pi/px4_dds_921600.env').read_text()
    script = (ROOT / 'scripts/pi/start_px4_dds_921600.sh').read_text()
    assert 'PX4_DDS_BAUDRATE=921600' in env
    assert 'Micro XRCE-DDS Agent v2.4.3' in env
    assert 'must not be the OpenMV serial device' in script
    assert 'MicroXRCEAgent serial' in script
