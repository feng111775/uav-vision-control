import os
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
RECONNECT = ROOT / 'scripts/deploy/test_vision_reconnect.sh'
CLEANUP = ROOT / 'scripts/deploy/cleanup_readonly_vision_test.sh'


def test_scripts_are_shell_valid_and_use_process_group_cleanup():
    for script in (RECONNECT, CLEANUP):
        assert subprocess.run(['bash', '-n', str(script)], check=False).returncode == 0
    text = RECONNECT.read_text()
    assert 'setsid ros2 launch uav_vision h7_v2_readonly.launch.py' in text
    assert 'kill -TERM -- "-$launch_pgid"' in text
    assert 'trap cleanup EXIT INT TERM HUP' in text
    assert 'cleanup_status' in text


def test_cleanup_has_no_broad_kill_patterns():
    text = CLEANUP.read_text()
    for forbidden in ('pkill -f python', 'pkill -f ros2', 'killall python3'):
        assert forbidden not in text
    assert '--check-only' in text and '--cleanup' in text


def test_cleanup_check_only_does_not_modify_processes():
    env = dict(os.environ, D_TASK_WORKSPACE=str(ROOT))
    result = subprocess.run([str(CLEANUP), '--check-only'], env=env,
                            capture_output=True, text=True, check=False)
    assert result.returncode == 0
    assert 'readonly_vision_processes:' in result.stdout
    assert 'cleanup_status' not in result.stdout
