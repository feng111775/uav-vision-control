from pathlib import Path
import subprocess


PACKAGE_ROOT = Path(__file__).parents[1]
HELPER = PACKAGE_ROOT / 'scripts' / '_ros_env.sh'


def run_bash(script):
    return subprocess.run(
        ['bash', '-u', '-c', script],
        check=False, capture_output=True, text=True)


def test_strict_shell_safely_loads_nounset_incompatible_setup(tmp_path):
    setup = tmp_path / 'setup.bash'
    setup.write_text(
        ': "${AMENT_TRACE_SETUP_FILES}"\n'
        'SAFE_SOURCE_MARKER=loaded\n',
        encoding='utf-8')
    result = run_bash(
        f'set -Eeuo pipefail; source "{HELPER}"; '
        'unset AMENT_TRACE_SETUP_FILES; '
        f'safe_source "{setup}"; '
        '[[ "${SAFE_SOURCE_MARKER}" == loaded ]]; '
        'case $- in *u*) exit 0;; *) exit 9;; esac')
    assert result.returncode == 0, result.stderr
    assert 'unbound variable' not in result.stderr


def test_real_ros_setup_loads_with_parent_and_script_nounset():
    result = run_bash(
        f'set -Eeuo pipefail; source "{HELPER}"; '
        'unset AMENT_TRACE_SETUP_FILES; '
        'safe_source /opt/ros/jazzy/setup.bash; '
        'case $- in *u*) exit 0;; *) exit 9;; esac')
    assert result.returncode == 0, result.stderr
    assert 'AMENT_TRACE_SETUP_FILES' not in result.stderr
    assert 'unbound variable' not in result.stderr


def test_source_failure_restores_nounset_and_returns_original_code(tmp_path):
    setup = tmp_path / 'failing_setup.bash'
    setup.write_text('return 23\n', encoding='utf-8')
    result = run_bash(
        f'set -Eeuo pipefail; source "{HELPER}"; '
        f'if safe_source "{setup}"; then exit 8; else rc=$?; fi; '
        'case $- in *u*) ;; *) exit 9;; esac; '
        '[[ "${rc}" -eq 23 ]]')
    assert result.returncode == 0, result.stderr


def test_missing_setup_fails_clearly_and_keeps_nounset(tmp_path):
    missing = tmp_path / 'missing_setup.bash'
    result = run_bash(
        f'set -Eeuo pipefail; source "{HELPER}"; '
        f'if safe_source "{missing}"; then exit 8; else rc=$?; fi; '
        'case $- in *u*) ;; *) exit 9;; esac; '
        '[[ "${rc}" -ne 0 ]]')
    assert result.returncode == 0
    assert 'ERROR: setup file not readable:' in result.stderr
    assert str(missing) in result.stderr


def test_all_strict_ros_scripts_use_safe_source():
    scripts = PACKAGE_ROOT / 'scripts'
    expected = {
        'run_sitl_hover.sh',
        'run_hover_matrix.sh',
        'run_d_task_stage.sh',
    }
    for name in expected:
        text = (scripts / name).read_text(encoding='utf-8')
        assert 'set -Eeuo pipefail' in text
        assert 'source "${script_dir}/_ros_env.sh"' in text
        assert 'safe_source ' in text
        setup_sources = [
            line for line in text.splitlines()
            if line.lstrip().startswith('source ')
            and ('setup.bash' in line or 'local_setup.bash' in line)
        ]
        assert not setup_sources


def test_regression_entry_delegates_to_repaired_runner():
    text = (
        PACKAGE_ROOT / 'scripts' /
        'run_hover_regression.sh').read_text(encoding='utf-8')
    assert '"${script_dir}/run_sitl_hover.sh" nominal 3' in text
