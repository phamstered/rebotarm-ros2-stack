"""Run the startup script's command-subscriber gate with delayed ROS discovery."""
from pathlib import Path
import os
import signal
import shlex
import subprocess
import sys
import time

import pytest


SCRIPT = Path(__file__).parents[4] / 'scripts' / 'start_arm_teleop.sh'


def run_gate(tmp_path, subscribers):
    script = SCRIPT.read_text()
    gate = script.split('subscriber_deadline=$((SECONDS + RELAY_START_TIMEOUT))', 1)[1]
    gate = 'subscriber_deadline=$((SECONDS + RELAY_START_TIMEOUT))' + gate.split(
        'ack_deadline=$((SECONDS + STM32_STATE_TIMEOUT))', 1)[0]
    (tmp_path / 'count').write_text('0')
    mock = '''
        RELAY_START_TIMEOUT=4
        timeout() { shift; "$@"; }
        sleep() { SECONDS=$((SECONDS + 1)); }
        pass() { echo "$*"; }
        ros2() {
            echo "$*" >> "$ROOT/calls"
            n=$(cat "$ROOT/count")
            echo $((n + 1)) > "$ROOT/count"
            if (( n == 0 )); then echo 'Subscription count: 0';
            else echo "Subscription count: $SUBSCRIBERS"; fi
        }
    '''
    result = subprocess.run(['bash', '-c', mock + gate], text=True, capture_output=True,
                            env=dict(os.environ, ROOT=str(tmp_path), SUBSCRIBERS=str(subscribers)),
                            timeout=5)
    return result, (tmp_path / 'calls').read_text()


def test_retries_direct_discovery_instead_of_accepting_stale_daemon_cache(tmp_path):
    result, calls = run_gate(tmp_path, 1)
    assert result.returncode == 0, result.stderr
    assert calls.count('--no-daemon --spin-time 1') == 2
    assert 'has 1 subscriber' in result.stdout


def test_missing_subscriber_still_blocks_startup_after_bounded_wait(tmp_path):
    result, calls = run_gate(tmp_path, 0)
    assert result.returncode != 0
    assert calls.count('--no-daemon --spin-time 1') == 4
    assert 'discovery timeout' in result.stderr


@pytest.mark.parametrize('safe_stop_accepts', [True, False])
def test_signal_cleanup_reaps_owned_groups_only_after_safe_stop(tmp_path, safe_stop_accepts):
    source = SCRIPT.read_text()
    functions = 'stop_matching_processes() {' + source.split(
        'stop_matching_processes() {', 1)[1].split(
            'stage "ROS environment', 1)[0]
    # Model ROS launchers that terminate with SIGINT status while the shell is
    # waiting for the same relay process. No ROS topics or production PIDs.
    child = shlex.quote(sys.executable) + ' -c ' + shlex.quote(
        'import signal,sys,time; '
        'signal.signal(signal.SIGINT, lambda *_: sys.exit(130)); time.sleep(60)')
    script = tmp_path / 'supervisor.sh'
    script.write_text('set -euxo pipefail\n' + functions + '''
pgrep() { return 1; }
''' + f'safe_stop_ros2_control() {{ return {0 if safe_stop_accepts else 1}; }}\n' + '\n'.join(f'setsid {child} & {name}=$!' for name in (
        'RELAY_PID', 'LEADER_PID', 'CONTROLLER_PID')) + '''
printf '%s %s %s' "$RELAY_PID" "$LEADER_PID" "$CONTROLLER_PID" > "$1/children"
wait "$RELAY_PID"
''')
    children = []
    with (tmp_path / 'trace').open('w') as trace:
        process = subprocess.Popen(['bash', str(script), str(tmp_path)],
                                   stdout=trace, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            deadline = time.monotonic() + 3
            while not (tmp_path / 'children').exists() and time.monotonic() < deadline:
                time.sleep(0.01)
            children = [int(pid) for pid in (tmp_path / 'children').read_text().split()]
            time.sleep(0.2)
            process.send_signal(signal.SIGTERM)
            process.wait(timeout=15)
            survivors = []
            for pid in children:
                try:
                    os.kill(pid, 0)
                    survivors.append(pid)
                except ProcessLookupError:
                    pass
            if safe_stop_accepts:
                assert not survivors, (tmp_path / 'trace').read_text()
            else:
                assert len(survivors) == 3, (tmp_path / 'trace').read_text()
                assert 'Refusing cleanup' in (tmp_path / 'trace').read_text()
        finally:
            for pid in children:
                try:
                    os.killpg(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
