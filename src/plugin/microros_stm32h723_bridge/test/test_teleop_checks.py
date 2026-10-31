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


@pytest.mark.parametrize('safe_stop_mode', ['accepted', 'rejected', 'no_controller'])
def test_signal_cleanup_reaps_owned_groups_only_after_safe_stop(tmp_path, safe_stop_mode):
    source = SCRIPT.read_text()
    functions = 'safe_stop_ros2_control() {' + source.split(
        'safe_stop_ros2_control() {', 1)[1].split(
            'stage "ROS environment', 1)[0]
    # Model ROS launchers that terminate with SIGINT status while the shell is
    # waiting for the same relay process. No ROS topics or production PIDs.
    child = shlex.quote(sys.executable) + ' -c ' + shlex.quote(
        'import signal,sys,time; '
        'signal.signal(signal.SIGINT, lambda *_: sys.exit(130)); time.sleep(60)')
    safe_stop_mock = '' if safe_stop_mode == 'no_controller' else (
        f'safe_stop_ros2_control() {{ return {0 if safe_stop_mode == "accepted" else 1}; }}\n')
    script = tmp_path / 'supervisor.sh'
    script.write_text('set -euxo pipefail\n' + functions + '''
pgrep() { return 1; }
stop_matching_processes() { echo GLOBAL_SCAN_FORBIDDEN; return 99; }
''' + safe_stop_mock + '\n'.join(f'setsid {child} & {name}=$!' for name in (
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
            assert 'GLOBAL_SCAN_FORBIDDEN' not in (tmp_path / 'trace').read_text()
            survivors = []
            for pid in children:
                try:
                    os.kill(pid, 0)
                    survivors.append(pid)
                except ProcessLookupError:
                    pass
            if safe_stop_mode != 'rejected':
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


@pytest.mark.parametrize('domain,group,fault,expected,called', [
    ('229', '100', '0', 0, False),
    ('0', '999', '0', 0, False),
    ('0', '100', '0', 0, True),
    ('0', '100', '11', 1, True),
    ('unreadable', '100', '0', 1, False),
])
def test_safe_stop_ignores_foreign_managers_and_requires_fault_free_disable(
        tmp_path, domain, group, fault, expected, called):
    source = SCRIPT.read_text()
    function = 'safe_stop_ros2_control() {' + source.split('safe_stop_ros2_control() {', 1)[1].split('stop_matching_processes() {', 1)[0]
    mocks = r'''
        ROS_DOMAIN_ID=0; CONTROLLER_PID=100
        pgrep() { echo 123; }
        arm_controller_domain() { [[ "$DOMAIN" != unreadable ]] || return 1; echo "$DOMAIN"; }
        ps() { echo "$GROUP"; }
        timeout() { shift; "$@"; }
        sleep() { SECONDS=$((SECONDS+1)); }
        pass() { :; }
        ros2() {
            if [[ "$1" == service ]]; then echo service >> "$ROOT/calls"; echo 'success=True';
            else printf 'mode: 3\nfault_code: %s\n' "$FAULT"; fi
        }
    '''
    result = subprocess.run(['bash', '-c', mocks + function + '\nsafe_stop_ros2_control'],
                            env=dict(os.environ, ROOT=str(tmp_path), DOMAIN=domain, GROUP=group, FAULT=fault),
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == expected, result.stderr
    assert (tmp_path / 'calls').exists() == called


@pytest.mark.parametrize('domain,expected,pids', [
    ('229', 0, ''), ('0', 0, '123'), ('', 0, '123'),
    ('unreadable', 1, ''),
])
def test_startup_reuses_only_current_domain_manager(domain, expected, pids):
    source = SCRIPT.read_text()
    function = 'arm_domain_controllers() {' + source.split(
        'arm_domain_controllers() {', 1)[1].split('safe_stop_ros2_control() {', 1)[0]
    mocks = r'''
        ROS_DOMAIN_ID=0
        pgrep() { echo 123; }
        arm_controller_domain() { [[ "$DOMAIN" != unreadable ]] || return 1; echo "$DOMAIN"; }
    '''
    result = subprocess.run(['bash', '-c', mocks + function + '\narm_domain_controllers'],
                            env=dict(os.environ, DOMAIN=domain),
                            text=True, capture_output=True, timeout=5)
    assert result.returncode == expected, result.stderr
    assert result.stdout.strip() == pids
    startup = source.split('stage "Controller Manager and controller states"', 1)[1]
    assert 'existing_controllers=$(arm_domain_controllers) || {' in startup
    assert "if pgrep -f '/controller_manager/ros2_control_node'" not in startup
