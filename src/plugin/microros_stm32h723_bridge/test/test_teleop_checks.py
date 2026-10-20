"""Run the startup script's command-subscriber gate with delayed ROS discovery."""
from pathlib import Path
import os
import subprocess


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
