#!/usr/bin/env python3
"""Real ROS signal tests with a synthetic vendor driver, never serial hardware."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory


def main():
    assert int(os.environ.get('ROS_DOMAIN_ID', '0')) != 0, 'isolated ROS domain required'
    prefix = Path(get_package_prefix('joint_state_relay'))
    share = Path(get_package_share_directory('joint_state_relay'))
    profile = share / 'config/fastdds_udp.xml'
    assert profile.is_file(), 'installed Fast DDS profile is missing'
    with tempfile.TemporaryDirectory(prefix='relay-lifecycle-') as directory:
        root = Path(directory)
        vendor = root / 'robo_driver'
        vendor.mkdir()
        (vendor / '__init__.py').write_text('')
        (vendor / 'robo_driver.py').write_text('''
import os
from rclpy.node import Node
class Arm_contorl(Node):
    def __init__(self):
        super().__init__('robo_driver_node')
        if os.environ.get('ROBOT_CONSOLE_TEST_FAIL'):
            raise RuntimeError('fixture initialization failure')
        print('NODE_READY ' + self.get_fully_qualified_name(), flush=True)
    def node_close(self):
        print('VENDOR_CLEANUP', flush=True)
''')
        environment = dict(os.environ, PYTHONPATH=str(root) + os.pathsep + os.environ.get('PYTHONPATH', ''), FASTRTPS_DEFAULT_PROFILES_FILE=str(profile))
        cases = 0
        for executable, ready, arguments in (
            ('leader_driver', 'NODE_READY /leader/robo_driver_node', ['--ros-args', '-r', '__ns:=/leader', '-p', 'lock:=disable']),
            ('relay_node', 'Relay started', ['--ros-args', '-p', 'config_path:=' + str(share / 'config/relay_microros_arm.yaml')]),
        ):
            for stop_signal in (signal.SIGINT, signal.SIGTERM):
                for cycle in range(2):
                    logfile = root / f'{executable}-{stop_signal}-{cycle}.log'
                    with logfile.open('w') as log:
                        process = subprocess.Popen([str(prefix / 'lib/joint_state_relay' / executable), *arguments], env=environment, stdout=log, stderr=subprocess.STDOUT)
                        try:
                            deadline = time.monotonic() + 10
                            while time.monotonic() < deadline and process.poll() is None:
                                if ready in logfile.read_text():
                                    break
                                time.sleep(.05)
                            assert ready in logfile.read_text(), logfile.read_text()
                            process.send_signal(stop_signal)
                            assert process.wait(timeout=8) == 0, logfile.read_text()
                            output = logfile.read_text()
                            for forbidden in ('Traceback', 'ExternalShutdownException', 'context is invalid', '[ERROR]'):
                                assert forbidden not in output, output
                            if executable == 'leader_driver':
                                assert 'VENDOR_CLEANUP' in output
                            cases += 1
                        finally:
                            if process.poll() is None:
                                process.kill()
                                process.wait()
        failure = subprocess.run([str(prefix / 'lib/joint_state_relay/leader_driver')], env={**environment, 'ROBOT_CONSOLE_TEST_FAIL': '1'}, text=True, capture_output=True, timeout=10)
        assert failure.returncode != 0 and 'fixture initialization failure' in failure.stderr
        print(f'PASS {cases} isolated SIGINT/SIGTERM relay/leader cycles, vendor cleanup, namespace/arguments, and genuine initialization failure propagation')


if __name__ == '__main__':
    main()
