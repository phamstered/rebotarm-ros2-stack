#!/usr/bin/env python3
"""Installed teaching stack and gripper adapter, mock MCU only, isolated DDS."""
import os
from pathlib import Path
import signal
import sys
import subprocess
import tempfile
import time
import uuid

if not 200 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 232:
    raise SystemExit('An isolated ROS_DOMAIN_ID in [200,232] is required')
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from controller_manager_msgs.srv import ListControllers, LoadController, ConfigureController, SwitchController
from rclpy.qos import qos_profile_sensor_data
from microros_stm32h723_bridge.msg import JointCommand
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState

rclpy.init()
node = rclpy.create_node('isolated_teaching_smoke')
frames = []
feedback = []
node.create_subscription(JointState, '/robot_feedback/joint_states', lambda m: feedback.append(m), qos_profile_sensor_data)
node.create_subscription(JointCommand, '/stm32/joint_commands', lambda m: frames.append(m), qos_profile_sensor_data)
owned, logs = [], []


def spin(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.02)


def request(path, typ, req):
    client = node.create_client(typ, path)
    try:
        assert client.wait_for_service(timeout_sec=15), path
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=15)
        assert future.done(), path
        return future.result()
    finally:
        node.destroy_client(client)


try:
    with tempfile.TemporaryDirectory() as tmp:
        def start(args, name):
            log = open(Path(tmp) / name, 'w+')
            logs.append(log)
            p = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            owned.append(p)
            return p, log
        def helper(*args):
            return start(['ros2', 'run', 'microros_stm32h723_bridge', 'teaching_gripper.py', *args], uuid.uuid4().hex)
        def wait(p, log, success=True):
            deadline = time.monotonic() + 35
            while p.poll() is None and time.monotonic() < deadline:
                spin(.05)
            assert p.poll() is not None, 'helper timed out'
            log.flush(); log.seek(0); output = log.read()
            assert (p.returncode == 0) == success, output
            return output
        # Simulated hand motion is confined to this already isolated DDS domain.
        mock_path = str(Path(get_package_prefix('microros_stm32h723_bridge')) /
                        'lib/microros_stm32h723_bridge/mock_stm32_node.py')
        fixture = Path(tmp) / 'manual_mock.py'
        fixture.write_text(f"""import math, runpy, time
import rclpy
Base = runpy.run_path({mock_path!r})['MockStm32Node']
class ManualMock(Base):
    def _update(self):
        q = .02 + .005 * math.sin(time.monotonic())
        self._positions[6] = (q - .004282757541) / -.008612826752
        self._target[6] = self._positions[6]
        super()._update()
rclpy.init()
node = ManualMock()
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
""")
        start([sys.executable, str(fixture)], 'mock.log')
        gains = str(Path(get_package_share_directory('microros_stm32h723_bridge')) / 'config/teaching_mit_gains.yaml')
        start(['ros2', 'launch', 'microros_stm32h723_bridge', 'stm32h723_arm.launch.py',
               'position_controller:=teaching', 'enable_gravity_controller:=true',
               'enable_gripper:=true', 'gains_file:=' + gains], 'control.log')
        deadline = time.monotonic() + 40
        while time.monotonic() < deadline:
            response = request('/controller_manager/list_controllers', ListControllers, ListControllers.Request())
            active = {c.name for c in response.controller if c.state == 'active'}
            if {'gravity_compensation_controller', 'joint_state_broadcaster'} <= active:
                break
            spin(.2)
        assert {'gravity_compensation_controller', 'joint_state_broadcaster'} <= active
        assert not active & {'arm_teleop_controller', 'arm_trajectory_controller', 'vla_position_controller', 'gripper_trajectory_controller', 'gripper_teleop_controller'}
        spin(2)
        wait(*helper('check-manual'))
        # At the mock's zero arm pose, the real Pinocchio model needs more
        # than 5 Nm on joint3. Exercise gravity -> hardware -> serialized MCU
        # command, catching accidental reuse of the position-mode 5 Nm cap.
        assert frames, 'missing MCU command frames'
        assert -10.0 < float(frames[-1].effort[2]) < -5.1, 'joint3 gravity feed-forward clipped'
        before = len(frames)
        spin(2)
        commands = frames[before:]
        assert len(commands) > 20
        assert all(all(float(k) == 0 for k in m.kp[:6]) for m in commands)
        assert all(all(float(v) == 0 for v in m.velocity[:6]) for m in commands)
        assert all(float(values[6]) == 0 for m in commands
                   for values in (m.position, m.velocity, m.kp, m.kd, m.effort))
        assert all(m.mode != 2 for m in commands), 'unexpected RETURN'
        measured = [float(m.position[m.name.index('gripper_finger_left_joint')]) for m in feedback]
        assert len(measured) > 20 and max(measured) - min(measured) > .003
        assert all(.014 <= q <= .026 for q in measured), 'manual opening lost calibration'
        wait(*helper('move', '--command-id', str(uuid.uuid4()), '--position', '0.014'), success=False)
        wait(*helper('cancel', '--command-id', str(uuid.uuid4())), success=False)
        # Even an explicit activation request cannot take over the manual gripper.
        assert request('/controller_manager/load_controller', LoadController,
                       LoadController.Request(name='gripper_trajectory_controller')).ok
        assert request('/controller_manager/configure_controller', ConfigureController,
                       ConfigureController.Request(name='gripper_trajectory_controller')).ok
        req = SwitchController.Request(activate_controllers=['gripper_trajectory_controller'], strictness=2)
        req.timeout.sec = 3
        assert not request('/controller_manager/switch_controller', SwitchController, req).ok
        # Loaded VLA cannot acquire arm position interfaces under teaching gains.
        req = SwitchController.Request(activate_controllers=['vla_position_controller'], strictness=2)
        req.timeout.sec = 3
        assert not request('/controller_manager/switch_controller', SwitchController, req).ok
        before = len(frames)
        assert request('/microros_stm32h723_hardware/safe_stop', Trigger, Trigger.Request()).success
        spin(.5)
        stopped = frames[before:]
        assert any(m.mode == 3 for m in stopped)
        assert all(m.mode != 2 for m in stopped)
        print('PASS teaching: gravity feed-forward above 5 Nm, zero arm stiffness, zero gripper output, calibrated manual feedback, legacy action rejection, controller exclusion, no RETURN')
except BaseException:
    for log in logs:
        log.flush(); log.seek(0); print(log.read())
    raise
finally:
    for p in reversed(owned):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
    for p in reversed(owned):
        try:
            p.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)
            p.wait(timeout=3)
    for log in logs:
        log.close()
    node.destroy_node()
    rclpy.shutdown()
