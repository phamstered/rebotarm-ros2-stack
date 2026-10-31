#!/usr/bin/env python3
"""Exercise installed JTC -> MicroRosSystem -> simulated MCU in an isolated domain.

Requires ROS_DOMAIN_ID in [200,232]. Never starts a serial/physical driver.
"""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

if not 200 <= int(os.environ.get('ROS_DOMAIN_ID', '0')) <= 232:
    raise SystemExit('Set an isolated ROS_DOMAIN_ID in [200,232]; never run in production domain')

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers, SwitchController
from microros_stm32h723_bridge.msg import JointCommand
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

rclpy.init()
node = rclpy.create_node('isolated_velocity_smoke')
messages = []
subscription = node.create_subscription(JointCommand, '/stm32/joint_commands',
    lambda message: messages.append((time.monotonic(), message)), qos_profile_sensor_data)
owned = []
logs = []


def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)


def request(service, typ, data, timeout=15):
    client = node.create_client(typ, service)
    try:
        assert client.wait_for_service(timeout_sec=timeout), service
        future = client.call_async(data)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        assert future.done(), service
        return future.result()
    finally:
        node.destroy_client(client)


def switch(start, stop):
    req = SwitchController.Request(); req.activate_controllers = start; req.deactivate_controllers = stop
    req.strictness = SwitchController.Request.STRICT
    req.timeout.sec = 5
    assert request('/controller_manager/switch_controller', SwitchController, req).ok


try:
    with tempfile.TemporaryDirectory(prefix='trajectory-velocity-smoke-') as tmp:
        def launch(arguments, name):
            log = open(Path(tmp)/name, 'w+'); logs.append(log)
            process = subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            owned.append(process)
        launch(['ros2', 'run', 'microros_stm32h723_bridge', 'mock_stm32_node.py', '--ros-args', '-p', 'active_joint_mask:=63'], 'mock.log')
        control_args = ['ros2', 'launch', 'microros_stm32h723_bridge', 'stm32h723_arm.launch.py',
                'position_controller:=trajectory', 'enable_gravity_controller:=false', 'enable_gripper:=false']
        launch(control_args, 'control.log')
        end = time.monotonic()+35
        while time.monotonic() < end:
            response = request('/controller_manager/list_controllers', ListControllers, ListControllers.Request())
            states = {c.name: c for c in response.controller}
            if states.get('arm_trajectory_controller') and states['arm_trajectory_controller'].state == 'active' and states.get('arm_teleop_controller'):
                break
            spin(.2)
        assert states['arm_trajectory_controller'].state == 'active'
        expected = {f'joint{i}/{interface}' for i in range(1,7) for interface in ('position','velocity')}
        assert expected.issubset(set(states['arm_trajectory_controller'].claimed_interfaces))
        action = ActionClient(node, FollowJointTrajectory, '/arm_trajectory_controller/follow_joint_trajectory')
        assert action.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = [f'joint{i}' for i in range(1,7)]
        for t, q in [(.1, [0.]*6), (2.1, [.2,-.1,-.1,0.,0.,0.])]:
            point = JointTrajectoryPoint(); point.positions=q; point.velocities=[0.]*6
            point.time_from_start.sec=int(t); point.time_from_start.nanosec=round((t-int(t))*1e9)
            goal.trajectory.points.append(point)
        begin=time.monotonic()
        accepted=action.send_goal_async(goal); rclpy.spin_until_future_complete(node,accepted,timeout_sec=5)
        assert accepted.done() and accepted.result().accepted
        completed=accepted.result().get_result_async();rclpy.spin_until_future_complete(node,completed,timeout_sec=10)
        assert completed.done() and completed.result().result.error_code == 0
        during=[m for t,m in messages if t>=begin]
        assert len(during)>30
        assert max(abs(float(m.velocity[0])) for m in during)>.05, 'JTC velocity did not reach MCU command'
        assert max(abs(float(m.velocity[1])) for m in during)>.02
        assert all(float(m.velocity[6])==0. for m in during)
        assert all(all(float(e)==0. for e in m.effort) for m in during), 'unexpected effort with gravity disabled'
        switch(['arm_teleop_controller'], ['arm_trajectory_controller'])
        begin=time.monotonic();spin(.6)
        after=[m for t,m in messages if t>begin+.1]
        assert len(after)>10 and all(all(float(v)==0. for v in m.velocity) for m in after), 'velocity leaked into Teleop'
        assert all(all(float(e)==0. for e in m.effort) for m in after), 'unexpected effort after switching to Teleop'
        switch(['arm_trajectory_controller'], ['arm_teleop_controller'])
        begin=time.monotonic();spin(.5)
        after=[m for t,m in messages if t>begin+.1]
        assert len(after)>10 and all(all(float(v)==0. for v in m.velocity) for m in after), 'stale velocity after JTC reactivation'
        # Legacy Robot View drag sends positions only: JTC still derives velocity.
        legacy = FollowJointTrajectory.Goal(); legacy.trajectory.joint_names = goal.trajectory.joint_names
        for t, q in [(.1, [.2,-.1,-.1,0.,0.,0.]), (1.6, [.23,-.1,-.1,0.,0.,0.])]:
            point = JointTrajectoryPoint(); point.positions = q
            point.time_from_start.sec = int(t); point.time_from_start.nanosec = round((t-int(t))*1e9)
            legacy.trajectory.points.append(point)
        begin = time.monotonic()
        accepted = action.send_goal_async(legacy); rclpy.spin_until_future_complete(node,accepted,timeout_sec=5)
        assert accepted.done() and accepted.result().accepted
        completed = accepted.result().get_result_async(); rclpy.spin_until_future_complete(node,completed,timeout_sec=10)
        assert completed.done() and completed.result().result.error_code == 0
        assert any(abs(float(m.velocity[0])) > .01 for t,m in messages if t>=begin), 'positions-only compatibility failed'
        response=request('/microros_stm32h723_hardware/disable', Trigger, Trigger.Request())
        assert response.success;begin=time.monotonic();spin(.3)
        assert any(m.mode==3 and all(float(v)==0. for values in
                   (m.position,m.velocity,m.kp,m.kd,m.effort) for v in values)
                   for t,m in messages if t>=begin)
        print('PASS actual JTC position+velocity -> MicroRosSystem -> mock MCU, Teleop switch, reactivation, DISABLE; domain',os.environ['ROS_DOMAIN_ID'])
except BaseException:
    for log in logs:
        log.flush();log.seek(0);print(log.read())
    raise
finally:
    for process in reversed(owned):
        if process.poll() is None:
            os.killpg(process.pid,signal.SIGINT)
    for process in reversed(owned):
        try:process.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=3)
    for log in logs:log.close()
    node.destroy_node();rclpy.shutdown()
