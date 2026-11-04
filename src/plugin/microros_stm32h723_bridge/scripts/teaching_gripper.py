#!/usr/bin/env python3
"""Bounded, UUID-scoped gripper action for hand-guiding; never commands arm joints.

The caller holds no robot command stream. The existing ros2_control hardware
continues gravity compensation. Cancellation never cancels another goal.
"""
import argparse
import fcntl
import json
import math
from pathlib import Path
import signal
import time
import uuid

from teaching_core import DURATION, JOINT, controller_error, fresh, target_position


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('operation', choices=['move', 'cancel', 'check'])
    parser.add_argument('--command-id', type=uuid.UUID)
    parser.add_argument('--position', type=target_position)
    args = parser.parse_args()
    if args.operation != 'check' and args.command_id is None:
        parser.error('--command-id is required')
    if args.operation == 'move' and args.position is None:
        parser.error('--position is required')
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from action_msgs.srv import CancelGoal
    from action_msgs.msg import GoalStatus
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListControllers
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import JointState
    from microros_stm32h723_bridge.msg import HardwareStatus
    from trajectory_msgs.msg import JointTrajectoryPoint
    from unique_identifier_msgs.msg import UUID

    rclpy.init()
    node = rclpy.create_node('teaching_gripper_' + uuid.uuid4().hex[:12])
    latest = {}
    canceled = False
    goal_handle = None
    goal_submitted = False
    result_future = None
    lock = None
    action_path = '/gripper_trajectory_controller/follow_joint_trajectory'
    def interrupt(*_):
        nonlocal canceled
        canceled = True
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    def store(key, msg):
        latest[key] = (msg, time.monotonic())
    for name, topic, typ in [('state', '/robot_feedback/joint_states', JointState),
                             ('status', '/stm32/status', HardwareStatus)]:
        node.create_subscription(typ, topic, lambda msg, key=name: store(key, msg), qos_profile_sensor_data)
    def await_future(future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        if not future.done():
            raise RuntimeError('ROS 请求超时')
        return future.result()
    def call(path, typ, request):
        client = node.create_client(typ, path)
        try:
            if not client.wait_for_service(timeout_sec=2):
                raise RuntimeError(f'服务未就绪：{path}')
            return await_future(client.call_async(request), 2)
        finally:
            node.destroy_client(client)
    def feedback():
        now = time.monotonic()
        ros_now = node.get_clock().now().nanoseconds
        for key in ('state', 'status'):
            if key not in latest:
                raise RuntimeError(f'等待新鲜反馈：{key}')
            msg, received = latest[key]
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            if not fresh(received, stamp, now, ros_now):
                raise RuntimeError(f'反馈过期或时钟无效：{key}')
        status = latest['status'][0]
        if status.fault_code != 0 or status.mode != 1 or status.active_joint_mask != 127:
            raise RuntimeError('夹爪操作需要七轴使能、无故障 TRACK')
        state = latest['state'][0]
        index = state.name.index(JOINT)
        q, v = state.position[index], state.velocity[index]
        if not math.isfinite(q) or not math.isfinite(v) or not -0.001 <= q <= 0.051:
            raise RuntimeError('夹爪标定反馈无效')
        return q, v
    def controllers():
        response = call('/controller_manager/list_controllers', ListControllers, ListControllers.Request())
        error = controller_error(response.controller)
        if error:
            raise RuntimeError(error)
    def cancel_goal():
        request = CancelGoal.Request()
        request.goal_info.goal_id = UUID(uuid=list(args.command_id.bytes))
        return call(action_path + '/_action/cancel_goal', CancelGoal, request)
    def wait_stopped(seconds=3):
        deadline, since = time.monotonic() + seconds, None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            _, velocity = feedback()
            if abs(velocity) <= 0.001:
                since = since or time.monotonic()
                if time.monotonic() - since >= 0.2:
                    return
            else:
                since = None
        raise RuntimeError('夹爪停止未确认')
    try:
        if args.operation == 'cancel':
            response = cancel_goal()
            if not response.goals_canceling:
                raise RuntimeError('本次夹爪目标尚未受理或已结束；请等待执行结果')
            # ACK is only a cancellation request. The move process verifies stop.
            print(json.dumps({'ok': True, 'message': '取消已请求，等待原操作确认停稳'}, ensure_ascii=False))
            return
        if args.operation == 'move':
            directory = Path.home() / '.cache/robot-console'
            directory.mkdir(parents=True, exist_ok=True)
            lock = (directory / 'arm-mode.lock').open('a')
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        request = GetParameters.Request(names=['drag_teaching'])
        values = call('/microros_stm32h723_hardware/get_parameters', GetParameters, request).values
        if len(values) != 1 or values[0].type != 1 or not values[0].bool_value:
            raise RuntimeError('硬件未启用经过增益校验的拖动示教模式，请升级并重新启动控制栈')
        controllers()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and len(latest) < 2:
            rclpy.spin_once(node, timeout_sec=0.02)
        q, velocity = feedback()
        if args.operation == 'check':
            print(json.dumps({'ok': True, 'message': '拖动示教与夹爪已就绪'}, ensure_ascii=False))
            return
        if abs(velocity) > 0.001:
            raise RuntimeError('夹爪尚未停稳')
        action = ActionClient(node, FollowJointTrajectory, action_path)
        if not action.wait_for_server(timeout_sec=2):
            raise RuntimeError('夹爪 Action 未就绪')
        if canceled:
            raise RuntimeError('操作已取消')
        q, velocity = feedback()
        if abs(velocity) > 0.001:
            raise RuntimeError('夹爪尚未停稳')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [JOINT]
        # Cubic interpolation, rest-to-rest, 5 s over <= 50 mm:
        # peak speed <=15 mm/s and acceleration <=12 mm/s².
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[q], velocities=[0.0]),
            JointTrajectoryPoint(positions=[args.position], velocities=[0.0])]
        goal.trajectory.points[1].time_from_start.sec = int(DURATION)
        goal_submitted = True
        goal_handle = await_future(action.send_goal_async(goal, goal_uuid=UUID(uuid=list(args.command_id.bytes))), 3)
        if not goal_handle.accepted:
            goal_handle = None
            raise RuntimeError('夹爪目标被拒绝')
        result_future = goal_handle.get_result_async()
        deadline, next_check = time.monotonic() + DURATION + 5, 0
        while not result_future.done():
            rclpy.spin_once(node, timeout_sec=0.02)
            feedback()
            if canceled or time.monotonic() > deadline:
                raise RuntimeError('操作取消或执行超时')
            if time.monotonic() >= next_check:
                controllers()
                next_check = time.monotonic() + 0.5
        result = result_future.result()
        wait_stopped()
        if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
            raise RuntimeError('夹爪操作已取消或失败：' + result.result.error_string)
        print(json.dumps({'ok': True, 'message': '夹爪动作完成并停稳；不代表夹持成功',
                          'actual_position': feedback()[0]}, ensure_ascii=False))
    except Exception as error:
        reason = str(error)
        # UUID also covers a timed-out goal acceptance response: never cancel all.
        if goal_submitted:
            try:
                cancel_goal()
                wait_stopped()
                reason += '；夹爪已停稳'
            except Exception as stop_error:
                reason += f'；停止未确认：{stop_error}'
        print(json.dumps({'ok': False, 'message': reason}, ensure_ascii=False))
        raise SystemExit(1)
    finally:
        if lock is not None:
            lock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
