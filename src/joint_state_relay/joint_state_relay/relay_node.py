import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
import yaml
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory


class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')
        self.declare_parameter('config_path', '')

        config_path = self.get_parameter('config_path').value
        if not config_path:
            config_path = os.path.join(
                get_package_share_directory('joint_state_relay'),
                'config', 'relay.yaml')
        with open(config_path, 'r') as f:
            self._config = yaml.safe_load(f)

        self._mode = self._config['mode']
        self._joints_cfg = self._config['joints']
        self._mapping = self._config['joint_mapping']
        self._max_vel = self._config.get('max_velocity', 3.0)
        self._timeout = self._config.get('command_timeout', 0.1)
        self._publish_rate = self._config.get('publish_rate', 100.0)

        # Separate arm joints from gripper joints
        self._arm_joint_names = []
        self._gripper_joint_names = []
        for name in self._joints_cfg.keys():
            if name.startswith('gripper_'):
                self._gripper_joint_names.append(name)
            else:
                self._arm_joint_names.append(name)

        # In real mode, include gripper in follower_joint_names for unified publishing
        if self._mode == 'real':
            self._follower_joint_names = self._arm_joint_names + self._gripper_joint_names
        else:
            self._follower_joint_names = self._arm_joint_names

        self._leader_joint_names = [k for k, v in sorted(
            self._mapping.items(), key=lambda x: list(self._mapping.values()).index(x[1]))]

        self._last_leader_positions = None
        self._last_published_positions = None
        self._last_cmd_time = self.get_clock().now()
        self._goal_active = False
        self._gripper_goal_active = False
        self._goal_send_time = None
        self._gripper_goal_send_time = None
        self._goal_timeout = 0.1  # 100ms timeout for goal response

        # Deadband: store last commanded position per joint
        self._last_commanded_positions = {}

        self._sub = self.create_subscription(
            JointState, self._config['leader_topic'],
            self._leader_cb, 10)

        if self._mode == 'real':
            self._pub = self.create_publisher(
                JointState, self._config['follower_command_topic'], 10)
            self._timer = self.create_timer(1.0 / self._publish_rate, self._publish_real)
        else:
            # Arm controller
            action_ns = self._config.get('gazebo_controller', 'arm_controller')
            action_name = f'/{action_ns}/follow_joint_trajectory'
            self._action_client = ActionClient(
                self, FollowJointTrajectory, action_name)
            self.get_logger().info(f'Waiting for action server: {action_name}')
            self._action_client.wait_for_server()
            self.get_logger().info(f'Action server connected: {action_name}')

            # Gripper controller
            gripper_action_name = '/gripper_controller/follow_joint_trajectory'
            self._gripper_action_client = ActionClient(
                self, FollowJointTrajectory, gripper_action_name)
            self.get_logger().info(f'Waiting for gripper action server: {gripper_action_name}')
            self._gripper_action_client.wait_for_server()
            self.get_logger().info(f'Gripper action server connected: {gripper_action_name}')

            self._timer = self.create_timer(1.0 / self._publish_rate, self._publish_gazebo)

        self.get_logger().info(
            f'Relay started in {self._mode} mode, '
            f'leader={self._config["leader_topic"]}, '
            f'rate={self._publish_rate} Hz')

    def _leader_cb(self, msg: JointState):
        positions = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                positions[name] = msg.position[i]

        mapped = {}
        for leader_name, follower_name in self._mapping.items():
            if leader_name in positions:
                cfg = self._joints_cfg[follower_name]
                pos = positions[leader_name]
                new_pos = cfg['sign'] * pos + cfg['offset']

                # Apply deadband: ignore small changes
                deadband = cfg.get('deadband', 0.0)
                last_pos = self._last_commanded_positions.get(follower_name)
                if last_pos is not None and deadband > 0:
                    if abs(new_pos - last_pos) < deadband:
                        # Change is within deadband, keep previous position
                        mapped[follower_name] = last_pos
                        continue

                mapped[follower_name] = new_pos
                self._last_commanded_positions[follower_name] = new_pos

        if mapped:
            self._last_leader_positions = mapped
            self._last_cmd_time = self.get_clock().now()

            # Send gripper command separately in gazebo mode
            if self._gripper_joint_names and self._mode == 'gazebo':
                self._send_gripper_command(mapped)

    def _transform_positions(self):
        if self._last_leader_positions is None:
            return None

        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if elapsed > self._timeout:
            return None

        positions = []
        for name in self._follower_joint_names:
            pos = self._last_leader_positions.get(name, 0.0)
            cfg = self._joints_cfg[name]
            lo, hi = cfg['follower_limit']
            pos = max(lo, min(hi, pos))
            positions.append(pos)

        if self._last_published_positions is not None and self._max_vel > 0:
            dt = 1.0 / self._publish_rate
            max_delta = self._max_vel * dt
            smoothed = []
            for i, pos in enumerate(positions):
                prev = self._last_published_positions[i]
                delta = pos - prev
                if abs(delta) > max_delta:
                    pos = prev + max_delta * np.sign(delta)
                smoothed.append(pos)
            positions = smoothed

        self._last_published_positions = positions
        return positions

    def _publish_real(self):
        positions = self._transform_positions()
        if positions is None:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._follower_joint_names
        msg.position = [float(p) for p in positions]
        msg.velocity = [0.0] * len(positions)
        msg.effort = []
        self._pub.publish(msg)

    def _send_gripper_command(self, mapped_positions):
        """Send gripper position command to gripper_controller."""
        if self._gripper_goal_active:
            # Check for timeout
            if self._gripper_goal_send_time is not None:
                elapsed = (self.get_clock().now() - self._gripper_goal_send_time).nanoseconds / 1e9
                if elapsed > self._goal_timeout:
                    self.get_logger().warn('Gripper goal timeout, resetting')
                    self._gripper_goal_active = False
                    self._gripper_goal_send_time = None
                else:
                    return
            else:
                return

        gripper_positions = []
        for name in self._gripper_joint_names:
            pos = mapped_positions.get(name, 0.0)
            cfg = self._joints_cfg[name]
            lo, hi = cfg['follower_limit']
            pos = max(lo, min(hi, pos))
            gripper_positions.append(pos)

        if not gripper_positions:
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._gripper_joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in gripper_positions]
        point.velocities = [0.0] * len(gripper_positions)
        point.time_from_start = Duration(sec=0, nanosec=int(0.02 * 1e9))
        goal.trajectory.points = [point]

        self._gripper_goal_active = True
        self._gripper_goal_send_time = self.get_clock().now()
        future = self._gripper_action_client.send_goal_async(goal)
        future.add_done_callback(self._gripper_goal_response_cb)

    def _gripper_goal_response_cb(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Gripper trajectory goal rejected')
                self._gripper_goal_active = False
                self._gripper_goal_send_time = None
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._gripper_goal_result_cb)
        except Exception as e:
            self.get_logger().warn(f'Gripper goal response error: {e}')
            self._gripper_goal_active = False
            self._gripper_goal_send_time = None

    def _gripper_goal_result_cb(self, future):
        self._gripper_goal_active = False
        self._gripper_goal_send_time = None

    def _publish_gazebo(self):
        if self._goal_active:
            # Check for timeout
            if self._goal_send_time is not None:
                elapsed = (self.get_clock().now() - self._goal_send_time).nanoseconds / 1e9
                if elapsed > self._goal_timeout:
                    self.get_logger().warn('Goal timeout, resetting')
                    self._goal_active = False
                    self._goal_send_time = None
                else:
                    return
            else:
                return

        positions = self._transform_positions()
        if positions is None:
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._follower_joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(sec=0, nanosec=int(0.02 * 1e9))
        goal.trajectory.points = [point]

        self._goal_active = True
        self._goal_send_time = self.get_clock().now()
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn('Trajectory goal rejected')
                self._goal_active = False
                self._goal_send_time = None
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._goal_result_cb)
        except Exception as e:
            self.get_logger().warn(f'Goal response error: {e}')
            self._goal_active = False
            self._goal_send_time = None

    def _goal_result_cb(self, future):
        self._goal_active = False
        self._goal_send_time = None


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
