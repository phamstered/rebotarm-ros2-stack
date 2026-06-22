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

        self._follower_joint_names = list(self._joints_cfg.keys())
        self._leader_joint_names = [k for k, v in sorted(
            self._mapping.items(), key=lambda x: list(self._mapping.values()).index(x[1]))]

        self._last_leader_positions = None
        self._last_published_positions = None
        self._last_cmd_time = self.get_clock().now()
        self._goal_active = False

        self._sub = self.create_subscription(
            JointState, self._config['leader_topic'],
            self._leader_cb, 10)

        if self._mode == 'real':
            self._pub = self.create_publisher(
                JointState, self._config['follower_command_topic'], 10)
            self._timer = self.create_timer(1.0 / self._publish_rate, self._publish_real)
        else:
            action_ns = self._config.get('gazebo_controller', 'arm_controller')
            action_name = f'/{action_ns}/follow_joint_trajectory'
            self._action_client = ActionClient(
                self, FollowJointTrajectory, action_name)
            self.get_logger().info(f'Waiting for action server: {action_name}')
            self._action_client.wait_for_server()
            self.get_logger().info(f'Action server connected: {action_name}')
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
                mapped[follower_name] = cfg['sign'] * pos + cfg['offset']

        if mapped:
            self._last_leader_positions = mapped
            self._last_cmd_time = self.get_clock().now()

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
        msg.position = positions
        msg.velocity = []
        msg.effort = []
        self._pub.publish(msg)

    def _publish_gazebo(self):
        if self._goal_active:
            return

        positions = self._transform_positions()
        if positions is None:
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._follower_joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(sec=0, nanosec=int(0.05 * 1e9))
        goal.trajectory.points = [point]

        self._goal_active = True
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Trajectory goal rejected')
            self._goal_active = False
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        self._goal_active = False


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
