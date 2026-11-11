"""Isolated real RSP test: bad source time cannot freeze corrected TF."""
import os
import signal
from pathlib import Path
import subprocess
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from rclpy.qos import qos_profile_sensor_data

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from stm32_state_republisher import Stm32StateRepublisher


def test_real_rsp_recovers_large_source_rollback_and_tf_rate_is_bounded():
    assert os.environ.get('ROS_DOMAIN_ID') == '218'
    rclpy.init(args=['--ros-args', '-p', 'publish_tf_feedback:=true'])
    relay = Stm32StateRepublisher()
    node = rclpy.create_node('clock_feedback_test')
    pub = node.create_publisher(JointState, '/stm32/joint_states', qos_profile_sensor_data)
    bypass = node.create_publisher(JointState, '/robot_feedback/tf_joint_states', qos_profile_sensor_data)
    received = []
    normalized = []
    node.create_subscription(TFMessage, '/tf', lambda m: received.extend(m.transforms), qos_profile_sensor_data)
    node.create_subscription(JointState, '/joint_states', normalized.append, qos_profile_sensor_data)
    urdf = '<robot name="test"><link name="base_link"/>'
    for i in range(1, 7):
        urdf += ('<link name="link%d"/><joint name="joint%d" type="continuous">'
                 '<parent link="%s"/><child link="link%d"/><axis xyz="0 0 1"/></joint>') % (i,i,'base_link' if i == 1 else 'link%d'%(i-1),i)
    urdf += '</robot>'
    process = subprocess.Popen(['ros2','run','robot_state_publisher','robot_state_publisher',
        '--ros-args','-p','robot_description:='+urdf,'-p','ignore_timestamp:=true',
        '-r','joint_states:=/robot_feedback/tf_joint_states'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    def pump(seconds, future=0, direct=False):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            msg = JointState()
            ns = node.get_clock().now().nanoseconds + int(future*1e9)
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns,1000000000)
            msg.name = ['joint%d'%i for i in range(1,7)]
            msg.position = [.1]*6
            (bypass if direct else pub).publish(msg)
            rclpy.spin_once(relay, timeout_sec=.001)
            rclpy.spin_once(node, timeout_sec=.001)
            time.sleep(.002)
    try:
        until = time.monotonic()+10
        while bypass.get_subscription_count() == 0 and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        assert bypass.get_subscription_count() > 0, "RSP subscription discovery timed out"
        pump(1., future=15)
        assert not normalized and not received
        # Deliberately seed RSP's prior-epoch throttle as would happen before upgrade.
        pump(.3, future=15, direct=True)
        assert received
        # Clearing the Python list cannot drain TF still queued by DDS/RSP.
        # Require corrected TF within the original one-second recovery budget;
        # then measure a full clean window without accepting any future samples.
        recovery_start = node.get_clock().now().nanoseconds
        deadline = time.monotonic() + 1.
        recovered = False
        while time.monotonic() < deadline and not recovered:
            pump(.05)
            now = node.get_clock().now().nanoseconds
            recovered = any(
                t.child_frame_id == 'link1' and
                recovery_start <= t.header.stamp.sec*1000000000+t.header.stamp.nanosec <= now
                for t in received)
        assert recovered, "RSP did not recover corrected TF within one second"
        received.clear()
        normalized.clear()
        start = node.get_clock().now().nanoseconds
        pump(1.)
        stamps = [t.header.stamp.sec*1000000000+t.header.stamp.nanosec for t in received if t.child_frame_id=='link1']
        assert stamps and min(stamps) >= start
        assert max(stamps) <= node.get_clock().now().nanoseconds
        assert 8 <= len(stamps) <= 22, len(stamps)
        assert len(normalized) > len(stamps)*2
        assert normalized[-1].header.stamp.sec != 0
    finally:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait()
        relay.destroy_node();node.destroy_node();rclpy.shutdown()
