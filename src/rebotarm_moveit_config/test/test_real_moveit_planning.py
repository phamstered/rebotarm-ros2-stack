#!/usr/bin/env python3
"""Real OMPL/Planning Scene smoke test, exclusively in an isolated ROS domain.

Run after installing this package with ROS_DOMAIN_ID=226. Calls planning
services only, and starts no controller or hardware process.
"""
import importlib.util
import os
from pathlib import Path
import random
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Pose
from sensor_msgs.msg import JointState
from tf2_ros import StaticTransformBroadcaster
from moveit_msgs.msg import JointConstraint, Constraints, CollisionObject
from moveit_msgs.srv import GetStateValidity, GetMotionPlan, ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def main():
    assert 200 <= int(os.environ.get("ROS_DOMAIN_ID", "0")) <= 232, "isolated ROS_DOMAIN_ID required"
    path = Path(get_package_share_directory("rebotarm_moveit_config")) / "launch/real_move_group.launch.py"
    spec = importlib.util.spec_from_file_location("real_launch", path); module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    params = []
    original_node = module.Node
    from launch.actions import OpaqueFunction
    module.Node = lambda **kwargs: (params.append(kwargs["parameters"][0]) or OpaqueFunction(function=lambda _: []))
    module.generate_launch_description(); module.Node = original_node
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        yaml.safe_dump({"move_group": {"ros__parameters": params[0]}}, f); parameter_path = f.name
    log = open("/tmp/codex-real-moveit-planning.log", "w")
    process = subprocess.Popen(["/opt/ros/humble/lib/moveit_ros_move_group/move_group", "--ros-args", "--params-file", parameter_path], stdout=log, stderr=log)
    rclpy.init(); node = Node("isolated_moveit_planning_probe")
    names = [f"joint{i}" for i in range(1,7)]; q = [0., -.2, -.3, 0.,0.,0.]
    pub = node.create_publisher(JointState, "/joint_states", 10)
    def publish():
        j = JointState(); j.header.stamp = node.get_clock().now().to_msg(); j.name = names+["gripper_finger_left_joint"]; j.position = q+[.02]; j.velocity=[0.]*7; pub.publish(j)
    node.create_timer(.03, publish)
    broadcaster = StaticTransformBroadcaster(node); transform = TransformStamped(); transform.header.frame_id="world";transform.child_frame_id="base_link";transform.transform.rotation.w=1.;broadcaster.sendTransform(transform)
    validity=node.create_client(GetStateValidity,"/check_state_validity"); planner=node.create_client(GetMotionPlan,"/plan_kinematic_path"); scene=node.create_client(ApplyPlanningScene,"/apply_planning_scene")
    def wait(client):
        end=time.monotonic()+60
        while time.monotonic()<end and not client.service_is_ready():
            if process.poll() is not None:raise RuntimeError("move_group failed: "+Path(log.name).read_text()[-4000:])
            rclpy.spin_once(node,timeout_sec=.1)
        assert client.service_is_ready(),"MoveIt startup timeout"
    def call(client, request):
        future=client.call_async(request);rclpy.spin_until_future_complete(node,future,timeout_sec=15)
        assert future.done(),"service timeout";return future.result()
    def state(values):
        from moveit_msgs.msg import RobotState
        s=RobotState();s.joint_state.name=names+["gripper_finger_left_joint"];s.joint_state.position=values+[.02];return s
    try:
        wait(validity);wait(planner);wait(scene)
        urdf=ET.fromstring(params[0]["robot_description"])
        limits=[]
        for name in names:
            limit=urdf.find(f"joint[@name='{name}']/limit");limits.append((float(limit.get("lower")),float(limit.get("upper"))))
        rng=random.Random(130);valid=None
        for index in range(80):
            candidate=q if index==0 else [rng.uniform(lo+.01,hi-.01) for lo,hi in limits]
            request=GetStateValidity.Request();request.robot_state=state(candidate);request.group_name="arm"
            if call(validity,request).valid:valid=list(candidate);break
        assert valid is not None,"No valid arm pose found; collision model must be reviewed, never weaken collision checking"
        q[:]=valid
        goal=list(valid);goal[0]=max(limits[0][0]+.01,min(limits[0][1]-.01,goal[0]+.02))
        request=GetMotionPlan.Request();request.motion_plan_request.group_name="arm";request.motion_plan_request.pipeline_id="ompl";request.motion_plan_request.allowed_planning_time=5.;request.motion_plan_request.num_planning_attempts=1
        request.motion_plan_request.max_velocity_scaling_factor=.2;request.motion_plan_request.max_acceleration_scaling_factor=.2;request.motion_plan_request.start_state=state(valid)
        constraints=Constraints()
        for name,value in zip(names,goal):
            c=JointConstraint();c.joint_name=name;c.position=value;c.tolerance_above=.005;c.tolerance_below=.005;c.weight=1.;constraints.joint_constraints.append(c)
        request.motion_plan_request.goal_constraints=[constraints]
        result=call(planner,request).motion_plan_response
        assert result.error_code.val==1,result.error_code.val
        assert len(result.trajectory.joint_trajectory.points)>1
        obstacle=CollisionObject();obstacle.header.frame_id="base_link";obstacle.id="isolated_blocking_box";obstacle.operation=CollisionObject.ADD
        box=SolidPrimitive();box.type=SolidPrimitive.BOX;box.dimensions=[2.,2.,2.];pose=Pose();pose.orientation.w=1.;obstacle.primitives=[box];obstacle.primitive_poses=[pose]
        update=ApplyPlanningScene.Request();update.scene.is_diff=True;update.scene.robot_state.is_diff=True;update.scene.world.collision_objects=[obstacle]
        assert call(scene,update).success
        invalid=GetStateValidity.Request();invalid.robot_state=state(valid);invalid.group_name="arm"
        check=call(validity,invalid);assert not check.valid
        rejected=call(planner,request).motion_plan_response
        assert rejected.error_code.val!=1
        print("PASS actual MoveIt OMPL planning",len(result.trajectory.joint_trajectory.points),"timed points; blocking Planning Scene obstacle rejected, error",rejected.error_code.val,"contacts",len(check.contacts))
    finally:
        node.destroy_node();rclpy.try_shutdown();process.send_signal(signal.SIGINT)
        try:process.wait(timeout=12)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();os.unlink(parameter_path)

if __name__=="__main__":main()
