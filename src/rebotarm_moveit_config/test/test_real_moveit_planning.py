#!/usr/bin/env python3
"""Real OMPL/Planning Scene smoke test, exclusively in an isolated ROS domain.

Run after installing this package with ROS_DOMAIN_ID=226. Calls planning
services only, and starts no controller or hardware process.
"""
import importlib.util
import math
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
from moveit_msgs.msg import JointConstraint, Constraints, CollisionObject, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetStateValidity, GetMotionPlan, ApplyPlanningScene, GetPositionFK
from shape_msgs.msg import SolidPrimitive


def main():
    assert 200 <= int(os.environ.get("ROS_DOMAIN_ID", "0")) <= 232, "isolated ROS_DOMAIN_ID required"
    path = Path(get_package_share_directory("rebotarm_moveit_config")) / "launch/real_move_group.launch.py"
    spec = importlib.util.spec_from_file_location("real_launch", path); module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    params = []
    original_node = module.Node
    from launch.actions import OpaqueFunction
    module.Node = lambda **kwargs: (params.append(kwargs["parameters"][0]) or OpaqueFunction(function=lambda _: []))
    module.move_group_node(True); module.Node = original_node
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
        yaml.safe_dump({"move_group": {"ros__parameters": params[0]}}, f); parameter_path = f.name
    probe = os.environ.get("ROBOT_MODEL_LIMITS_PROBE")
    if probe:
        subprocess.run([probe, "--ros-args", "--params-file", parameter_path], check=True)
    log = open("/tmp/codex-real-moveit-planning.log", "w")
    process = subprocess.Popen(["/opt/ros/humble/lib/moveit_ros_move_group/move_group", "--ros-args", "--params-file", parameter_path, "--log-level", "moveit_constraint_samplers.constraint_sampler_manager:=debug"], stdout=log, stderr=log)
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
        fk = node.create_client(GetPositionFK, "/compute_fk"); wait(fk)
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
        request.motion_plan_request.max_velocity_scaling_factor=1.;request.motion_plan_request.max_acceleration_scaling_factor=1.;request.motion_plan_request.start_state=state(valid)
        constraints=Constraints()
        for name,value in zip(names,goal):
            c=JointConstraint();c.joint_name=name;c.position=value;c.tolerance_above=.005;c.tolerance_below=.005;c.weight=1.;constraints.joint_constraints.append(c)
        request.motion_plan_request.goal_constraints=[constraints]
        result=call(planner,request).motion_plan_response
        assert result.error_code.val==1,result.error_code.val
        points=result.trajectory.joint_trajectory.points
        assert len(points)>1
        previous=-1
        for point in points:
            stamp=point.time_from_start.sec*1000000000+point.time_from_start.nanosec
            assert stamp>previous;previous=stamp
            assert len(point.velocities)==6 and all(abs(v)<=2.+1e-6 for v in point.velocities)
            assert len(point.accelerations)==6 and all(abs(a)<=2.+1e-6 for a in point.accelerations)
        # Target a known reachable full pose using MoveIt's own FK. The target
        # differs from start, so a zero-motion shortcut cannot satisfy the test.
        def tip_pose(values):
            query = GetPositionFK.Request()
            query.header.frame_id = "base_link"
            query.fk_link_names = ["link6"]
            query.robot_state = values
            response = call(fk, query)
            assert response.error_code.val == 1
            return response.pose_stamped[0].pose

        desired = tip_pose(state(goal))
        for group in ("arm", "arm_with_gripper"):
            pose_request = GetMotionPlan.Request()
            motion = pose_request.motion_plan_request
            motion.group_name = group; motion.pipeline_id = "ompl"
            motion.allowed_planning_time = 10.; motion.num_planning_attempts = 1
            motion.max_velocity_scaling_factor = 1.; motion.max_acceleration_scaling_factor = 1.
            motion.start_state = state(valid)
            pc = PositionConstraint(); pc.header.frame_id = "base_link"; pc.link_name = "link6"; pc.weight = 1.
            region = SolidPrimitive(); region.type = SolidPrimitive.SPHERE; region.dimensions = [.001]
            centre = Pose(); centre.position = desired.position; centre.orientation.w = 1.
            pc.constraint_region.primitives = [region]; pc.constraint_region.primitive_poses = [centre]
            oc = OrientationConstraint(); oc.header.frame_id = "base_link"; oc.link_name = "link6"; oc.weight = 1.
            oc.orientation = desired.orientation
            oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = oc.absolute_z_axis_tolerance = .005
            target = Constraints(); target.position_constraints = [pc]; target.orientation_constraints = [oc]
            if group == "arm_with_gripper":
                jc = JointConstraint(); jc.joint_name = "gripper_finger_left_joint"
                jc.position = .03; jc.tolerance_above = jc.tolerance_below = .0005; jc.weight = 1.
                target.joint_constraints = [jc]
            motion.goal_constraints = [target]
            planned = call(planner, pose_request).motion_plan_response
            assert planned.error_code.val == 1, (group, planned.error_code.val)
            trajectory = planned.trajectory.joint_trajectory
            expected = names + (["gripper_finger_left_joint"] if group == "arm_with_gripper" else [])
            assert set(trajectory.joint_names) == set(expected)
            assert len(trajectory.points) > 1
            previous_stamp = -1
            for point in trajectory.points:
                stamp = point.time_from_start.sec*1000000000 + point.time_from_start.nanosec
                assert stamp > previous_stamp; previous_stamp = stamp
                assert len(point.positions) == len(point.velocities) == len(point.accelerations) == len(expected)
                for index, name in enumerate(trajectory.joint_names):
                    assert all(math.isfinite(values[index]) for values in (point.positions, point.velocities, point.accelerations))
                    velocity, acceleration = (.02, .04) if name == "gripper_finger_left_joint" else (2., 2.)
                    assert abs(point.velocities[index]) <= velocity + 1e-6
                    assert abs(point.accelerations[index]) <= acceleration + 1e-6
                    if name == "gripper_finger_left_joint":
                        assert 0. <= point.positions[index] <= .05
            endpoint = dict(zip(trajectory.joint_names, trajectory.points[-1].positions))
            end_state = state([endpoint[name] for name in names])
            end_state.joint_state.position[-1] = endpoint.get("gripper_finger_left_joint", .02)
            actual_pose = tip_pose(end_state)
            distance = math.sqrt(sum((getattr(actual_pose.position, key)-getattr(desired.position, key))**2 for key in ("x", "y", "z")))
            dot = abs(sum(getattr(actual_pose.orientation, key)*getattr(desired.orientation, key) for key in ("x", "y", "z", "w")))
            angle = 2*math.acos(min(1., max(-1., dot)))
            assert distance <= .0011 and angle <= .01, (group, distance, angle)
            if group == "arm_with_gripper":
                assert abs(endpoint["gripper_finger_left_joint"]-.03) <= .0005
            check = GetStateValidity.Request(); check.robot_state = end_state; check.group_name = group
            assert call(validity, check).valid
            print("PASS actual OMPL full-pose goal", group, len(trajectory.points), "points", distance, "m", angle, "rad", flush=True)
        # Prove the runtime used the arm IK sampler plus the gripper joint sampler.
        log.flush()
        assert "Constructing sampler for group 'arm_with_gripper' as a union" in Path(log.name).read_text()
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
