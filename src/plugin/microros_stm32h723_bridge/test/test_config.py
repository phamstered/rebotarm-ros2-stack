import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).parents[1]


def test_all_gain_groups_have_seven_finite_nonnegative_values():
    config = yaml.safe_load((PACKAGE_ROOT / 'config' / 'mit_gains.yaml').read_text())
    for group_name in ('return', 'hold', 'track', 'idle'):
        for field in ('kp', 'kd'):
            values = config[group_name][field]
            assert len(values) == 7
            assert all(isinstance(value, (int, float)) and value >= 0 for value in values)
    assert len(config['effort_limits']) == 7
    assert all(value > 0 for value in config['effort_limits'])
    assert config['track']['kp'][6] == 100.0
    assert config['track']['kd'][6] == 3.0


def test_joint_command_uses_only_fixed_size_control_arrays():
    definition = (PACKAGE_ROOT / 'msg' / 'JointCommand.msg').read_text()
    for field in ('position', 'velocity', 'kp', 'kd', 'effort'):
        assert f'float32[7] {field}' in definition
    assert 'string[]' not in definition
    assert 'float64[]' not in definition


def test_hardware_status_exposes_active_mask_and_ack_sequence():
    definition = (PACKAGE_ROOT / 'msg' / 'HardwareStatus.msg').read_text()
    assert 'uint32 last_command_sequence' in definition
    assert 'uint32 active_joint_mask' in definition
    assert 'uint16 fault_code' in definition


def test_state_only_launch_uses_read_only_republisher():
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'stm32h723_arm.launch.py'
    ).read_text()
    assert "'state_only'" in launch_source
    assert "default_value='false'" in launch_source
    assert "executable='stm32_state_republisher.py'" in launch_source
    assert 'condition=IfCondition(state_only)' in launch_source
    assert launch_source.count('condition=normal_control') == 3
    assert "'position_controller'" in launch_source
    assert "'enable_gravity_controller'" in launch_source
    assert "'enable_joint4_calibration'" in launch_source
    assert "'enable_gripper'" in launch_source
    assert "choices=['teleop', 'trajectory', 'none']" in launch_source
    assert 'condition=teleop_active' in launch_source
    assert 'condition=trajectory_active' in launch_source
    assert 'condition=gravity_active' in launch_source
    assert 'condition=gravity_tuning' in launch_source
    assert "'hardware_components_initial_state'" in launch_source
    assert "'inactive': ['Stm32H723ArmSystem']" in launch_source
    assert 'gravity_compensation_controller' in launch_source
    assert 'joint4_calibration_controller' in launch_source
    assert "'vla_position_controller', '--inactive'" in launch_source

    cmake_source = (PACKAGE_ROOT / 'CMakeLists.txt').read_text()
    assert 'scripts/stm32_state_republisher.py' in cmake_source


def test_robot_state_publisher_uses_live_stm32_joint_feedback():
    launch_source = (
        PACKAGE_ROOT / 'launch' / 'stm32h723_arm.launch.py'
    ).read_text()
    nodes = (
        node for node in ast.walk(ast.parse(launch_source))
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'Node'
        and any(
            keyword.arg == 'executable'
            and isinstance(keyword.value, ast.Constant)
            and keyword.value.value == 'robot_state_publisher'
            for keyword in node.keywords
        )
    )
    publisher = next(nodes)
    remappings = next(
        keyword.value for keyword in publisher.keywords
        if keyword.arg == 'remappings'
    )
    assert ast.literal_eval(remappings) == [
        ('/joint_states', '/stm32/joint_states')
    ]


def test_bringup_profile_exports_separate_position_and_effort_interfaces():
    xacro_path = (
        PACKAGE_ROOT / 'urdf' / 'rebotarm.arm.micro_ros.xacro')
    root = ET.parse(xacro_path).getroot()
    ros2_control = root.find('ros2_control')
    assert ros2_control is not None
    expected_joints = [f'joint{i}' for i in range(1, 7)]
    assert [joint.attrib['name'] for joint in ros2_control.findall('joint')] == (
        expected_joints)
    parameters = {
        item.attrib['name']: (item.text or '').strip()
        for item in ros2_control.findall('./hardware/param')
    }
    xacro_source = xacro_path.read_text()
    assert '<param name="required_joint_mask">0x3F</param>' in xacro_source
    assert '<param name="required_joint_mask">0x7F</param>' in xacro_source
    assert parameters['gripper_position_scale'] == '-0.008612826752'
    assert parameters['gripper_position_offset'] == '0.004282757541'
    assert 'enable_gravity_compensation' not in parameters
    assert 'model_urdf_path' not in parameters
    assert 'joint4_gravity_scale' not in parameters
    assert int(parameters['state_timeout_ms']) >= 500
    for joint in ros2_control.findall('joint'):
        assert [interface.attrib['name'] for interface in
                joint.findall('command_interface')] == ['position', 'effort']


def test_bringup_controllers_claim_joints_1_to_6():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'arm_controllers.yaml').read_text())
    expected_joints = [f'joint{i}' for i in range(1, 7)]
    assert config['arm_trajectory_controller']['ros__parameters'][
        'joints'] == expected_joints
    assert config['arm_trajectory_controller']['ros__parameters'][
        'allow_partial_joints_goal'] is True
    manager = config['controller_manager']['ros__parameters']
    assert manager['arm_trajectory_controller']['type'] == (
        'joint_trajectory_controller/JointTrajectoryController')
    assert manager['arm_teleop_controller']['type'] == (
        'position_controllers/JointGroupPositionController')
    assert manager['gripper_teleop_controller']['type'] == (
        'position_controllers/JointGroupPositionController')
    assert manager['gravity_compensation_controller']['type'] == (
        'microros_stm32h723_bridge/GravityCompensationController')
    assert manager['joint4_calibration_controller']['type'] == (
        'microros_stm32h723_bridge/Joint4CalibrationController')
    assert manager['vla_position_controller']['type'] == (
        'microros_stm32h723_bridge/VlaPositionController')
    assert config['arm_teleop_controller']['ros__parameters']['joints'] == (
        expected_joints)
    assert config['gripper_teleop_controller']['ros__parameters']['joints'] == [
        'gripper_finger_left_joint']
    broadcaster = config['joint_state_broadcaster']['ros__parameters']
    assert 'joints' not in broadcaster
    assert broadcaster['interfaces'] == ['position', 'velocity', 'effort']
    gravity = config['gravity_compensation_controller']['ros__parameters']
    assert gravity['joints'] == expected_joints
    assert gravity['gravity_scales'] == [1.0] * 6
    assert gravity['effort_limits'] == [10.0] * 6
    calibration = config['joint4_calibration_controller']['ros__parameters']
    assert calibration['joints'] == expected_joints
    assert calibration['calibration_joint'] == 'joint4'
    assert calibration['payload_frame'] == 'camera_link'
    assert calibration['nominal_payload_mass'] == 0.075
    assert 0.0 < calibration['max_correction_effort'] <= 1.5
    assert 0.0 < calibration['max_position_error'] <= 0.12
    assert 0.0 < calibration['max_calibration_velocity'] <= 0.20
    assert calibration['capture_velocity_threshold'] <= 0.01
    vla = config['vla_position_controller']['ros__parameters']
    assert vla['joints'] == expected_joints
    assert vla['chunk_topic'] == '/vla/joint_chunk'
    assert vla['command_timeout_sec'] == 0.2
    assert vla['max_delta_per_sec'] == 1.0


def test_realsense_camera_contributes_mass_to_gravity_model():
    urdf_path = PACKAGE_ROOT.parents[1] / 'rebotarm_gazebo' / 'urdf' / 'rebotarm.urdf'
    root = ET.parse(urdf_path).getroot()
    camera_link = root.find("./link[@name='camera_link']")
    assert camera_link is not None
    inertial = camera_link.find('inertial')
    assert inertial is not None
    assert float(inertial.find('mass').attrib['value']) == 0.075
    inertia = inertial.find('inertia').attrib
    assert float(inertia['ixx']) > 0.0
    assert float(inertia['iyy']) > 0.0
    assert float(inertia['izz']) > 0.0


def test_gravity_is_computed_only_by_the_effort_controller():
    hardware_source = (PACKAGE_ROOT / 'src' / 'micro_ros_system.cpp').read_text()
    controller_source = (
        PACKAGE_ROOT / 'src' / 'gravity_compensation_controller.cpp').read_text()
    cmake_source = (PACKAGE_ROOT / 'CMakeLists.txt').read_text()

    assert 'enable_gravity_compensation' not in hardware_source
    assert 'joint4_gravity_scale' not in hardware_source
    assert 'gravity_.compute' not in hardware_source
    assert 'gravity_.compute' in controller_source
    assert 'HW_IF_EFFORT' in controller_source
    assert 'HW_IF_POSITION' in controller_source
    assert (
        'target_link_libraries(${HARDWARE_LIBRARY} yaml-cpp)'
        in cmake_source)


def test_joint4_calibration_controller_is_guarded_and_regresses_multiple_poses():
    source = (
        PACKAGE_ROOT / 'src' / 'joint4_calibration_controller.cpp').read_text()
    plugin = ET.parse(
        PACKAGE_ROOT / 'gravity_compensation_controller.xml').getroot()
    plugin_names = {item.attrib['name'] for item in plugin.findall('class')}
    start_script = (PACKAGE_ROOT.parents[2] / 'scripts' /
                    'start_arm_teleop.sh').read_text()

    assert 'microros_stm32h723_bridge/Joint4CalibrationController' in plugin_names
    assert 'compute_point_mass_gravity_sensitivity' in source
    assert 'max_correction_effort_' in source
    assert 'max_position_error_' in source
    assert 'max_calibration_velocity_' in source
    assert 'stable_duration_' in source
    assert 'samples.size() < 4' in source
    for service in ('~/begin_pose', '~/release_pose', '~/capture_pose',
                    '~/fit', '~/reset'):
        assert service in source
    release_body = source.split(
        'void Joint4CalibrationController::release_pose(', 1)[1].split(
            'void Joint4CalibrationController::capture_pose(', 1)[0]
    assert 'pose_active_ = false' in release_body
    assert 'pose_faulted_ = false' in release_body
    assert 'pose_captured_ = false' in release_body
    assert 'pose fault cleared' in release_body
    assert '--joint4-calibration=true|false' in start_script
    assert 'gravity and joint4 calibration controllers claim the same' in start_script


def test_hardware_waits_for_fresh_feedback_before_activation():
    hardware_source = (PACKAGE_ROOT / 'src' / 'micro_ros_system.cpp').read_text()
    activate_source = hardware_source.split(
        'hardware_interface::CallbackReturn MicroRosSystem::on_activate', 1)[1].split(
        'hardware_interface::CallbackReturn MicroRosSystem::on_deactivate', 1)[0]
    read_source = hardware_source.split(
        'hardware_interface::return_type MicroRosSystem::read', 1)[1].split(
        'hardware_interface::return_type MicroRosSystem::write', 1)[0]
    write_source = hardware_source.split(
        'hardware_interface::return_type MicroRosSystem::write', 1)[1].split(
        'void MicroRosSystem::joint_state_callback', 1)[0]
    # Activation blocks until fresh JointState + required joint mask arrive.
    assert 'feedback_cv_.wait_for' in activate_source
    assert 'feedback_.valid && required_joints_active_.load()' in activate_source
    # read() rejects stale / missing feedback via state_timeout_.
    assert 'snapshot.valid' in read_source
    assert 'snapshot.received_at > state_timeout_' in read_source
    # Commands are gated on the active_ flag (not read()).
    assert 'if (!active_.load())' in write_source
    assert 'message.sequence = ++sequence_' in hardware_source


def test_bringup_gains_match_configured_profiles_for_joints_1_to_6():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'arm_mit_gains.yaml').read_text())
    expected_gains = {
        'return': ([30.0] * 3 + [15.0] * 3, [3.0] * 3 + [2.0] * 3),
        'hold': ([20.0] * 3 + [10.0] * 3, [2.0] * 3 + [1.5] * 3),
        'track': ([40.0] * 3 + [15.0] * 3, [3.0] * 3 + [1.5] * 3),
        'idle': ([0.0] * 6, [1.0] * 6),
    }
    for group_name, (expected_kp, expected_kd) in expected_gains.items():
        assert config[group_name]['kp'][:6] == expected_kp
        assert config[group_name]['kd'][:6] == expected_kd
    assert config['effort_limits'][:6] == [10.0] * 3 + [6.0] * 3
    for group_name in ('return', 'hold', 'track', 'idle'):
        assert config[group_name]['kp'][6] == 0.0
        assert config[group_name]['kd'][6] == 0.0
    assert config['effort_limits'][6] == 0.0


def test_gravity_mode_uses_zero_position_stiffness():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'gravity_mit_gains.yaml').read_text())
    assert config['return']['kp'] == [0.0] * 7
    assert config['hold']['kp'] == [0.0] * 7
    assert config['track']['kp'] == [0.0] * 7
    for group_name in ('return', 'hold', 'track', 'idle'):
        assert config[group_name]['kd'] == [1.0] * 6 + [0.0]
    assert config['idle']['kp'] == [0.0] * 7
    assert config['idle']['kd'] == [1.0] * 6 + [0.0]
    assert config['effort_limits'] == [10.0] * 6 + [0.0]


def test_hardware_supports_calibrated_gripper_position_conversion():
    source = (PACKAGE_ROOT / 'src' / 'micro_ros_system.cpp').read_text()
    assert '"gripper_position_scale"' in source
    assert '"gripper_position_offset"' in source
    assert '(targets[i] - position_offsets_[i]) / position_scales_[i]' in source
    assert 'message->position[item->second] * position_scales_' in source


def test_gripper_gain_profile_never_returns_to_closed_position():
    config = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'arm_gripper_mit_gains.yaml').read_text())
    assert config['track']['kp'][3:6] == [15.0] * 3
    assert config['track']['kd'][3:6] == [1.5] * 3
    assert config['return']['kp'][6] == 0.0
    assert config['return']['kd'][6] == 1.0
    assert config['hold']['kp'][6] == 2.0
    assert config['track']['kp'][6] == 100.0
    assert config['track']['kd'][6] == 3.0
    assert config['effort_limits'][6] == 2.0

    hardware_source = (PACKAGE_ROOT / 'src' / 'micro_ros_system.cpp').read_text()
    assert 'std::abs(return_gains_.kp[index]) <= 1e-9' in hardware_source

def test_vla_joint_chunk_message_is_simple():
    definition = (PACKAGE_ROOT / 'msg' / 'VlaJointChunk.msg').read_text()
    assert 'std_msgs/Header header' in definition
    assert 'float32[] positions' in definition
    assert 'float32 timeout_sec' in definition


def test_vla_position_controller_is_registered_and_position_only():
    source = (PACKAGE_ROOT / 'src' / 'vla_position_controller.cpp').read_text()
    header = (
        PACKAGE_ROOT / 'include' / 'microros_stm32h723_bridge' /
        'vla_position_controller.hpp').read_text()
    plugin = ET.parse(
        PACKAGE_ROOT / 'gravity_compensation_controller.xml').getroot()
    plugin_names = {item.attrib['name'] for item in plugin.findall('class')}
    cmake_source = (PACKAGE_ROOT / 'CMakeLists.txt').read_text()
    yaml_text = (PACKAGE_ROOT / 'config' / 'arm_controllers.yaml').read_text()

    assert 'microros_stm32h723_bridge/VlaPositionController' in plugin_names
    assert 'HW_IF_POSITION' in source
    assert 'HW_IF_EFFORT' not in source
    assert 'chunk_topic' in source
    assert 'command_timeout_sec' in source
    assert 'max_delta_per_sec' in source
    assert 'holding last positions' in source
    assert 'VlaPositionController' in header
    assert 'src/vla_position_controller.cpp' in cmake_source
    assert 'msg/VlaJointChunk.msg' in cmake_source
    assert 'scripts/mock_vla_chunk_publisher.py' in cmake_source
    assert 'Mutually exclusive with arm_teleop_controller' in yaml_text
    assert '/stm32/joint_commands' in yaml_text


def test_mock_vla_chunk_publisher_targets_default_topic():
    script = (PACKAGE_ROOT / 'scripts' / 'mock_vla_chunk_publisher.py').read_text()
    assert "declare_parameter('topic', '/vla/joint_chunk')" in script
    assert 'VlaJointChunk' in script



def test_gravity_identification_action_contract():
    definition = (PACKAGE_ROOT / 'action' / 'GravityIdentification.action').read_text()
    assert 'uint8 MODE_FULL_ARM_GRAVITY=0' in definition
    assert 'uint8 MODE_PAYLOAD_SUBMODE=1' in definition
    assert 'uint32 DEFAULT_JOINT_MASK=63' in definition
    assert 'uint8 PHASE_FAILED=10' in definition
    assert 'float32[] gravity_scales' in definition
    assert 'float32[] torque_biases_nm' in definition
    assert 'float32[] residual_nm' in definition


def test_gravity_identification_orchestrator_is_wired():
    cmake_source = (PACKAGE_ROOT / 'CMakeLists.txt').read_text()
    package_xml = (PACKAGE_ROOT / 'package.xml').read_text()
    constants = (
        PACKAGE_ROOT / 'include' / 'microros_stm32h723_bridge' /
        'gravity_identification_constants.hpp').read_text()
    node_source = (PACKAGE_ROOT / 'src' / 'gravity_identification_node.cpp').read_text()
    launch_source = (PACKAGE_ROOT / 'launch' / 'gravity_identification.launch.py').read_text()
    params = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'gravity_identification.yaml').read_text())
    controllers = yaml.safe_load(
        (PACKAGE_ROOT / 'config' / 'arm_controllers.yaml').read_text())

    assert 'gravity_identification_node' in cmake_source
    assert 'src/gravity_identification_node.cpp' in cmake_source
    assert 'test_gravity_identification_fit' in cmake_source
    assert '<depend>rclcpp_action</depend>' in package_xml
    assert '<depend>controller_manager_msgs</depend>' in package_xml
    assert 'ACTION_SERVER_NAME = "/StartGravityID"' in constants
    assert 'MODE_PAYLOAD_SUBMODE' in node_source
    assert 'abort_to_hold' in node_source
    assert 'MODE_TRACK' in node_source
    assert 'stop_controllers' in node_source
    assert '/stm32/joint_commands' in node_source
    assert '/stm32/joint_states' in node_source
    assert 'gravity_identification_node' in launch_source
    node_params = params['gravity_identification']['ros__parameters']
    assert node_params['command_topic'] == '/stm32/joint_commands'
    assert node_params['state_topic'] == '/stm32/joint_states'
    assert len(node_params['pose_amplitudes']) == 6
    assert node_params['pose_amplitudes'][0] != node_params['pose_amplitudes'][3]
    gc = controllers['gravity_compensation_controller']['ros__parameters']
    assert 'torque_biases' in gc
    assert len(gc['torque_biases']) == 6


def test_gravity_compensation_supports_biases_and_hot_reload():
    source = (PACKAGE_ROOT / 'src' / 'gravity_compensation_controller.cpp').read_text()
    assert 'torque_biases' in source
    assert 'on_parameters_set' in source
    assert 'add_on_set_parameters_callback' in source
