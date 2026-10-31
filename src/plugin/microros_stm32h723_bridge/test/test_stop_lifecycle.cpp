#include <gtest/gtest.h>
#include <algorithm>
#include <limits>
#include "microros_stm32h723_bridge/micro_ros_system.hpp"

namespace microros_stm32h723_bridge
{
// Exercise the actual write/deactivate implementation with isolated ROS topics.
struct MicroRosSystemTestAccess
{
  static void prepare(MicroRosSystem & system)
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    system.node_ = std::make_shared<rclcpp::Node>("stop_lifecycle_test");
    system.command_publisher_ = system.node_->create_publisher<msg::JointCommand>(
      "/robot_console_tests/stop_commands", rclcpp::QoS(100).best_effort());
    system.active_.store(true);
    system.configured_joint_mask_ = 0x7f;
    system.configured_protocol_indices_ = {0, 1, 2, 3, 4, 5, 6};
    system.joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper_finger_left_joint"};
    system.velocity_available_.fill(true);
    system.velocity_available_[6] = false;
    system.position_scales_.fill(1.0);
    system.track_gains_.kp.fill(40.0);
    system.track_gains_.kd.fill(3.0);
    system.effort_limits_.fill(10.0);
    system.lower_limits_.fill(-3.0);
    system.upper_limits_.fill(3.0);
    system.command_position_.fill(0.2);
    system.command_velocity_.fill(0.1);
    system.command_effort_.fill(1.0);
    system.idle_gains_.kp.fill(0.0);
    system.idle_gains_.kd.fill(1.0);
    system.return_gains_.kp.fill(10.0);
    system.return_gains_.kd.fill(1.0);
  }

  static void velocity(MicroRosSystem & system, double value) {system.command_velocity_.fill(value);}
  static void legacy(MicroRosSystem & system) {system.velocity_available_.fill(false);}
  static void hold(MicroRosSystem & system)
  {
    system.command_mode_.store(msg::JointCommand::MODE_HOLD);
    system.command_position_ = system.state_position_;
    system.command_effort_.fill(0.0);
  }
  static void returning(MicroRosSystem & system)
  {
    system.safe_stop_requested_.store(true);
    system.command_mode_.store(msg::JointCommand::MODE_RETURN);
    system.return_start_.fill(0.2);
    system.return_started_at_ = std::chrono::steady_clock::now();
  }

  static void write(MicroRosSystem & system)
  {
    ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
      hardware_interface::return_type::OK);
  }

  static void completed_return(MicroRosSystem & system)
  {
    system.safe_stop_requested_.store(true);
    system.command_mode_.store(msg::JointCommand::MODE_RETURN);
    system.return_started_at_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    system.state_position_.fill(0.0);
  }

  static void assert_disabled(const MicroRosSystem & system)
  {
    ASSERT_EQ(system.command_mode_.load(), msg::JointCommand::MODE_DISABLE);
  }

  static void disable(MicroRosSystem & system)
  {
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    system.request_disable(std::make_shared<std_srvs::srv::Trigger::Request>(), response);
    ASSERT_TRUE(response->success);
  }

  static void reject_inactive_disable(MicroRosSystem & system)
  {
    system.active_.store(false);
    auto response = std::make_shared<std_srvs::srv::Trigger::Response>();
    system.request_disable(std::make_shared<std_srvs::srv::Trigger::Request>(), response);
    EXPECT_FALSE(response->success);
    EXPECT_FALSE(system.disable_requested_.load());
  }

  static void acknowledge(MicroRosSystem & system, uint32_t sequence, uint8_t mode, uint16_t fault)
  {
    auto status = std::make_shared<msg::HardwareStatus>();
    status->last_command_sequence = sequence;
    status->mode = mode;
    status->fault_code = fault;
    system.status_callback(status);
  }

  static uint32_t first_disable(const MicroRosSystem & system) {return system.disable_sequence_;}
  static uint32_t sequence(const MicroRosSystem & system) {return system.sequence_;}
  static void near_wrap(MicroRosSystem & system)
  {
    system.sequence_ = std::numeric_limits<uint32_t>::max() - 1;
  }

  static std::shared_ptr<rclcpp::Node> node(MicroRosSystem & system) {return system.node_;}
};

TEST(StopLifecycle, CompletedReturnRemainsDisabledAndSequencesAdvance)
{
  MicroRosSystem system;
  MicroRosSystemTestAccess::prepare(system);
  MicroRosSystemTestAccess::completed_return(system);
  for (unsigned frame = 1; frame <= 100; ++frame) {
    MicroRosSystemTestAccess::write(system);
    MicroRosSystemTestAccess::assert_disabled(system);
    EXPECT_EQ(MicroRosSystemTestAccess::sequence(system), frame);
    EXPECT_EQ(MicroRosSystemTestAccess::first_disable(system), 1U);
  }
}

TEST(StopLifecycle, InactiveHardwareRejectsDisableRequest)
{
  MicroRosSystem system;
  MicroRosSystemTestAccess::prepare(system);
  MicroRosSystemTestAccess::reject_inactive_disable(system);
}

TEST(StopLifecycle, ExplicitDisableZerosAllFieldsAndHandlesSequenceWrap)
{
  MicroRosSystem system;
  MicroRosSystemTestAccess::prepare(system);
  auto node = MicroRosSystemTestAccess::node(system);
  std::vector<msg::JointCommand> messages;
  auto subscription = node->create_subscription<msg::JointCommand>(
    "/robot_console_tests/stop_commands", rclcpp::QoS(100).best_effort(),
    [&messages](const msg::JointCommand::SharedPtr message) {messages.push_back(*message);});
  MicroRosSystemTestAccess::disable(system);
  MicroRosSystemTestAccess::near_wrap(system);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (subscription->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  for (unsigned frame = 0; frame < 3; ++frame) {
    MicroRosSystemTestAccess::write(system);
    rclcpp::spin_some(node);
  }
  while (messages.size() < 3 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(messages.size(), 3U);
  EXPECT_EQ(messages[0].sequence, std::numeric_limits<uint32_t>::max());
  EXPECT_EQ(messages[1].sequence, 1U);
  EXPECT_EQ(messages[2].sequence, 2U);
  for (const auto & message : messages) {
    EXPECT_EQ(message.mode, msg::JointCommand::MODE_DISABLE);
    for (std::size_t i = 0; i < 7; ++i) {
      EXPECT_EQ(message.position[i], 0.0F);
      EXPECT_EQ(message.velocity[i], 0.0F);
      EXPECT_EQ(message.kp[i], 0.0F);
      EXPECT_EQ(message.kd[i], 0.0F);
      EXPECT_EQ(message.effort[i], 0.0F);
    }
  }
}

TEST(StopLifecycle, DeactivateRequiresFaultFreeDisableAcknowledgement)
{
  MicroRosSystem system;
  MicroRosSystemTestAccess::prepare(system);
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  MicroRosSystemTestAccess::disable(system);
  MicroRosSystemTestAccess::write(system);
  MicroRosSystemTestAccess::write(system);
  MicroRosSystemTestAccess::acknowledge(system, 0, 3, 0);
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  MicroRosSystemTestAccess::acknowledge(system, 100, 3, 0);
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  MicroRosSystemTestAccess::acknowledge(system, 2, 0, 0);
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  MicroRosSystemTestAccess::acknowledge(system, 2, 3, 11);
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
  MicroRosSystemTestAccess::acknowledge(system, 2, 3, 0);
  EXPECT_EQ(system.on_deactivate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
}


TEST(TrajectoryVelocity, OptionalInterfacesAndOwnershipSwitchClearResidualVelocity)
{
  MicroRosSystem system;
  MicroRosSystemTestAccess::prepare(system);
  auto interfaces = system.export_command_interfaces();
  EXPECT_EQ(interfaces.size(), 20U);  // 6 x position/effort/velocity + gripper position/effort
  EXPECT_EQ(system.prepare_command_mode_switch({"joint1/velocity"}, {}),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(system.prepare_command_mode_switch({"gripper_finger_left_joint/position", "gripper_finger_left_joint/velocity"}, {}),
    hardware_interface::return_type::ERROR);
  ASSERT_EQ(system.perform_command_mode_switch({"joint1/position", "joint1/velocity"}, {}),
    hardware_interface::return_type::OK);
  auto velocity = std::find_if(interfaces.begin(), interfaces.end(), [](const auto & interface) {
      return interface.get_name() == "joint1/velocity";
    });
  ASSERT_NE(velocity, interfaces.end());
  EXPECT_EQ(velocity->get_value(), 0.0);
  velocity->set_value(0.17);
  EXPECT_EQ(system.prepare_command_mode_switch({}, {"joint1/position"}),
    hardware_interface::return_type::ERROR);
  ASSERT_EQ(system.perform_command_mode_switch({}, {"joint1/position", "joint1/velocity"}),
    hardware_interface::return_type::OK);
  EXPECT_EQ(velocity->get_value(), 0.0);
  ASSERT_EQ(system.perform_command_mode_switch({"joint1/position", "joint1/velocity"}, {}),
    hardware_interface::return_type::OK);
  EXPECT_EQ(velocity->get_value(), 0.0);
  MicroRosSystem legacy;
  MicroRosSystemTestAccess::prepare(legacy);
  MicroRosSystemTestAccess::legacy(legacy);
  EXPECT_EQ(legacy.export_command_interfaces().size(), 14U);
  EXPECT_EQ(legacy.perform_command_mode_switch({"joint1/position"}, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(legacy.prepare_command_mode_switch({"joint1/velocity"}, {}), hardware_interface::return_type::ERROR);
}

TEST(TrajectoryVelocity, TrackForwardsOwnedVelocityButHoldReturnAndDisableDoNot)
{
  MicroRosSystem system;
  MicroRosSystemTestAccess::prepare(system);
  auto node = MicroRosSystemTestAccess::node(system);
  std::vector<msg::JointCommand> messages;
  auto subscription = node->create_subscription<msg::JointCommand>(
    "/robot_console_tests/stop_commands", rclcpp::QoS(100).best_effort(),
    [&messages](const msg::JointCommand::SharedPtr message) {messages.push_back(*message);});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (subscription->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  auto frame = [&]() {
      const auto count = messages.size();
      MicroRosSystemTestAccess::write(system);
      while (messages.size() == count && std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      EXPECT_GT(messages.size(), count);
      return messages.empty() ? msg::JointCommand() : messages.back();
    };
  ASSERT_EQ(system.perform_command_mode_switch({"joint1/position", "joint1/velocity"}, {}),
    hardware_interface::return_type::OK);
  MicroRosSystemTestAccess::velocity(system, 0.12);
  auto message = frame();
  EXPECT_EQ(message.mode, msg::JointCommand::MODE_TRACK);
  EXPECT_NEAR(message.velocity[0], 0.12F, 1e-6);
  EXPECT_EQ(message.velocity[1], 0.0F);  // Position-only Teleop does not inherit velocity.
  EXPECT_EQ(message.velocity[6], 0.0F);
  MicroRosSystemTestAccess::velocity(system, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)),
    hardware_interface::return_type::ERROR);
  MicroRosSystemTestAccess::velocity(system, std::numeric_limits<double>::max());
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)),
    hardware_interface::return_type::ERROR);
  MicroRosSystemTestAccess::hold(system);
  message = frame(); EXPECT_EQ(message.mode, msg::JointCommand::MODE_HOLD);
  for (const auto value : message.velocity) {EXPECT_EQ(value, 0.0F);}
  MicroRosSystemTestAccess::returning(system);
  message = frame(); EXPECT_EQ(message.mode, msg::JointCommand::MODE_RETURN);
  for (const auto value : message.velocity) {EXPECT_EQ(value, 0.0F);}
  MicroRosSystemTestAccess::disable(system);
  message = frame(); EXPECT_EQ(message.mode, msg::JointCommand::MODE_DISABLE);
  for (const auto value : message.velocity) {EXPECT_EQ(value, 0.0F);}
}

}  // namespace microros_stm32h723_bridge
