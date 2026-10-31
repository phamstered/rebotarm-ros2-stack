#include <gtest/gtest.h>
#include <algorithm>
#include <limits>
#include <filesystem>
#include <fstream>
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
  static void velocity(MicroRosSystem & system, std::size_t index, double value)
  {system.command_velocity_[index] = value;}
  static void integral(MicroRosSystem & system)
  {
    TrajectoryIntegral::Values ki{}, caps{}, rates{};
    for (unsigned i = 0; i < 3; ++i) {ki[i] = 4.0; caps[i] = 1.0; rates[i] = .05;}
    system.trajectory_integral_.configure(ki, caps, rates, system.effort_limits_);
    system.trajectory_integral_configured_ = true;
    system.status_.mode = msg::HardwareStatus::MODE_TRACK;
    system.status_.active_joint_mask = 0x7f;
    system.status_received_at_ = std::chrono::steady_clock::now();
  }
  static void status(MicroRosSystem & system, uint8_t mode, uint32_t sequence, uint32_t fault = 0)
  {
    auto status = std::make_shared<msg::HardwareStatus>();
    status->mode = mode; status->active_joint_mask = 0x7f;
    status->last_command_sequence = sequence; status->fault_code = fault;
    system.status_callback(status);
  }
  static void stale_status(MicroRosSystem & system)
  {system.status_received_at_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);}
  static double trim(const MicroRosSystem & system, unsigned i)
  {return system.trajectory_integral_.value(i);}
  static void load_integral(MicroRosSystem & system, const std::string & path)
  {system.load_trajectory_integral(path);}
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

TEST(TrajectoryIntegralHardware, OnlyOwnedTrackIntegratesAndModeSwitchesClearTrim)
{
  MicroRosSystem system;
  MicroRosSystemTestAccess::prepare(system);
  MicroRosSystemTestAccess::integral(system);
  MicroRosSystemTestAccess::write(system);
  EXPECT_DOUBLE_EQ(MicroRosSystemTestAccess::trim(system, 0), 0.0);
  ASSERT_EQ(system.perform_command_mode_switch({"joint1/position", "joint1/velocity"}, {}),
    hardware_interface::return_type::OK);
  MicroRosSystemTestAccess::write(system);
  EXPECT_NEAR(MicroRosSystemTestAccess::trim(system, 0), .0005, 1e-12);
  EXPECT_DOUBLE_EQ(MicroRosSystemTestAccess::trim(system, 1), 0.0);
  // Even a position-owner replacement in one switch must not inherit trim.
  ASSERT_EQ(system.perform_command_mode_switch({"joint1/position"}, {"joint1/position"}),
    hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(MicroRosSystemTestAccess::trim(system, 0), 0.0);
  MicroRosSystemTestAccess::write(system);
  EXPECT_GT(MicroRosSystemTestAccess::trim(system, 0), 0.0);
  ASSERT_EQ(system.perform_command_mode_switch({}, {"joint1/position", "joint1/velocity"}),
    hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(MicroRosSystemTestAccess::trim(system, 0), 0.0);
  ASSERT_EQ(system.perform_command_mode_switch({"joint1/position", "joint1/velocity"}, {}),
    hardware_interface::return_type::OK);
  MicroRosSystemTestAccess::write(system);
  EXPECT_GT(MicroRosSystemTestAccess::trim(system, 0), 0.0);
  MicroRosSystemTestAccess::hold(system);
  MicroRosSystemTestAccess::write(system);
  EXPECT_DOUBLE_EQ(MicroRosSystemTestAccess::trim(system, 0), 0.0);
  MicroRosSystemTestAccess::returning(system);
  MicroRosSystemTestAccess::write(system);
  EXPECT_DOUBLE_EQ(MicroRosSystemTestAccess::trim(system, 0), 0.0);
  MicroRosSystemTestAccess::disable(system);
  MicroRosSystemTestAccess::write(system);
  EXPECT_DOUBLE_EQ(MicroRosSystemTestAccess::trim(system, 0), 0.0);
}

TEST(TrajectoryIntegralHardware, InvalidLaterJointDoesNotCommitPartialIntegralState)
{
  MicroRosSystem system;
  MicroRosSystemTestAccess::prepare(system);
  MicroRosSystemTestAccess::integral(system);
  ASSERT_EQ(system.perform_command_mode_switch(
      {"joint1/position", "joint1/velocity", "joint2/position", "joint2/velocity"}, {}),
    hardware_interface::return_type::OK);
  MicroRosSystemTestAccess::velocity(system, 1, std::numeric_limits<double>::quiet_NaN());
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)),
    hardware_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(MicroRosSystemTestAccess::trim(system, 0), 0.0);
  MicroRosSystemTestAccess::velocity(system, 1, 0.0);
  MicroRosSystemTestAccess::write(system);
  EXPECT_NEAR(MicroRosSystemTestAccess::trim(system, 0), .0005, 1e-12);
}

TEST(TrajectoryIntegralHardware, MissingOptionalConfigurationPreservesLegacyAndBadPathFails)
{
  MicroRosSystem system;
  MicroRosSystemTestAccess::prepare(system);
  EXPECT_NO_THROW(MicroRosSystemTestAccess::load_integral(system, ""));
  EXPECT_THROW(MicroRosSystemTestAccess::load_integral(system,
      "/this/file/does/not/exist/integral.yaml"), std::exception);
  MicroRosSystemTestAccess::write(system);
  EXPECT_DOUBLE_EQ(MicroRosSystemTestAccess::trim(system, 0), 0.0);
}

TEST(TrajectoryIntegralHardware, PublishesTrimOnlyForClaimedJointAndDisableZerosEveryField)
{
  MicroRosSystem system;
  MicroRosSystemTestAccess::prepare(system);
  MicroRosSystemTestAccess::integral(system);
  auto node = MicroRosSystemTestAccess::node(system);
  std::vector<msg::JointCommand> messages;
  auto sub = node->create_subscription<msg::JointCommand>(
    "/robot_console_tests/stop_commands", rclcpp::QoS(100).best_effort(),
    [&messages](const msg::JointCommand::SharedPtr value) {messages.push_back(*value);});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (sub->get_publisher_count() == 0 && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  auto frame = [&]() {
      MicroRosSystemTestAccess::status(system, msg::HardwareStatus::MODE_TRACK, 0);
      MicroRosSystemTestAccess::write(system);
      const auto sequence = MicroRosSystemTestAccess::sequence(system);
      while ((messages.empty() || messages.back().sequence != sequence) &&
        std::chrono::steady_clock::now() < deadline)
      {
        rclcpp::spin_some(node); std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
      EXPECT_FALSE(messages.empty());
      return messages.empty() ? msg::JointCommand() : messages.back();
    };
  ASSERT_EQ(system.perform_command_mode_switch({"joint1/position", "joint1/velocity"}, {}),
    hardware_interface::return_type::OK);
  auto message = frame();
  EXPECT_NEAR(message.effort[0], 1.0005F, 1e-6);
  EXPECT_FLOAT_EQ(message.effort[1], 1.0F);
  MicroRosSystemTestAccess::disable(system);
  message = frame();
  EXPECT_EQ(message.mode, msg::JointCommand::MODE_DISABLE);
  for (unsigned i = 0; i < 7; ++i) {
    EXPECT_FLOAT_EQ(message.position[i], 0.0F); EXPECT_FLOAT_EQ(message.velocity[i], 0.0F);
    EXPECT_FLOAT_EQ(message.kp[i], 0.0F); EXPECT_FLOAT_EQ(message.kd[i], 0.0F);
    EXPECT_FLOAT_EQ(message.effort[i], 0.0F);
  }
}

TEST(TrajectoryIntegralHardware, LoadsSeparateYamlAndRejectsInvalidSettings)
{
  MicroRosSystem system;
  MicroRosSystemTestAccess::prepare(system);
  MicroRosSystemTestAccess::integral(system);
  const auto path = std::filesystem::temp_directory_path() /
    ("integral-settings-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".yaml");
  struct Cleanup {std::filesystem::path path; ~Cleanup() {std::filesystem::remove(path);}} cleanup{path};
  auto write = [&path](const std::string & content) {std::ofstream(path) << content;};
  write("ki: [4,0,0,0,0,0,0]\nmax_effort: [1,0,0,0,0,0,0]\nslew_rate: [0.05,0,0,0,0,0,0]\n");
  ASSERT_NO_THROW(MicroRosSystemTestAccess::load_integral(system, path.string()));
  ASSERT_EQ(system.perform_command_mode_switch({"joint1/position", "joint1/velocity"}, {}),
    hardware_interface::return_type::OK);
  MicroRosSystemTestAccess::write(system);
  EXPECT_NEAR(MicroRosSystemTestAccess::trim(system, 0), .0005, 1e-12);
  write("ki: [4]\nmax_effort: [1]\nslew_rate: [0.05]\n");
  EXPECT_THROW(MicroRosSystemTestAccess::load_integral(system, path.string()), std::exception);
  write("ki: [.nan,0,0,0,0,0,0]\nmax_effort: [1,0,0,0,0,0,0]\nslew_rate: [0.05,0,0,0,0,0,0]\n");
  EXPECT_THROW(MicroRosSystemTestAccess::load_integral(system, path.string()), std::exception);
}

TEST(TrajectoryIntegralHardware, McuResetFaultAndStaleStatusCannotRetainOldTrim)
{
  MicroRosSystem system;
  MicroRosSystemTestAccess::prepare(system);
  MicroRosSystemTestAccess::integral(system);
  ASSERT_EQ(system.perform_command_mode_switch({"joint1/position", "joint1/velocity"}, {}),
    hardware_interface::return_type::OK);
  for (unsigned i = 0; i < 5; ++i) {MicroRosSystemTestAccess::write(system);}
  EXPECT_GT(MicroRosSystemTestAccess::trim(system, 0), .002);
  // Even if DISABLE and the next TRACK arrive between writes, clear old trim.
  MicroRosSystemTestAccess::status(system, msg::HardwareStatus::MODE_DISABLE, 0);
  MicroRosSystemTestAccess::status(system, msg::HardwareStatus::MODE_TRACK, 20);
  MicroRosSystemTestAccess::write(system);
  EXPECT_NEAR(MicroRosSystemTestAccess::trim(system, 0), .0005, 1e-12);
  MicroRosSystemTestAccess::write(system);
  MicroRosSystemTestAccess::status(system, msg::HardwareStatus::MODE_TRACK, 1);
  MicroRosSystemTestAccess::write(system);
  EXPECT_NEAR(MicroRosSystemTestAccess::trim(system, 0), .0005, 1e-12);
  MicroRosSystemTestAccess::status(system, msg::HardwareStatus::MODE_TRACK, 2, 11);
  MicroRosSystemTestAccess::write(system);
  EXPECT_DOUBLE_EQ(MicroRosSystemTestAccess::trim(system, 0), 0.0);
  MicroRosSystemTestAccess::status(system, msg::HardwareStatus::MODE_TRACK, 3);
  MicroRosSystemTestAccess::write(system);
  EXPECT_GT(MicroRosSystemTestAccess::trim(system, 0), 0.0);
  MicroRosSystemTestAccess::stale_status(system);
  MicroRosSystemTestAccess::write(system);
  EXPECT_DOUBLE_EQ(MicroRosSystemTestAccess::trim(system, 0), 0.0);
}

}  // namespace microros_stm32h723_bridge
