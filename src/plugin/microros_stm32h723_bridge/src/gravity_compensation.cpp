#include "microros_stm32h723_bridge/gravity_compensation.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>
#include <stdexcept>
#include <utility>

namespace microros_stm32h723_bridge
{

class GravityCompensation::Impl
{
public:
  pinocchio::Model model;
  std::unique_ptr<pinocchio::Data> data;
  std::vector<pinocchio::JointIndex> model_joint_ids;
  bool ready{false};
};

GravityCompensation::GravityCompensation()
: impl_(std::make_unique<Impl>())
{
}

GravityCompensation::~GravityCompensation() = default;

void GravityCompensation::initialize(
  const std::string & urdf_path,
  const std::vector<std::string> & hardware_joint_names)
{
  if (hardware_joint_names.empty()) {
    throw std::invalid_argument("hardware joint list is empty");
  }

  pinocchio::urdf::buildModel(urdf_path, impl_->model);
  impl_->data = std::make_unique<pinocchio::Data>(impl_->model);
  impl_->model_joint_ids.clear();
  impl_->model_joint_ids.reserve(hardware_joint_names.size());

  for (std::size_t hardware_index = 0; hardware_index < hardware_joint_names.size();
    ++hardware_index)
  {
    const auto & name = hardware_joint_names[hardware_index];
    const auto joint_id = impl_->model.getJointId(name);
    if (joint_id == 0 || joint_id >= impl_->model.joints.size()) {
      throw std::runtime_error("joint not found in Pinocchio model: " + name);
    }
    const auto & joint = impl_->model.joints[joint_id];
    if (joint.nq() != 1 || joint.nv() != 1) {
      throw std::runtime_error("only 1-DoF hardware joints are supported: " + name);
    }
    impl_->model_joint_ids.push_back(joint_id);
  }

  impl_->ready = true;
}

std::vector<double> GravityCompensation::compute(
  const std::vector<double> & hardware_positions)
{
  if (!impl_->ready || hardware_positions.size() != impl_->model_joint_ids.size()) {
    throw std::runtime_error("gravity compensation is not initialized or input size is invalid");
  }

  Eigen::VectorXd q = pinocchio::neutral(impl_->model);
  for (std::size_t i = 0; i < hardware_positions.size(); ++i) {
    const auto joint_id = impl_->model_joint_ids[i];
    q[impl_->model.joints[joint_id].idx_q()] = hardware_positions[i];
  }

  const Eigen::VectorXd & model_effort =
    pinocchio::computeGeneralizedGravity(impl_->model, *impl_->data, q);
  std::vector<double> result(hardware_positions.size(), 0.0);
  for (std::size_t i = 0; i < result.size(); ++i) {
    const auto joint_id = impl_->model_joint_ids[i];
    result[i] = model_effort[impl_->model.joints[joint_id].idx_v()];
  }
  return result;
}

std::vector<double> GravityCompensation::compute_point_mass_gravity_sensitivity(
  const std::vector<double> & hardware_positions, const std::string & frame_name)
{
  if (!impl_->ready || hardware_positions.size() != impl_->model_joint_ids.size()) {
    throw std::runtime_error("gravity compensation is not initialized or input size is invalid");
  }

  const auto frame_id = impl_->model.getFrameId(frame_name);
  if (frame_id >= impl_->model.frames.size()) {
    throw std::runtime_error("frame not found in Pinocchio model: " + frame_name);
  }

  Eigen::VectorXd q = pinocchio::neutral(impl_->model);
  for (std::size_t i = 0; i < hardware_positions.size(); ++i) {
    const auto joint_id = impl_->model_joint_ids[i];
    q[impl_->model.joints[joint_id].idx_q()] = hardware_positions[i];
  }

  pinocchio::computeJointJacobians(impl_->model, *impl_->data, q);
  pinocchio::updateFramePlacements(impl_->model, *impl_->data);
  Eigen::MatrixXd frame_jacobian = Eigen::MatrixXd::Zero(6, impl_->model.nv);
  pinocchio::getFrameJacobian(
    impl_->model, *impl_->data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED,
    frame_jacobian);

  // Pinocchio's gravity vector is the free-fall acceleration.  The actuator
  // torque required to hold a one-kilogram point mass is -J_linear^T * g.
  const Eigen::VectorXd model_sensitivity =
    -frame_jacobian.topRows<3>().transpose() * impl_->model.gravity.linear();
  std::vector<double> result(hardware_positions.size(), 0.0);
  for (std::size_t i = 0; i < result.size(); ++i) {
    const auto joint_id = impl_->model_joint_ids[i];
    result[i] = model_sensitivity[impl_->model.joints[joint_id].idx_v()];
  }
  return result;
}

bool GravityCompensation::initialized() const noexcept
{
  return impl_->ready;
}

}  // namespace microros_stm32h723_bridge
