#include "microros_stm32h723_bridge/gravity_compensation.hpp"

#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>
#include <limits>
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
  std::size_t joint4_hardware_index{std::numeric_limits<std::size_t>::max()};
  double joint4_scale{0.85};
  bool ready{false};
};

GravityCompensation::GravityCompensation()
: impl_(std::make_unique<Impl>())
{
}

GravityCompensation::~GravityCompensation() = default;

void GravityCompensation::initialize(
  const std::string & urdf_path,
  const std::vector<std::string> & hardware_joint_names,
  double joint4_scale)
{
  if (hardware_joint_names.empty()) {
    throw std::invalid_argument("hardware joint list is empty");
  }

  pinocchio::urdf::buildModel(urdf_path, impl_->model);
  impl_->data = std::make_unique<pinocchio::Data>(impl_->model);
  impl_->model_joint_ids.clear();
  impl_->joint4_hardware_index = std::numeric_limits<std::size_t>::max();
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
    if (name == "joint4") {
      impl_->joint4_hardware_index = hardware_index;
    }
  }

  impl_->joint4_scale = joint4_scale;
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
  if (impl_->joint4_hardware_index < result.size()) {
    result[impl_->joint4_hardware_index] *= impl_->joint4_scale;
  }
  return result;
}

bool GravityCompensation::initialized() const noexcept
{
  return impl_->ready;
}

}  // namespace microros_stm32h723_bridge
