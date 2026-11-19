#ifndef MICROROS_STM32H723_BRIDGE__MOTOR_MODE_GATE_HPP_
#define MICROROS_STM32H723_BRIDGE__MOTOR_MODE_GATE_HPP_

#include <array>
#include <chrono>
#include <cstdint>

namespace microros_stm32h723_bridge
{

// No ROS dependency: the same admission decisions can be tested on a host.
// Callers serialize access. Seeing a V2 endpoint is sticky for this lifecycle.
class MotorModeGate
{
public:
  using Clock = std::chrono::steady_clock;
  struct Snapshot
  {
    uint8_t protocol_version{0};
    uint64_t boot_id{0};
    uint64_t source_stamp{0};
    uint32_t control_epoch{0};
    uint32_t last_command_sequence{0};
    uint64_t owner_source_id{0};
    uint32_t lease_id{0};
    uint32_t verified_joint_mask{0};
    uint32_t enabled_joint_mask{0};
    bool disabled_verified{false};
    std::array<uint8_t, 7> control_modes{};
    bool switching{false};
    uint8_t operation_state{0};
    uint8_t mode{0};
    uint16_t fault_code{0};
    uint16_t error_code{0};
  };

  void discovered() {observed_ = true;}

  bool command_sequence(uint32_t & sequence) const
  {
    if (snapshot_.boot_id == 0) {return false;}
    sequence = snapshot_.last_command_sequence;
    return true;
  }

  // Returns true on a boot change. The caller terminates an active lifecycle.
  bool observe(const Snapshot & next, Clock::time_point now)
  {
    observed_ = true;
    const bool reboot = next.boot_id != 0 && snapshot_.boot_id != 0 &&
      next.boot_id != snapshot_.boot_id;
    if (identity_exhausted_) {return false;}
    for (std::size_t index = 0; index < retired_count_; ++index) {
      if (next.boot_id == retired_boots_[index]) {return false;}
    }
    if (next.protocol_version != 2 || next.boot_id == 0 || next.source_stamp == 0 ||
      next.control_epoch == 0 || next.operation_state > 4)
    {
      valid_ = false;
      return reboot;
    }
    if (!reboot && next.boot_id == snapshot_.boot_id &&
      next.source_stamp <= snapshot_.source_stamp)
    {
      return false;  // Replays never extend source freshness.
    }
    if (!reboot && next.boot_id == snapshot_.boot_id &&
      next.control_epoch < snapshot_.control_epoch)
    {
      valid_ = false;
      return false;  // Keep the epoch high-water mark after a rollback.
    }
    if (reboot) {
      if (retired_count_ == retired_boots_.size()) {
        valid_ = false;
        identity_exhausted_ = true;
        return true;  // Reconstruct the hardware instance before another boot.
      }
      retired_boots_[retired_count_++] = snapshot_.boot_id;
    }
    valid_ = true;
    snapshot_ = next;
    received_at_ = now;
    return reboot;
  }

  bool allows_mit(
    uint32_t required_mask, bool require_v2, Clock::time_point now,
    std::chrono::milliseconds timeout) const
  {
    if (!observed_) {return !require_v2;}
    if (!valid_ || now < received_at_ || now - received_at_ >= timeout ||
      snapshot_.switching || snapshot_.operation_state == 1 ||
      snapshot_.owner_source_id != 0 || snapshot_.lease_id != 0 ||
      snapshot_.fault_code != 0 || snapshot_.error_code != 0 ||
      required_mask == 0 || (required_mask & ~0x7fU) != 0 ||
      (snapshot_.verified_joint_mask & required_mask) != required_mask)
    {
      return false;
    }
    for (std::size_t index = 0; index < snapshot_.control_modes.size(); ++index) {
      if ((required_mask & (1U << index)) && snapshot_.control_modes[index] != 1) {
        return false;
      }
    }
    return true;
  }

  bool confirms_disable(
    uint32_t first_sequence, uint32_t last_sent_sequence,
    Clock::time_point now, std::chrono::milliseconds timeout) const
  {
    if (!observed_) {return true;}  // Legacy lifecycle keeps its existing proof.
    return valid_ && now >= received_at_ && now - received_at_ < timeout &&
      snapshot_.mode == 3 && snapshot_.enabled_joint_mask == 0 && snapshot_.disabled_verified &&
      static_cast<int32_t>(snapshot_.last_command_sequence - first_sequence) >= 0 &&
      static_cast<int32_t>(last_sent_sequence - snapshot_.last_command_sequence) >= 0;
  }

private:
  Snapshot snapshot_;
  Clock::time_point received_at_{};
  bool observed_{false};
  bool valid_{false};
  std::array<uint64_t, 16> retired_boots_{};
  std::size_t retired_count_{0};
  bool identity_exhausted_{false};
};

}  // namespace microros_stm32h723_bridge
#endif
