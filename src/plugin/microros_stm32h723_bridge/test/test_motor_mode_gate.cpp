#include <cassert>
#include <chrono>
#include "microros_stm32h723_bridge/motor_mode_gate.hpp"

using microros_stm32h723_bridge::MotorModeGate;
using namespace std::chrono_literals;

int main()
{
  const auto now = MotorModeGate::Clock::now();
  MotorModeGate legacy;
  assert(legacy.allows_mit(0x7f, false, now, 1s));
  assert(!legacy.allows_mit(0x7f, true, now, 1s));
  legacy.discovered();
  assert(!legacy.allows_mit(0x7f, false, now, 1s));

  MotorModeGate::Snapshot status;
  status.protocol_version = 2;
  status.boot_id = 12;
  status.source_stamp = 100;
  status.control_epoch = 3;
  status.verified_joint_mask = 0x7f;
  status.control_modes.fill(1);
  assert(!legacy.observe(status, now));
  assert(legacy.allows_mit(0x7f, true, now, 1s));
  assert(!legacy.allows_mit(0x7f, false, now + 1s, 1s));
  legacy.observe(status, now + 900ms);  // Replay does not refresh reception age.
  assert(!legacy.allows_mit(0x7f, false, now + 1s, 1s));

  const auto reject = [&](MotorModeGate::Snapshot changed) {
      MotorModeGate gate;
      gate.observe(changed, now);
      assert(!gate.allows_mit(0x7f, false, now, 1s));
    };
  auto changed = status;
  changed.control_modes[6] = 2;
  reject(changed);
  changed = status; changed.verified_joint_mask = 0x3f; reject(changed);
  changed = status; changed.owner_source_id = 99; reject(changed);
  changed = status; changed.lease_id = 1; reject(changed);
  changed = status; changed.switching = true; reject(changed);
  changed = status; changed.operation_state = 1; reject(changed);
  changed = status; changed.error_code = 12; reject(changed);
  changed = status; changed.fault_code = 11; reject(changed);
  changed = status; changed.protocol_version = 3; reject(changed);
  changed = status; changed.boot_id = 0; reject(changed);
  changed = status; changed.source_stamp = 0; reject(changed);
  changed = status; changed.control_epoch = 0; reject(changed);
  changed = status; changed.operation_state = 99; reject(changed);

  status.source_stamp++;
  legacy.observe(status, now + 1s);
  assert(legacy.allows_mit(0x7f, true, now + 1s, 1s));
  changed = status; changed.control_epoch--; changed.source_stamp++;
  legacy.observe(changed, now + 1s);
  assert(!legacy.allows_mit(0x7f, false, now + 1s, 1s));
  changed.source_stamp++;
  legacy.observe(changed, now + 1s);
  assert(!legacy.allows_mit(0x7f, false, now + 1s, 1s));
  changed.boot_id++;
  changed.source_stamp = 1;  // Source clock may restart with a new boot.
  assert(legacy.observe(changed, now + 2s));
  assert(legacy.allows_mit(0x7f, true, now + 2s, 1s));
  assert(!legacy.allows_mit(0, true, now + 2s, 1s));
  assert(!legacy.allows_mit(0x80, true, now + 2s, 1s));

  MotorModeGate stop;
  assert(stop.confirms_disable(10, 12, now, 1s));
  status.mode = 3;
  status.last_command_sequence = 9;
  stop.observe(status, now);
  assert(!stop.confirms_disable(10, 12, now, 1s));

  MotorModeGate retired;
  status.control_modes.fill(1);
  retired.observe(status, now);
  changed = status; changed.boot_id++; changed.source_stamp = 1;
  changed.control_modes[0] = 2;
  retired.observe(changed, now);
  status.source_stamp++;
  assert(!retired.observe(status, now + 500ms));
  assert(!retired.allows_mit(0x7f, true, now + 500ms, 1s));

  MotorModeGate many_boots;
  status.control_modes.fill(1); status.boot_id = 100; status.source_stamp = 1;
  many_boots.observe(status, now);
  for (unsigned index = 1; index <= 16; ++index) {
    status.boot_id++;
    assert(many_boots.observe(status, now));
    assert(many_boots.allows_mit(0x7f, true, now, 1s));
  }
  status.boot_id++;
  assert(many_boots.observe(status, now));
  assert(!many_boots.allows_mit(0x7f, true, now, 1s));
  status.boot_id--; status.source_stamp++;
  assert(!many_boots.observe(status, now));
  assert(!many_boots.allows_mit(0x7f, true, now, 1s));
  status.last_command_sequence = 11; status.source_stamp++;
  stop.observe(status, now);
  assert(!stop.confirms_disable(10, 12, now, 1s));
  status.disabled_verified = true; status.source_stamp++;
  stop.observe(status, now);
  assert(stop.confirms_disable(10, 12, now, 1s));
  assert(!stop.confirms_disable(10, 12, now + 1s, 1s));
  status.enabled_joint_mask = 1; status.source_stamp++;
  stop.observe(status, now);
  assert(!stop.confirms_disable(10, 12, now, 1s));
  status.enabled_joint_mask = 0; status.mode = 0; status.source_stamp++;
  stop.observe(status, now);
  assert(!stop.confirms_disable(10, 12, now, 1s));
}
