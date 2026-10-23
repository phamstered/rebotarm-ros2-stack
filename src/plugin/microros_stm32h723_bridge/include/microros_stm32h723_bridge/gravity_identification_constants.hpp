// SPDX-License-Identifier: MIT
// Frozen constants for /StartGravityID (GravityIdentification action).
// Prefer the generated action constants when available; this header mirrors
// docs/gravity-identification.md for C++ callers that do not include the
// generated typesupport yet.
#pragma once

#include <cstdint>

namespace microros_stm32h723_bridge {
namespace gravity_identification {

// Goal.mode
constexpr uint8_t MODE_FULL_ARM_GRAVITY = 0;
constexpr uint8_t MODE_PAYLOAD_SUBMODE = 1;

// When Goal.joint_mask == 0, treat as full arm (bits 0..5).
constexpr uint32_t DEFAULT_JOINT_MASK = 0x3Fu;

// Feedback.phase — docs/gravity-identification.md §4.1
constexpr uint8_t PHASE_IDLE = 0;
constexpr uint8_t PHASE_ARMING = 1;
constexpr uint8_t PHASE_ARMED = 2;
constexpr uint8_t PHASE_MOVING = 3;
constexpr uint8_t PHASE_HOLD_SETTLE = 4;
constexpr uint8_t PHASE_CAPTURING = 5;
constexpr uint8_t PHASE_FITTING = 6;
constexpr uint8_t PHASE_WRITING = 7;
constexpr uint8_t PHASE_ABORTING = 8;
constexpr uint8_t PHASE_DONE = 9;
constexpr uint8_t PHASE_FAILED = 10;

// Action server name (relative; resolve under node namespace as needed).
constexpr const char * ACTION_SERVER_NAME = "/StartGravityID";

}  // namespace gravity_identification
}  // namespace microros_stm32h723_bridge
