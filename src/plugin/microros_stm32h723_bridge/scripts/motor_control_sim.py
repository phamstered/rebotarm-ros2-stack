"""Deterministic, hardware-free V2 protocol peer for isolated integration tests.

The simulated limits and mode readbacks are not device qualification data.
"""

import math
import struct

NONE, INVALID, STALE_IDENTITY, NOT_DISABLED, BUSY = range(5)
UNSUPPORTED, MODE, LEASE, SEQUENCE, NONFINITE, LIMITS = range(5, 11)
FAULT, READBACK, TIMEOUT = range(11, 14)


def newer(sequence, previous):
    return 0 < ((sequence - previous) & 0xffffffff) < 0x80000000


def finite_float32(value):
    try:
        return math.isfinite(value) and math.isfinite(struct.unpack('f', struct.pack('f', value))[0])
    except (TypeError, ValueError, OverflowError):
        return False


class MotorControlSim:
    """All time comes from the caller; no real CAN, timers, or ROS in this model."""

    def __init__(self, boot_id=1, supported_mask=0x3f):
        self.boot_id = boot_id
        self.control_epoch = 1
        self.owner_source_id = 0
        self.lease_id = 0
        self.selected_joint_mask = 0
        self.enabled_joint_mask = 0
        self.disabled_verified = False
        self.verified_joint_mask = 0x7f
        self.pos_vel_supported_mask = supported_mask
        self.control_modes = [1] * 7
        self.velocity_limit_max = [1.0] * 7
        self.position_min = [-3.0] * 7
        self.position_max = [3.0] * 7
        self.switching = False
        self.operation_id = 0
        self.operation_state = 0
        self.last_command_sequence = 0
        self.mode = 3
        self.fault_code = 0
        self.error_code = 0
        self.rejected_sequence = 0
        self.reject_code = 0
        self.position = [0.0] * 7
        self.target = [0.0] * 7
        self.speed = [0.0] * 7
        self._next_lease = 0
        self._last_motion_at = None
        self._lease_at = None
        self._pending = None
        self._last_tick = None
        self._operations = {}
        self._disable_received_at = None
        self._disable_sequence = None
        self._disabled_feedback_at = None
        self._feedback_at = None
        self.readback_failure_mask = 0
        self.feedback_enabled = True

    def status(self):
        names = ('boot_id control_epoch owner_source_id lease_id selected_joint_mask '
                 'enabled_joint_mask verified_joint_mask pos_vel_supported_mask control_modes '
                 'velocity_limit_max position_min position_max switching operation_id '
                 'operation_state last_command_sequence mode fault_code error_code '
                 'rejected_sequence reject_code disabled_verified').split()
        result = {name: getattr(self, name) for name in names}
        result['protocol_version'] = 2
        return result

    def _reply(self, operation_id, error=NONE):
        return dict(accepted=error == NONE, error_code=error,
                    control_epoch=self.control_epoch, lease_id=self.lease_id,
                    operation_id=operation_id)

    def service(self, request, now):
        op = request['operation_id']
        source = request['source_id']
        key = (source, op)
        digest = tuple(sorted(request.items()))
        if key in self._operations:
            old_digest, reply = self._operations[key]
            return dict(reply) if old_digest == digest else self._reply(op, INVALID)
        error = self._service(request, now)
        reply = self._reply(op, error)
        if op and source:
            self._operations[key] = (digest, dict(reply))
            if len(self._operations) > 16:
                del self._operations[next(iter(self._operations))]
        return reply

    def _service(self, request, now):
        action, op, source = (request[key] for key in ('action', 'operation_id', 'source_id'))
        mask, drive = request['active_joint_mask'], request['control_mode']
        if action not in (1, 2, 3, 4) or not op or not source:
            return INVALID
        if request['boot_id'] != self.boot_id or request['expected_control_epoch'] != self.control_epoch:
            return STALE_IDENTITY
        if action != 1 and self.owner_source_id not in (0, source):
            return BUSY
        if action == 4:
            self._disable()
            self.operation_id, self.operation_state, self.error_code = op, 4, NONE
            return NONE
        if (self.mode != 3 or self.enabled_joint_mask or not self.disabled_verified or
                self._disable_received_at is None or self._disabled_feedback_at is None or
                self._disabled_feedback_at <= self._disable_received_at or
                now - self._disabled_feedback_at > 0.1 or
                request['disable_sequence'] != self.last_command_sequence or
                self._disable_sequence != self.last_command_sequence):
            return NOT_DISABLED
        if self.switching:
            return BUSY
        if action == 1:
            if self.lease_id:
                return BUSY
            if drive not in (1, 2) or mask == 0 or mask & ~0x7f:
                return INVALID
            if drive == 2 and (mask & self.pos_vel_supported_mask) != mask:
                return UNSUPPORTED
            if self.fault_code:
                return FAULT
            self.owner_source_id = source
            self.selected_joint_mask = mask
            self.switching = True
            self.operation_id, self.operation_state, self.error_code = op, 1, NONE
            self._pending = (now + 0.05, mask, drive)
        elif action == 2:
            if self.fault_code:
                return FAULT
            if self.owner_source_id != source or self.lease_id:
                return LEASE
            if mask == 0 or mask != self.selected_joint_mask or drive != 2:
                return INVALID
            if mask & ~self.verified_joint_mask or any(
                    mask & (1 << index) and mode != 2 for index, mode in enumerate(self.control_modes)):
                return MODE
            self._next_lease = (self._next_lease + 1) & 0xffffffff or 1
            self.lease_id = self._next_lease
            self._lease_at = now
            self.operation_id, self.operation_state, self.error_code = op, 2, NONE
        elif action == 3:
            if self.lease_id or any(mode != 1 for mode in self.control_modes) or self.verified_joint_mask != 0x7f:
                return MODE
            self.owner_source_id = 0
            self.selected_joint_mask = 0
            self.operation_id, self.operation_state, self.error_code = op, 2, NONE
        return NONE

    def _disable(self):
        self.mode = 3
        self.disabled_verified = False
        self._disable_sequence = None
        self._disable_received_at = None
        self.enabled_joint_mask = 0
        self.lease_id = 0
        self._last_motion_at = self._lease_at = None
        self.speed = [0.0] * 7
        if self._pending is not None:
            self.operation_state = 4
        self._pending = None
        self.switching = False

    def _reject(self, sequence, code):
        self.rejected_sequence, self.reject_code = sequence, code
        return code

    def command_v1(self, command, now):
        sequence, mode = command['sequence'], command['mode']
        if not newer(sequence, self.last_command_sequence):
            return self._reject(sequence, SEQUENCE)
        if mode == 3:
            self.last_command_sequence = sequence
            self._disable()  # Idempotent output-off does not clear a latched fault.
            self._disable_received_at = now
            self._disable_sequence = sequence
            return NONE
        if self.switching or self.owner_source_id or self.lease_id:
            return self._reject(sequence, BUSY)
        if any(drive != 1 for drive in self.control_modes) or self.verified_joint_mask != 0x7f:
            return self._reject(sequence, MODE)
        if self.fault_code:
            return self._reject(sequence, FAULT)
        if mode not in (0, 1, 2):
            return self._reject(sequence, INVALID)
        arrays = [command.get(key, [0.0] * 7) for key in ('position', 'velocity', 'kp', 'kd', 'effort')]
        if any(len(values) != 7 for values in arrays):
            return self._reject(sequence, INVALID)
        if not all(finite_float32(value) for values in arrays for value in values):
            return self._reject(sequence, NONFINITE)
        self.target = self.position.copy() if mode == 0 else list(arrays[0])
        self.speed = [1.0] * 7
        self.mode, self.last_command_sequence = mode, sequence
        self.disabled_verified = False
        self.enabled_joint_mask = 0x7f
        self._last_motion_at = now
        self.reject_code = NONE
        return NONE

    def command_v2(self, command, now):
        sequence = command['sequence']
        if command['boot_id'] != self.boot_id or command['control_epoch'] != self.control_epoch:
            return self._reject(sequence, STALE_IDENTITY)
        if self.switching:
            return self._reject(sequence, BUSY)
        if self.fault_code:
            return self._reject(sequence, FAULT)
        if not self.lease_id or command['lease_id'] != self.lease_id or command['source_id'] != self.owner_source_id:
            return self._reject(sequence, LEASE)
        mask = command['active_joint_mask']
        if mask == 0 or mask != self.selected_joint_mask:
            return self._reject(sequence, INVALID)
        if command['control_mode'] != 2 or any(
                mask & (1 << index) and self.control_modes[index] != 2 for index in range(7)):
            return self._reject(sequence, MODE)
        if mask & ~self.verified_joint_mask:
            return self._reject(sequence, MODE)
        if command['mode'] not in (0, 1):
            return self._reject(sequence, MODE)
        if not newer(sequence, self.last_command_sequence):
            return self._reject(sequence, SEQUENCE)
        position, speed = command['position'], command['velocity_limit']
        if len(position) != 7 or len(speed) != 7:
            return self._reject(sequence, INVALID)
        if not all(finite_float32(value) for value in list(position) + list(speed)):
            return self._reject(sequence, NONFINITE)
        hold = command['mode'] == 0
        if hold and any(value != 0 for value in position):
            return self._reject(sequence, INVALID)
        if hold and (self._feedback_at is None or now - self._feedback_at > 0.1):
            return self._reject(sequence, FAULT)
        for index in range(7):
            if mask & (1 << index):
                target = self.position[index] if hold else position[index]
                if (not finite_float32(target) or
                        not self.position_min[index] <= target <= self.position_max[index] or
                        not 0 < speed[index] <= self.velocity_limit_max[index]):
                    return self._reject(sequence, LIMITS)
            elif position[index] != 0 or speed[index] != 0:
                return self._reject(sequence, INVALID)
        self.target = self.position.copy() if command['mode'] == 0 else list(position)
        self.speed = list(speed)
        self.mode, self.last_command_sequence = command['mode'], sequence
        self.disabled_verified = False
        self.enabled_joint_mask = mask
        self._last_motion_at = now
        self.reject_code = NONE
        return NONE

    def tick(self, now):
        if self.feedback_enabled:
            self._feedback_at = now
        if self.feedback_enabled and self.mode == 3 and self.enabled_joint_mask == 0:
            self._disabled_feedback_at = now
        self.disabled_verified = bool(
            self.mode == 3 and self.enabled_joint_mask == 0 and
            self._disable_received_at is not None and self._disabled_feedback_at is not None and
            self._disabled_feedback_at > self._disable_received_at and
            now - self._disabled_feedback_at <= 0.1 and
            self._disable_sequence == self.last_command_sequence)
        if self._pending and now >= self._pending[0]:
            _, mask, drive = self._pending
            self._pending, self.switching = None, False
            if self.readback_failure_mask & mask:
                self.verified_joint_mask &= ~self.readback_failure_mask
                self.operation_state, self.error_code = 3, READBACK
            else:
                for index in range(7):
                    if mask & (1 << index):
                        self.control_modes[index] = drive
                self.verified_joint_mask |= mask
                self.control_epoch += 1
                self.operation_state, self.error_code = 2, NONE
        if self._last_motion_at is not None:
            age = now - self._last_motion_at
            if age > 0.1:
                self.lease_id = 0
            if self.mode == 1 and age > 0.1:
                self.mode = 0
                self.target = self.position.copy()
            if self.mode != 3 and age > 0.5:
                self.fault_code = 11
                self.mode = 0
                self.target = self.position.copy()
                self.lease_id = 0
        elif self.lease_id and self._lease_at is not None and now - self._lease_at > 0.1:
            self.lease_id = 0
        dt = 0.0 if self._last_tick is None else max(0.0, now - self._last_tick)
        if self.mode != 3:
            for index in range(7):
                if self.enabled_joint_mask & (1 << index):
                    error = self.target[index] - self.position[index]
                    step = self.speed[index] * dt
                    self.position[index] += max(-step, min(step, error))
        self._last_tick = now
