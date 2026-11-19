# V1 JointCommand is frozen. This message only commands confirmed POS_VEL axes.
std_msgs/Header header
uint8 MODE_HOLD=0
uint8 MODE_TRACK=1
uint8 CONTROL_POS_VEL=2
uint64 boot_id
uint32 control_epoch
uint64 source_id
uint32 lease_id
uint32 sequence
# Only HOLD and TRACK are valid. Global DISABLE always uses V1 mode=3.
uint8 mode
# A declaration checked against register-readback state, never a switch request.
uint8 control_mode
uint32 active_joint_mask
# Fixed MCU channel order; unselected channels must be zero and stay disabled.
float32[7] position
float32[7] velocity_limit
