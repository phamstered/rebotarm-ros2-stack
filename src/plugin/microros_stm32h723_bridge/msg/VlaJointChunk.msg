# Joint position chunk from an external VLA / inference node.
# Published on /vla/joint_chunk (see docs/vla-integration.md).
# positions length must match the controller joints list (typically 6).
# Multi-horizon action chunks should be reduced to the active setpoint
# by the publisher; this message carries one command vector.
std_msgs/Header header
float32[] positions
# Optional per-message timeout hint (seconds). If <= 0, the controller
# uses its command_timeout_sec parameter.
float32 timeout_sec
