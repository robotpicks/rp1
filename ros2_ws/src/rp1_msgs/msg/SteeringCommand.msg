# Per-wheel steering angle command for the swerve extension (see docs/can_id_map.md's
# steering actuator_id = wheel index + 4 convention -- that offset is a DroneCAN addressing
# detail internal to vesc_dronecan_driver/firmware, not used at this ROS2 topic level).
#
# `angle` is in radians. The steering joints are continuous-rotation (not bounded to a fixed
# range) -- see CLAUDE.md.

std_msgs/Header header

uint8 WHEEL_FRONT_LEFT=0
uint8 WHEEL_FRONT_RIGHT=1
uint8 WHEEL_REAR_LEFT=2
uint8 WHEEL_REAR_RIGHT=3

float32[4] angle
