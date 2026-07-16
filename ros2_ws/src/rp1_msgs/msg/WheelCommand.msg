# Per-wheel drive command for the 4-wheel MVP base (skid-steer, no steering joints yet).
# `velocity` is a normalized setpoint in [-1.0, 1.0] mapped by rp1_dronecan_bridge onto the
# DroneCAN esc.RawCommand range; the exact physical meaning (duty vs current vs rpm) is decided
# by the bridge firmware and documented in docs/can_id_map.md.

std_msgs/Header header

uint8 WHEEL_FRONT_LEFT=0
uint8 WHEEL_FRONT_RIGHT=1
uint8 WHEEL_REAR_LEFT=2
uint8 WHEEL_REAR_RIGHT=3

float32[4] velocity
