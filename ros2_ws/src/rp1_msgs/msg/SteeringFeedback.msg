# Per-wheel steering telemetry, published as each actuator.Status frame arrives over DroneCAN
# (or from a simulator standing in for it).

std_msgs/Header header

uint8 WHEEL_FRONT_LEFT=0
uint8 WHEEL_FRONT_RIGHT=1
uint8 WHEEL_REAR_LEFT=2
uint8 WHEEL_REAR_RIGHT=3

uint8 wheel_index
float32 angle             # radians
float32 angular_velocity  # radians/sec
