# Per-wheel telemetry, published as each esc.Status frame arrives from the bridge over DroneCAN.

std_msgs/Header header

uint8 WHEEL_FRONT_LEFT=0
uint8 WHEEL_FRONT_RIGHT=1
uint8 WHEEL_REAR_LEFT=2
uint8 WHEEL_REAR_RIGHT=3

uint8 wheel_index
float32 rpm
float32 voltage
float32 current
