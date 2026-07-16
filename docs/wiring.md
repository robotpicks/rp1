# Wiring notes (MVP: 4 drive wheels, no steering joints)

## Buses

- **DroneCAN bus**: PC <-> bridge MCU. PC side needs a SocketCAN-capable USB-CAN adapter (e.g.
  CANable/candleLight running `gs_usb`, or an Innomaker USB2CAN) so it shows up as a plain
  Linux `can0` interface -- no custom driver needed. Bring the interface up before launching:
  ```bash
  sudo ip link set can0 type can bitrate 1000000   # match the bridge MCU's DroneCAN bitrate
  sudo ip link set can0 up
  ```
- **VESC-CAN bus**: bridge MCU <-> VESC #1-4, physically separate from the DroneCAN bus (the
  bridge MCU needs two independent CAN controllers). Terminate both buses with 120 ohm
  resistors at each end per standard CAN wiring practice.

## VESC configuration (one-off, not part of the runtime path)

Each VESC is configured directly over its own USB or CAN link using VESC Tool: set its CAN
controller ID (see `docs/can_id_map.md`), motor/FOC parameters, and current/duty limits. This
is a bench/setup-time activity -- the runtime control path never talks to VESC Tool.

## Controller

Xbox Series X controller, paired over USB or Bluetooth to the PC running ROS2. Verify it's
seen as a joystick device and check axis/button numbering before trusting the defaults in
`rp1_teleop/config/joy_xbox_series_x.yaml`:

```bash
ros2 run joy joy_node    # then, in another terminal:
ros2 topic echo /joy     # wiggle each stick / press each button, note the indices
# or, outside ROS2:
jstest /dev/input/js0
```

## Bring-up order (see the project plan for full detail)

1. Bench-test the bridge firmware alone (DroneCAN <-> VESC-CAN framing) before connecting
   ROS2 or motors.
2. One VESC + one motor on the bench, confirm a synthetic RawCommand spins it correctly.
3. Repeat for all 4 wheels, fill in `docs/can_id_map.md`.
4. Bring up the full ROS2 pipeline (`ros2 launch rp1_bringup rp1_mvp.launch.py`), wheels off
   the ground first, with the deadman button held.
5. On-ground drive test, low speed limits first (`rp1_control`'s `max_wheel_speed` param).
