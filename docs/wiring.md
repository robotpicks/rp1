# Wiring notes (MVP: 4 drive wheels, no steering joints)

## Bus

One CAN bus, PC directly to the 4 VESCs -- no bridge MCU (VESC's UAVCAN CAN mode speaks
DroneCAN natively, see `docs/can_id_map.md`). PC side needs a SocketCAN-capable USB-CAN adapter
(e.g. CANable/candleLight running `gs_usb`, or an Innomaker USB2CAN) so it shows up as a plain
Linux `can0` interface -- no custom driver needed. Bring the interface up before launching:
```bash
sudo ip link set can0 type can bitrate 1000000   # match the VESCs' configured UAVCAN bitrate
sudo ip link set can0 up
```
Terminate the bus with 120 ohm resistors at each end per standard CAN wiring practice.

## VESC configuration (one-off, not part of the runtime path)

Each VESC is configured directly over its own USB link using VESC Tool: CAN mode = UAVCAN,
DroneCAN node ID + `esc_index` (see `docs/can_id_map.md`), motor/FOC parameters, and
current/duty limits. This is a bench/setup-time activity -- the runtime control path never
talks to VESC Tool.

`vesc_tool` also has a headless CLI mode, useful for scripting this instead of using the GUI:
```bash
vesc_tool --vescPort /dev/ttyACM0 --getAppConf app_conf.xml   # read current app config
vesc_tool --vescPort /dev/ttyACM0 --setAppConf app_conf.xml   # write it back after editing
```
The CAN mode/node ID/esc_index fields are part of the app configuration XML.

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

## Testing without hardware

Two levels, from least to most CAN-realistic:

1. `ros2 launch rp1_bringup rp1_mvp_sim.launch.py` -- swaps `rp1_dronecan_bridge` for `rp1_sim`,
   a pure-ROS2 node with no CAN involved at all. Use this to check the ROS2 graph itself
   (teleop -> control -> odometry/TF) with `ros2 topic echo` or rviz2.
2. `simulation/` (bonus) -- a virtual CAN interface (`vcan0`) plus a script that pretends to be
   the 4 VESCs, so the real `rp1_dronecan_bridge` can be exercised (DroneCAN wire format and
   all) with zero physical hardware. See `simulation/README.md`.

## Bring-up order

1. Configure each VESC's UAVCAN settings on the bench (one at a time, confirm with VESC Tool
   before wiring it into the shared bus) and fill in `docs/can_id_map.md`.
2. One VESC + one motor on the real `can0` bus, confirm a synthetic RawCommand (or the real
   ROS2 pipeline) spins it correctly.
3. Repeat for all 4 wheels.
4. Bring up the full ROS2 pipeline (`ros2 launch rp1_bringup rp1_mvp.launch.py`), wheels off
   the ground first, with the deadman button held.
5. On-ground drive test, low speed limits first (`rp1_control`'s `max_wheel_speed` param).
