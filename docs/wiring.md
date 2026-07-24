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

Each VESC is configured directly over its own USB link using VESC Tool: CAN mode =
**VESC+UAVCAN** (not plain UAVCAN -- see `docs/can_id_map.md` for why), CAN ID (doubles as the
DroneCAN node ID in this mode) + `esc_index` (see `docs/can_id_map.md`), motor/FOC parameters,
and current/duty limits. This is a bench/setup-time activity -- the runtime control path never
talks to VESC Tool.

`vesc_tool` also has a headless CLI mode, useful for scripting this instead of using the GUI:
```bash
vesc_tool --vescPort /dev/ttyACM0 --getAppConf app_conf.xml   # read current app config
vesc_tool --vescPort /dev/ttyACM0 --setAppConf app_conf.xml   # write it back after editing
```
The CAN mode/node ID/esc_index fields are part of the app configuration XML.

## Controller

Xbox Series X controller. **Recommended default: wired USB-C, not Bluetooth.** Over Bluetooth
it pairs as BLE HID (`hid_microsoft` driver, shows up via `bluetoothctl` rather than `xpad`),
which on Ubuntu is prone to repeated connect/disconnect flapping (`joy_node` log alternating
"Opened joystick" / "Unable to open ...") even at good signal strength -- a known Linux BT/Xbox
issue, not a range problem. Wired USB avoids it entirely (and switches to the more stable
`xpad` driver), which matters here since this is a robot control input, not just a games
controller. If Bluetooth must be used, try `options bluetooth disable_ertm=1` in
`/etc/modprobe.d/` first (the most commonly effective fix for this specific symptom) and check
USB autosuspend isn't powering down the BT adapter (`/sys/bus/usb/devices/*/power/control`).

Verify it's seen as a joystick device and check axis/button numbering before trusting the
defaults in `rp1_teleop/config/joy_xbox_series_x.yaml`:

```bash
ros2 run joy joy_node    # then, in another terminal:
ros2 topic echo /joy     # wiggle each stick / press each button, note the indices
# or, outside ROS2:
jstest /dev/input/js0
```

## RC radio (ExpressLRS / CRSF) -- alternative to the Xbox pad

An ExpressLRS receiver can drive the robot instead of the Xbox controller. `rp1_elrs`'s
`elrs_node` reads the receiver's **CRSF** stream over a UART and republishes it as `/joy`, so the
same `rp1_teleop` mapping/deadman applies (see `rp1_elrs/config/joy_elrs.yaml`). It also pushes
battery telemetry (from `/wheel_feedback`) back over the link to the handset.

Wiring / setup:

- Put the receiver in **CRSF serial** mode (its default), *not* MAVLink mode (that's a separate,
  not-yet-built Phase-2 path). Default baud is **420000**.
- Connect the RX UART to the PC. Simplest is a 3V3 USB-UART adapter: adapter **RX <- RX TX pad**
  (channels), adapter **TX -> RX RX pad** (telemetry back to the handset), plus GND and 5V.
  On a Pi you can instead use a header UART (`/dev/ttyAMA0`) with the console disabled.
- Set `serial_port` (default `/dev/ttyUSB0`) in `rp1_elrs/config/rp1_elrs.yaml` (or the
  `elrs_node` override in `rp1_bringup/config/rp1_mvp.yaml`).
- CRSF is a half-duplex protocol; on a proper full-duplex two-wire adapter hookup telemetry
  writes are fine. On a true single-wire hookup, telemetry timing collisions are possible --
  `elrs_node` writes telemetry on a simple timer and does not yet arbitrate RX telemetry slots.

Verify the channel/switch mapping before trusting the defaults in `rp1_elrs/config/rp1_elrs.yaml`
(`axis_channels`, `deadman_channel`) -- they assume AETR channel order and a 2-position arm
switch, which is TX-dependent:

```bash
ros2 launch rp1_elrs elrs_teleop.launch.py    # elrs_node + rp1_teleop, no joy_node
# then watch /joy (small rclpy subscriber preferred; see CLAUDE.md on echo flakiness) and move
# each stick / flip the arm switch -- confirm axes move and button 4 toggles with the switch.
```

To exercise the parser without a receiver, make a virtual serial pair
(`socat -d -d pty,raw,echo=0 pty,raw,echo=0`), point `serial_port` at one pty, and write canned
`RC_CHANNELS_PACKED` bytes into the other. Or run `elrs_node` with `require_serial:=false` to
dry-run the telemetry side (frames are logged, no port opened) -- the analogue of the DroneCAN
bridge's `require_can:=false`.

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
