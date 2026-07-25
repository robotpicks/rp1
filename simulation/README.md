# Bonus: DroneCAN-level simulation (no hardware, but real DroneCAN wire traffic)

`rp1_sim` (the ROS2 package) is the primary way to check the ROS2 architecture with zero
hardware -- see the top-level README and `docs/wiring.md`. This directory is a step below that:
it lets you run **real** ROS2/DroneCAN nodes (`rp1_dronecan_bridge` for the 4 drive wheels,
`rp1_hardware_interface` for the steering actuators -- actual DroneCAN framing over a CAN bus)
against software stand-ins for the VESCs, using a Linux virtual CAN interface instead of a
physical adapter.

## One-time setup: `vcan0`

Requires root (not automatable from a sandboxed session -- run these yourself):
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## Run: drive wheels (`rp1_dronecan_bridge`)

The whole thing is wrapped up as one command from the meta repo, which starts both sides, asserts
the round trip, and tears down — this is what CI runs:

```bash
./robotpicks.sh smoke dronecan       # DRONECAN_IFACE=vcan0 by default
```

To drive it by hand instead:

```bash
python3 simulation/sim_vesc_node.py --iface vcan0        # pretends to be the 4 VESCs

source /opt/ros/$ROS_DISTRO/setup.bash
source ros2_ws/install/setup.bash
ros2 run rp1_dronecan_bridge bridge_node --ros-args -p can_iface:=vcan0 -p require_can:=true
```

Then drive it like real hardware, e.g. publish to `/wheel_cmd` directly, or run
`rp1_teleop`/`rp1_control` on top and feed it through `/cmd_vel`/`/joy`. `/wheel_feedback`
should start showing plausible rpm/voltage/current once `sim_vesc_node.py` receives commands.

`check_dronecan_loopback.py` is the assertion half, usable on its own against an already-running
pair. It publishes a known `WheelCommand` and checks the rpm that comes back, then stops
publishing and checks the bridge's command-timeout failsafe returns the wheels to zero — so it
fails if telemetry never arrives, arrives at the wrong scale, or latches the last command.

## Run: steering actuators (`rp1_hardware_interface`)

```bash
python3 simulation/sim_actuator_node.py --iface vcan0    # pretends to be the steering VESCs
                                                           # (default actuator_id 5,6; pass
                                                           # --actuator-ids 4,5,6,7 for all 4)

source /opt/ros/kilted/setup.bash
source ros2_ws/install/setup.bash
# rp1_hardware_interface talks SocketCAN directly (not through rp1_dronecan_bridge) -- point
# its hardware "can_iface" param at vcan0, e.g. via steering_hardware.launch.py, or edit
# rp1_steering.urdf's <param name="can_iface"> to vcan0 for this test.
ros2 launch rp1_hardware_interface steering_hardware.launch.py
```

`sim_vesc_node.py` and `sim_actuator_node.py` can run simultaneously on the same `vcan0` (use
distinct `--node-id` values, they default to 10 and 11 respectively) to exercise the full
4-wheel + 4-steering setup at once.

## Dependencies

`dronecan` and `python-can` (see `ros2_ws/src/rp1_dronecan_bridge/requirements.txt`) -- neither
script has a ROS2 dependency, only these.
