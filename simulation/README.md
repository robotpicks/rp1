# Bonus: DroneCAN-level simulation (no hardware, but real DroneCAN wire traffic)

`rp1_sim` (the ROS2 package) is the primary way to check the ROS2 architecture with zero
hardware -- see the top-level README and `docs/wiring.md`. This directory is a step below that:
it lets you run the **real** `rp1_dronecan_bridge` node (actual DroneCAN framing over a CAN
bus) against a software stand-in for the 4 VESCs, using a Linux virtual CAN interface instead
of a physical adapter.

## One-time setup: `vcan0`

Requires root (not automatable from a sandboxed session -- run these yourself):
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

## Run

```bash
python3 simulation/sim_vesc_node.py --iface vcan0        # pretends to be the 4 VESCs

source /opt/ros/kilted/setup.bash
source ros2_ws/install/setup.bash
ros2 run rp1_dronecan_bridge bridge_node --ros-args -p can_iface:=vcan0 -p require_can:=true
```

Then drive it like real hardware, e.g. publish to `/wheel_cmd` directly, or run
`rp1_teleop`/`rp1_control` on top and feed it through `/cmd_vel`/`/joy`. `/wheel_feedback`
should start showing plausible rpm/voltage/current once `sim_vesc_node.py` receives commands.

## Dependencies

`dronecan` and `python-can` (see `ros2_ws/src/rp1_dronecan_bridge/requirements.txt`) --
`sim_vesc_node.py` has no ROS2 dependency, only these.
