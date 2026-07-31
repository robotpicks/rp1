#!/usr/bin/env python3
"""Standalone DroneCAN simulator: pretends to be N steering actuator VESC UAVCAN nodes on a
CAN bus, speaking uavcan.equipment.actuator.{ArrayCommand,Status} -- the steering counterpart
to sim_vesc_node.py (which simulates the 4 drive-wheel ESCs).

No ROS2 dependency -- the real VESCs don't have one either, they just speak DroneCAN on the
wire. Point this at a virtual CAN interface (see README.md in this directory for `vcan0` setup)
alongside sim_vesc_node.py and it lets vesc_dronecan_driver (ros2_ws/src/
vesc_dronecan_driver) be exercised for real -- actual DroneCAN framing, not just unit tests --
with zero physical hardware.

Actuator IDs default to 5 and 6 (the two currently bench-wired steering actuators, see
docs/can_id_map.md's actuator_id = drive wheel index + 4 convention); pass --actuator-ids to
simulate a different set, e.g. all 4 (4,5,6,7) for the full swerve case.

Steering dynamics here are a simple rate-limited slew toward the commanded angle (shortest
angular path), not a physical FOC/servo model -- same level of fidelity as sim_vesc_node.py's
rpm slew, good enough for plausible-looking, moving telemetry.
"""

import argparse
import math
import time

import dronecan

MAX_STEERING_RATE_RAD_S = 6.0
NOMINAL_VOLTAGE = 24.0


def _force_native_socketcan_driver() -> None:
    """dronecan.driver.make_driver() always prefers its python-can-backed PythonCAN driver over
    the native SocketCAN one whenever the `can` package is merely importable (see
    dronecan/driver/__init__.py's make_driver(): `elif PythonCAN is not None: return
    PythonCAN(...)` runs before the SocketCAN fallback, and PythonCAN is non-None as long as
    `import can` succeeded, regardless of which python-can version). Confirmed broken against
    python-can 4.6.1 -- not just the flush_tx_buffer() incompatibility this function used to
    patch around (python-can >=4's BusABC no longer implements it), but something deeper:
    node.broadcast() silently queues a frame that never reaches the bus at all, and node.spin()
    then hangs indefinitely rather than timing out. Forcing dronecan.driver.PythonCAN to None
    here makes make_driver() fall through to the native SocketCAN class instead (Linux-only, no
    python-can involved), which was confirmed to actually place frames on the bus and complete
    spin() normally. Must run before dronecan.make_node() below.
    """
    import dronecan.driver
    dronecan.driver.PythonCAN = None


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class SimulatedActuator:

    def __init__(self):
        self.target_angle = 0.0
        self.angle = 0.0
        self.angular_velocity = 0.0

    def set_command(self, target_angle: float) -> None:
        self.target_angle = target_angle

    def step(self, dt: float) -> None:
        max_step = MAX_STEERING_RATE_RAD_S * dt
        delta = _normalize_angle(self.target_angle - self.angle)
        step = delta if abs(delta) <= max_step else math.copysign(max_step, delta)
        self.angle = _normalize_angle(self.angle + step)
        self.angular_velocity = step / dt if dt > 0 else 0.0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iface', default='vcan0', help='SocketCAN interface (default: vcan0)')
    parser.add_argument('--node-id', type=int, default=11,
                         help='this simulator\'s DroneCAN node ID (default: 11, distinct from '
                              'sim_vesc_node.py\'s default of 10)')
    parser.add_argument('--actuator-ids', default='5,6',
                         help='comma-separated actuator_id values to simulate '
                              '(default: 5,6 -- the two currently bench-wired steering '
                              'actuators; use 4,5,6,7 for the full swerve case)')
    parser.add_argument('--status-rate-hz', type=float, default=10.0)
    args = parser.parse_args()

    actuator_ids = [int(x) for x in args.actuator_ids.split(',')]
    actuators = {actuator_id: SimulatedActuator() for actuator_id in actuator_ids}

    _force_native_socketcan_driver()
    # bitrate is accepted for API compatibility but ignored by the native SocketCAN driver (the
    # interface's real bitrate is set via `ip link`).
    node = dronecan.make_node(args.iface, node_id=args.node_id, bitrate=1000000)

    def on_array_command(event):
        for command in event.message.commands:
            actuator = actuators.get(command.actuator_id)
            if actuator is None:
                continue
            if command.command_type == command.COMMAND_TYPE_POSITION:
                actuator.set_command(command.command_value)

    node.add_handler(dronecan.uavcan.equipment.actuator.ArrayCommand, on_array_command)

    last_step_time = [time.monotonic()]

    def broadcast_status():
        now = time.monotonic()
        dt = now - last_step_time[0]
        last_step_time[0] = now
        for actuator_id, actuator in actuators.items():
            actuator.step(dt)
            node.broadcast(dronecan.uavcan.equipment.actuator.Status(
                actuator_id=actuator_id,
                position=actuator.angle,
                force=float('nan'),
                speed=actuator.angular_velocity,
                power_rating_pct=min(127, int(abs(actuator.angular_velocity) / MAX_STEERING_RATE_RAD_S * 100.0)),
            ))

    # Deliberately not using node.periodic(): verified experimentally that its callback never
    # actually fires with this dronecan version (sched.scheduler.run(blocking=False) interaction
    # issue) -- manual elapsed-time tracking in the main loop instead, same pattern already used
    # in rp1_dronecan_bridge/bridge_node.py and the bench test scripts this session.
    status_period = 1.0 / args.status_rate_hz
    next_status_time = time.monotonic() + status_period

    print('sim_actuator_node: simulating actuator_id(s) %s on %s (node_id=%d)'
          % (actuator_ids, args.iface, args.node_id))
    try:
        while True:
            try:
                # timeout=0 deliberately: this dronecan/python-can version pair multiplies any
                # nonzero timeout by 1000 before handing it to python-can's recv() (which wants
                # seconds, not ms), so e.g. timeout=0.1 would block for ~100s instead of 100ms.
                node.spin(timeout=0)
            except Exception as exc:  # noqa: BLE001 - a bad received frame must not kill the sim
                print('sim_actuator_node: spin error, ignoring: %s' % exc)

            now = time.monotonic()
            if now >= next_status_time:
                next_status_time = now + status_period
                broadcast_status()

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
