#!/usr/bin/env python3
"""Standalone DroneCAN simulator: pretends to be the 4 VESC UAVCAN ESC nodes on a CAN bus.

No ROS2 dependency -- the real VESCs don't have one either, they just speak DroneCAN on the
wire. Point this at a virtual CAN interface (see README.md in this directory for `vcan0`
setup) and it'll consume `uavcan.equipment.esc.RawCommand` and reply with
`uavcan.equipment.esc.Status`, so `rp1_dronecan_bridge` can be run for real -- actual DroneCAN
framing, not its `require_can:=false` dry-run mode -- with zero physical hardware.

Wheel dynamics here are a simple rate-limited approach to a target rpm, not a physical motor
model -- good enough to see plausible-looking, moving telemetry.
"""

import argparse
import time

import dronecan

NUM_WHEELS = 4
MAX_RPM = 3000.0
RPM_SLEW_PER_SEC = 4000.0
NOMINAL_VOLTAGE = 24.0
ROOM_TEMPERATURE_KELVIN = 298.15  # DroneCAN temperature fields are Kelvin


def _patch_python_can_flush_tx_buffer() -> None:
    """dronecan's python-can driver calls bus.flush_tx_buffer() after every send, but
    python-can >=4's BusABC no longer implements it for any interface (raises
    NotImplementedError), which otherwise kills the writer thread on the first broadcast.
    """
    import can
    can.bus.BusABC.flush_tx_buffer = lambda self: None


class SimulatedWheel:

    def __init__(self):
        self.target_rpm = 0.0
        self.rpm = 0.0

    def set_command(self, raw_cmd: int) -> None:
        self.target_rpm = (raw_cmd / 8191.0) * MAX_RPM

    def step(self, dt: float) -> None:
        delta = self.target_rpm - self.rpm
        max_step = RPM_SLEW_PER_SEC * dt
        if abs(delta) <= max_step:
            self.rpm = self.target_rpm
        else:
            self.rpm += max_step if delta > 0 else -max_step


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--iface', default='vcan0', help='SocketCAN interface (default: vcan0)')
    parser.add_argument('--node-id', type=int, default=10, help='this simulator\'s DroneCAN node ID')
    parser.add_argument('--status-rate-hz', type=float, default=10.0)
    args = parser.parse_args()

    wheels = [SimulatedWheel() for _ in range(NUM_WHEELS)]
    _patch_python_can_flush_tx_buffer()
    # bitrate is required by this dronecan version's python-can driver even for SocketCAN,
    # which actually ignores it (the interface's real bitrate is set via `ip link`).
    node = dronecan.make_node(args.iface, node_id=args.node_id, bitrate=1000000)

    def on_raw_command(event):
        for esc_index, raw in enumerate(event.message.cmd):
            if esc_index < NUM_WHEELS:
                wheels[esc_index].set_command(raw)

    node.add_handler(dronecan.uavcan.equipment.esc.RawCommand, on_raw_command)

    last_step_time = [time.monotonic()]

    def broadcast_status():
        now = time.monotonic()
        dt = now - last_step_time[0]
        last_step_time[0] = now
        for esc_index, wheel in enumerate(wheels):
            wheel.step(dt)
            node.broadcast(dronecan.uavcan.equipment.esc.Status(
                esc_index=esc_index,
                rpm=int(wheel.rpm),
                voltage=NOMINAL_VOLTAGE,
                current=abs(wheel.rpm) / MAX_RPM * 10.0,
                temperature=ROOM_TEMPERATURE_KELVIN,
            ))

    # Deliberately not using node.periodic(): verified experimentally that its callback never
    # actually fires with this dronecan version (sched.scheduler.run(blocking=False) interaction
    # issue) -- manual elapsed-time tracking in the main loop instead, same pattern already used
    # in rp1_dronecan_bridge/bridge_node.py and the bench test scripts this session.
    status_period = 1.0 / args.status_rate_hz
    next_status_time = time.monotonic() + status_period

    print('sim_vesc_node: simulating %d VESCs on %s (node_id=%d)'
          % (NUM_WHEELS, args.iface, args.node_id))
    try:
        while True:
            try:
                # timeout=0 deliberately: this dronecan/python-can version pair multiplies any
                # nonzero timeout by 1000 before handing it to python-can's recv() (which wants
                # seconds, not ms), so e.g. timeout=0.1 would block for ~100s instead of 100ms.
                node.spin(timeout=0)
            except Exception as exc:  # noqa: BLE001 - a bad received frame must not kill the sim
                print('sim_vesc_node: spin error, ignoring: %s' % exc)

            now = time.monotonic()
            if now >= next_status_time:
                next_status_time = now + status_period
                broadcast_status()

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
