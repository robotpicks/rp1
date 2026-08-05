#!/usr/bin/env python3
"""Bench tool: talk directly to the real VESCs over the physical can0 bus, bypassing
ros2_control/diff_drive_controller entirely, for docs/wiring.md's "Bring-up order" steps 2-3
("One VESC + one motor on the real can0 bus, confirm a synthetic RawCommand ... spins it
correctly. Repeat for all 4 wheels.") -- before trusting the full ROS2 pipeline with all 4 at
once.

Two modes:

  listen   Passive: reports which esc_index values are broadcasting esc.Status, and their
           rpm/voltage/current/temperature. Sends nothing -- always safe, run this first to
           confirm each VESC is even present/configured (VESC+UAVCAN mode, see
           docs/can_id_map.md) before trying to command anything.
  pulse    Sends a real esc.RawCommand duty-cycle pulse to ONE esc_index and nothing else, then
           always sends a zero command back on exit (normal, Ctrl-C, or error). WHEELS MUST BE
           OFF THE GROUND AND THE E-STOP WITHIN REACH -- this moves a real motor. Prompts for
           an explicit 'yes' before doing anything.

Addressing note: esc.RawCommand/RPMCommand are broadcast and keyed by esc_index (position in
the `cmd`/`rpm` array), not addressed to a DroneCAN node ID -- see docs/can_id_map.md. That
matters here because as of this writing the 4 VESCs' actual node IDs are still unassigned
("TBD" in can_id_map.md); esc_index-based addressing means this tool doesn't need them either
(`listen` reports esc_index for the same reason -- it's the identifier that's actually
meaningful for cross-checking against docs/can_id_map.md's wheel index table).

DroneCAN plumbing mirrors simulation/sim_vesc_node.py's two workarounds for this sandbox's
dronecan + python-can pair (see that file's docstring / CLAUDE.md for the underlying bug
reports): force the native SocketCAN driver (python-can's path silently drops broadcast()
sends and node.spin() hangs), and only ever call node.spin(timeout=0) (any nonzero timeout gets
multiplied by 1000 into seconds by dronecan's receive() and hangs for ~100x longer than asked).
"""

import argparse
import sys
import time


def _force_native_socketcan_driver() -> None:
    import dronecan.driver  # lazy: see main()'s import
    dronecan.driver.PythonCAN = None


def cmd_listen(dronecan, node, seconds: float) -> bool:
    seen = {}

    def on_status(event):
        s = event.message
        seen[s.esc_index] = (s.rpm, s.voltage, s.current, s.temperature)

    node.add_handler(dronecan.uavcan.equipment.esc.Status, on_status)
    print(f"Listening for esc.Status on the bus for {seconds:.0f}s...")
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            node.spin(timeout=0)
        except Exception as exc:  # noqa: BLE001 - a bad received frame must not kill this tool
            print(f"...spin error, ignoring: {exc}")
        time.sleep(0.01)

    if not seen:
        print("No esc.Status seen -- no VESC is broadcasting. Check power, CAN wiring/"
              "termination, and that each VESC is actually in VESC+UAVCAN mode (see "
              "docs/can_id_map.md's VESC UAVCAN configuration section).")
        return False
    print(f"{len(seen)} distinct esc_index seen (cross-check against docs/can_id_map.md's "
          "wheel index table):")
    for idx in sorted(seen):
        rpm, volt, cur, temp = seen[idx]
        print(f"  esc_index={idx}: rpm={rpm} voltage={volt:.1f}V current={cur:.1f}A "
              f"temp={temp:.1f}K")
    return True


def cmd_pulse(dronecan, node, esc_index: int, duty: float, seconds: float) -> None:
    raw_val = max(-8191, min(8191, int(round(duty * 8192))))
    # Only elements up to esc_index are included -- a VESC only reacts if ITS esc_index has an
    # entry in the array at all, so this is how a single esc_index is targeted without touching
    # the others (see docs/can_id_map.md's RawCommand duty-cycle formula).
    cmd = [0] * esc_index + [raw_val]
    print(f"Pulsing esc_index={esc_index} at duty={duty:+.2%} (raw={raw_val}) for "
          f"{seconds:.1f}s. Ctrl-C stops early -- a zero command is always sent on exit either "
          "way.")
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=cmd))
            try:
                node.spin(timeout=0)
            except Exception as exc:  # noqa: BLE001 - must not skip the finally's stop command
                print(f"...spin error, ignoring: {exc}")
            time.sleep(0.02)
    finally:
        stop_cmd = [0] * (esc_index + 1)
        for _ in range(5):  # repeat a few times -- RawCommand isn't acked, so don't rely on one
            node.broadcast(dronecan.uavcan.equipment.esc.RawCommand(cmd=stop_cmd))
            time.sleep(0.02)
        print("Stop command sent.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--iface", default="can0")
    parser.add_argument("--node-id", type=int, default=99,
                         help="This tool's own DroneCAN node ID -- distinct from the real "
                         "vesc_dronecan_driver's (42, see docs/can_id_map.md) so it can safely "
                         "coexist if something else is also on the bus.")
    sub = parser.add_subparsers(dest="mode", required=True)

    p_listen = sub.add_parser("listen", help="Passive, always safe: report which esc_index "
                               "values are broadcasting esc.Status.")
    p_listen.add_argument("--seconds", type=float, default=5.0)

    p_pulse = sub.add_parser("pulse", help="Send a real RawCommand duty-cycle pulse to ONE "
                              "esc_index. WHEELS MUST BE OFF THE GROUND.")
    p_pulse.add_argument("esc_index", type=int, choices=(0, 1, 2, 3))
    p_pulse.add_argument("--duty", type=float, default=0.05,
                          help="Fraction of full duty cycle, -1.0..1.0. Default 0.05 (5%%) -- "
                          "start small and only increase once a wheel is confirmed spinning "
                          "the right direction at low duty.")
    p_pulse.add_argument("--seconds", type=float, default=1.0)

    args = parser.parse_args()

    try:
        import dronecan
    except ImportError:
        print("python module 'dronecan' not found. Install it with:\n\n"
              "  pip install --break-system-packages dronecan\n", file=sys.stderr)
        return 1

    _force_native_socketcan_driver()
    node = dronecan.make_node(args.iface, node_id=args.node_id, bitrate=1000000)
    try:
        if args.mode == "listen":
            return 0 if cmd_listen(dronecan, node, args.seconds) else 1
        else:
            print("*** WHEELS MUST BE OFF THE GROUND AND THE E-STOP WITHIN REACH ***")
            reply = input(f"About to pulse esc_index={args.esc_index} at duty="
                           f"{args.duty:+.2%}. Type 'yes' to continue: ")
            if reply.strip().lower() != "yes":
                print("Aborted, nothing sent.")
                return 1
            cmd_pulse(dronecan, node, args.esc_index, args.duty, args.seconds)
            return 0
    finally:
        node.close()


if __name__ == "__main__":
    sys.exit(main())
