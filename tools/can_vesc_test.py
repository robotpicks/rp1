#!/usr/bin/env python3
"""Bench tool: talk directly to the real VESCs over the physical can0 bus, bypassing
ros2_control/diff_drive_controller entirely, for docs/wiring.md's "Bring-up order" steps 2-3
("One VESC + one motor on the real can0 bus, confirm a synthetic RawCommand ... spins it
correctly. Repeat for all 4 wheels.") -- before trusting the full ROS2 pipeline with all 4 at
once.

Three modes:

  listen   Passive: reports which esc_index values are broadcasting esc.Status, and their
           rpm/voltage/current/temperature. Sends nothing -- always safe, run this first to
           confirm each VESC is even present/configured (VESC+UAVCAN mode, see
           docs/can_id_map.md) before trying to command anything. Also issues a
           uavcan.protocol.GetNodeInfo request to each distinct DroneCAN node ID seen (the
           esc.Status transfer's source_node_id, i.e. the VESC's CAN ID / VESC Tool's
           controller_id -- a different number from esc_index, see docs/can_id_map.md) to read
           back that VESC's hardware unique_id ("PROM ID" / UUID). Cross-reference that against
           VESC Tool's own per-device UUID cache -- any device VESC Tool shows as not cached is
           one this esc_index/CAN ID assignment hasn't been made permanent for yet, and the UUID
           printed here is what lets you tell physical units apart on the bench before doing so.
  watch    Passive, continuous (Ctrl-C to stop): live table of the esc.RPMCommand ERPM actually
           on the bus per esc_index (the "CAN bus command" -- what vesc_dronecan_driver really
           sent, decoded straight off the wire, not what the code intended to send) next to that
           esc_index's latest esc.Status feedback. Run this while teleop/the full ros2_control
           pipeline is driving the robot to catch, live: a command that never reaches the bus at
           all (dead esc_index column), one stuck at 0 while others move (a per-joint bug, e.g.
           esc_index mixup), a magnitude that doesn't match what full stick should produce (see
           docs/can_id_map.md's ERPM formula), or a command present but no corresponding rpm in
           the Status feedback (motor not actually turning -- see docs/can_id_map.md's
           s_pid_min_erpm gate and Hall/encoder notes for known causes of exactly that symptom).
           `status_erpm_equiv` converts Status's mechanical RPM back to ERPM
           (x --pole-pairs) so it's directly comparable to `cmd_erpm` -- the two should track
           each other at steady state; a persistent gap is the motor not following the command.
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


# docs/can_id_map.md's wheel index table (drive esc_index 1-4, 0 deliberately unused) and
# steering convention (actuator_id = drive esc_index + 4, i.e. 5-8).
_WHEEL_NAMES = {1: "Front-left", 2: "Front-right", 3: "Rear-left", 4: "Rear-right"}


def _label_and_task(esc_index: int) -> tuple:
    if esc_index in _WHEEL_NAMES:
        return _WHEEL_NAMES[esc_index], "drive"
    steering_wheel = _WHEEL_NAMES.get(esc_index - 4)
    if steering_wheel is not None:
        # NOT a config error: firmware's periodic status loop unconditionally calls both
        # sendEscStatus() and sendActuatorStatus() every period for every VESC (see
        # bldc/libcanard/canard_driver.c), so esc.Status at a steering actuator_id is expected --
        # role (drive vs steering) is decided by which *command* message the PC sends, not which
        # status message the VESC emits (both are always emitted).
        return f"{steering_wheel} steering?", "actuator_id -- also broadcasts actuator.Status"
    return "unassigned", "unknown -- not in docs/can_id_map.md"


def _query_uuids(dronecan, node, node_ids, timeout: float = 2.0) -> dict:
    """GetNodeInfo each of node_ids, returning {node_id: uuid_hex or None on timeout}."""
    uuids = {node_id: None for node_id in node_ids}

    def make_callback(node_id):
        def callback(event):
            if event is not None:  # None means the request timed out, leave uuids[node_id] as-is
                raw = list(event.response.hardware_version.unique_id)
                uuids[node_id] = "".join(f"{b:02X}" for b in raw)
        return callback

    for node_id in node_ids:
        node.request(dronecan.uavcan.protocol.GetNodeInfo.Request(), node_id,
                      make_callback(node_id), timeout=timeout)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            node.spin(timeout=0)
        except Exception as exc:  # noqa: BLE001 - a bad received frame must not kill this tool
            print(f"...spin error, ignoring: {exc}")
        time.sleep(0.01)
    return uuids


def cmd_listen(dronecan, node, seconds: float) -> bool:
    # Keyed by (esc_index, node_id) rather than just esc_index -- if two physical VESCs share an
    # esc_index, overwriting by esc_index alone would silently hide it (whichever's frame arrived
    # last "wins" and the collision never surfaces). Keeping both keyed separately means a
    # collision shows up as two rows sharing one esc_index instead of one row flip-flopping
    # between two node_ids across runs.
    seen = {}

    def on_status(event):
        s = event.message
        node_id = event.transfer.source_node_id
        seen[(s.esc_index, node_id)] = (s.rpm, s.voltage, s.current, s.temperature)

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

    node_ids = sorted({node_id for _, node_id in seen})
    print(f"Querying GetNodeInfo for {len(node_ids)} distinct CAN/node ID(s) to read back each "
          "VESC's hardware UUID...")
    uuids = _query_uuids(dronecan, node, node_ids)

    esc_indices = sorted({idx for idx, _ in seen})
    collisions = {idx for idx in esc_indices
                  if sum(1 for i, _ in seen if i == idx) > 1}
    if collisions:
        print(f"*** COLLISION: esc_index {sorted(collisions)} each have MORE THAN ONE physical "
              "VESC broadcasting -- set can_esc_index to a unique value on each via VESC Tool. "
              "***")

    print(f"{len(seen)} distinct (esc_index, node_id) pairs seen (label/task per "
          "docs/can_id_map.md):")
    rows = []
    for idx, node_id in sorted(seen):
        rpm, volt, cur, temp = seen[(idx, node_id)]
        label, task = _label_and_task(idx)
        if idx in collisions:
            task = "COLLISION -- " + task
        uuid = uuids.get(node_id) or "no GetNodeInfo response"
        rows.append((str(idx), label, task, str(node_id), uuid, str(rpm), f"{volt:.1f}V",
                     f"{cur:.1f}A", f"{temp:.1f}K"))
    headers = ("esc_index", "label", "task", "node_id", "uuid", "rpm", "voltage", "current",
               "temp")
    widths = [max(len(h), *(len(r[i]) for r in rows)) for i, h in enumerate(headers)]
    row_fmt = "  " + "  ".join(f"{{:<{w}}}" for w in widths)
    print(row_fmt.format(*headers))
    print(row_fmt.format(*("-" * w for w in widths)))
    for row in rows:
        print(row_fmt.format(*row))
    if any(idx not in _WHEEL_NAMES for idx in seen):
        print("Note: esc_index 5-8 broadcasting esc.Status is expected, not a config error --"
              " firmware sends both esc.Status and actuator.Status for every VESC every period"
              " regardless of role (bldc/libcanard/canard_driver.c); role is decided by which"
              " command message the PC sends, not which status message comes back.")
    return True


_WATCH_STALE_AFTER = 1.0   # seconds since last message before a column is shown as stale ("--")
_WATCH_DROP_AFTER = 10.0   # seconds since last message before an esc_index row is dropped
_WATCH_REFRESH = 0.2       # seconds between redraws


def cmd_watch(dronecan, node, pole_pairs: float, seconds: float) -> None:
    # cmd_seen/status_seen are separate dicts, not one merged table, because RPMCommand and
    # Status are independent broadcasts on independent schedules -- a row with a fresh command
    # and a stale/missing status (or vice versa) is exactly the mismatch this mode exists to
    # surface, so the two must be able to go stale independently.
    cmd_seen = {}     # esc_index -> (rpm_value, last_seen)
    status_seen = {}  # esc_index -> (rpm, voltage, current, temperature, last_seen)

    def on_rpm_command(event):
        now = time.monotonic()
        for esc_index, value in enumerate(event.message.rpm):
            if esc_index == 0:
                continue  # esc_index 0 is deliberately unused, see docs/can_id_map.md
            cmd_seen[esc_index] = (value, now)

    def on_status(event):
        s = event.message
        status_seen[s.esc_index] = (s.rpm, s.voltage, s.current, s.temperature, time.monotonic())

    node.add_handler(dronecan.uavcan.equipment.esc.RPMCommand, on_rpm_command)
    node.add_handler(dronecan.uavcan.equipment.esc.Status, on_status)

    print(f"Watching esc.RPMCommand (CAN bus command) + esc.Status on the bus"
          + (f" for {seconds:.0f}s" if seconds > 0 else " until Ctrl-C") + "...")
    is_tty = sys.stdout.isatty()
    deadline = time.monotonic() + seconds if seconds > 0 else None
    headers = ("esc_index", "label", "cmd_erpm", "cmd_age", "status_rpm", "status_erpm_equiv",
               "status_age")
    try:
        while deadline is None or time.monotonic() < deadline:
            try:
                node.spin(timeout=0)
            except Exception as exc:  # noqa: BLE001 - a bad received frame must not kill this
                print(f"...spin error, ignoring: {exc}")
            now = time.monotonic()
            for table, ttl in ((cmd_seen, _WATCH_DROP_AFTER), (status_seen, _WATCH_DROP_AFTER)):
                for key in [k for k, v in table.items() if now - v[-1] > ttl]:
                    del table[key]

            rows = []
            for idx in sorted(set(cmd_seen) | set(status_seen)):
                label, _task = _label_and_task(idx)
                if idx in cmd_seen:
                    value, last_seen = cmd_seen[idx]
                    age = now - last_seen
                    cmd_col = str(value) if age <= _WATCH_STALE_AFTER else f"({value}, stale)"
                    cmd_age = f"{age:.1f}"
                else:
                    cmd_col, cmd_age = "no RPMCommand seen", "--"
                if idx in status_seen:
                    rpm, _volt, _cur, _temp, last_seen = status_seen[idx]
                    age = now - last_seen
                    status_col = str(rpm) if age <= _WATCH_STALE_AFTER else f"({rpm}, stale)"
                    erpm_equiv = f"{rpm * pole_pairs:.0f}"
                    status_age = f"{age:.1f}"
                else:
                    status_col, erpm_equiv, status_age = "no Status seen", "--", "--"
                rows.append((str(idx), label, cmd_col, cmd_age, status_col, erpm_equiv,
                             status_age))

            if is_tty:
                print("\x1b[2J\x1b[H", end="")  # clear screen + move cursor home
            if not rows:
                print("No esc.RPMCommand or esc.Status seen yet...")
            else:
                widths = [max(len(h), *(len(r[i]) for r in rows)) for i, h in enumerate(headers)]
                row_fmt = "  " + "  ".join(f"{{:<{w}}}" for w in widths)
                print(row_fmt.format(*headers))
                print(row_fmt.format(*("-" * w for w in widths)))
                for row in rows:
                    print(row_fmt.format(*row))
            time.sleep(_WATCH_REFRESH)
    except KeyboardInterrupt:
        pass


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

    p_watch = sub.add_parser("watch", help="Passive, always safe: live esc.RPMCommand "
                              "(CAN bus command) vs esc.Status, continuously.")
    p_watch.add_argument("--seconds", type=float, default=0.0,
                          help="Stop after this many seconds; 0 (default) runs until Ctrl-C.")
    p_watch.add_argument("--pole-pairs", type=float, default=7.0,
                          help="motor_pole_pairs from urdf/rp1_drive.urdf (default 7.0) -- used "
                          "only to convert status_rpm (mechanical) to status_erpm_equiv for "
                          "comparison against cmd_erpm; doesn't affect what's actually decoded "
                          "off the bus.")

    p_pulse = sub.add_parser("pulse", help="Send a real RawCommand duty-cycle pulse to ONE "
                              "esc_index. WHEELS MUST BE OFF THE GROUND.")
    p_pulse.add_argument("esc_index", type=int, choices=(1, 2, 3, 4))
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
        elif args.mode == "watch":
            cmd_watch(dronecan, node, args.pole_pairs, args.seconds)
            return 0
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
