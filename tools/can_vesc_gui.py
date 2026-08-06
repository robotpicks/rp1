#!/usr/bin/env python3
"""GUI counterpart of can_vesc_test.py's 'listen' mode: a live-updating PyQt5 table of every
esc.Status broadcasting on the bus (esc_index, node_id, UUID, rpm/voltage/current/temperature),
for the same bench-verification use case as docs/can_id_map.md's "confirm per-unit bus presence"
note -- just watched continuously instead of a fixed-duration snapshot, and easier to eyeball
while walking the bench toggling VESCs on/off one at a time.

Passive only -- like 'listen', this never broadcasts a command, so it's always safe to run.
There is no GUI equivalent of 'pulse' (sending RawCommand still goes through can_vesc_test.py's
explicit e-stop confirmation prompt, deliberately not a button click).

Rows flagged COLLISION (more than one physical VESC sharing an esc_index) are highlighted, same
condition can_vesc_test.py's listen reports as "*** COLLISION ***". Rows that stop updating are
greyed out after STALE_AFTER seconds and dropped after DROP_AFTER, so a VESC that's powered off
disappears instead of leaving a frozen last-known row indistinguishable from a live one.

Reuses _label_and_task/_force_native_socketcan_driver from can_vesc_test.py (same directory)
rather than redefining the wheel-index table and DroneCAN driver workaround a second time -- see
that file's docstring for why the native-SocketCAN-driver + spin(timeout=0) dance is needed with
this sandbox's dronecan + python-can pair.

_refresh()'s in-place row diffing (update/insert/remove by key instead of clearing and
rebuilding the whole table every tick) mirrors OpenCyphal-Garage/gui_tool's (formerly UAVCAN GUI
Tool, archived) NodeTable._update -- that project is built on pyuavcan_v0, the direct ancestor of
the 'dronecan' package used here (DroneCAN is the community fork of UAVCAN v0 after the UAVCAN
project itself moved on to Cyphal/UAVCAN v1), so its table-refresh pattern applies directly. Its
newer sibling project, pycyphal, implements Cyphal v1 instead and is NOT wire-compatible with
this bus -- don't use it here.
"""

import argparse
import sys
import time

from can_vesc_test import _force_native_socketcan_driver, _label_and_task

STALE_AFTER = 2.0   # seconds since last esc.Status before a row is greyed out
DROP_AFTER = 10.0   # seconds since last esc.Status before a row is removed entirely

_COLUMNS = ("esc_index", "label", "task", "node_id", "uuid", "rpm", "voltage", "current", "temp",
            "age (s)")


class MonitorWindow:
    def __init__(self, QtWidgets, QtGui, QtCore, dronecan, node, iface: str):
        self.QtWidgets = QtWidgets
        self.QtGui = QtGui
        self.dronecan = dronecan
        self.node = node
        self.iface = iface
        self.seen = {}       # (esc_index, node_id) -> (rpm, voltage, current, temp, last_seen)
        self.uuids = {}      # node_id -> uuid_hex, or "no GetNodeInfo response"
        self.requested = set()  # node_ids we've already sent a GetNodeInfo request for

        self.window = QtWidgets.QMainWindow()
        self.window.setWindowTitle(f"can_vesc_gui -- {iface}, listening (passive, sends nothing)")
        self.window.resize(900, 400)

        self.table = QtWidgets.QTableWidget(0, len(_COLUMNS))
        self.table.setHorizontalHeaderLabels(_COLUMNS)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        self.table.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        self._row_keys = []  # (esc_index, node_id) per row, kept in sync with table row order
        self.window.setCentralWidget(self.table)
        self.window.statusBar().showMessage(
            f"Listening for esc.Status on {iface} (own node id {node.node_id})...")

        node.add_handler(dronecan.uavcan.equipment.esc.Status, self._on_status)

        self.spin_timer = QtCore.QTimer()
        self.spin_timer.timeout.connect(self._spin)
        self.spin_timer.start(20)

        self.refresh_timer = QtCore.QTimer()
        self.refresh_timer.timeout.connect(self._refresh)
        self.refresh_timer.start(300)

    def show(self):
        self.window.show()

    def _on_status(self, event):
        s = event.message
        node_id = event.transfer.source_node_id
        self.seen[(s.esc_index, node_id)] = (s.rpm, s.voltage, s.current, s.temperature,
                                              time.monotonic())

    def _spin(self):
        try:
            self.node.spin(timeout=0)
        except Exception as exc:  # noqa: BLE001 - a bad received frame must not kill this tool
            self.window.statusBar().showMessage(f"...spin error, ignoring: {exc}", 3000)

    def _request_uuid(self, node_id: int) -> None:
        if node_id in self.requested:
            return
        self.requested.add(node_id)

        def callback(event):
            if event is None:  # request timed out
                self.uuids[node_id] = "no GetNodeInfo response"
            else:
                raw = list(event.response.hardware_version.unique_id)
                self.uuids[node_id] = "".join(f"{b:02X}" for b in raw)

        try:
            self.node.request(self.dronecan.uavcan.protocol.GetNodeInfo.Request(), node_id,
                               callback, timeout=2.0)
        except Exception as exc:  # noqa: BLE001 - a request failure must not kill this tool
            self.uuids[node_id] = f"request error: {exc}"

    def _refresh(self):
        now = time.monotonic()
        for key in [k for k, v in self.seen.items() if now - v[4] > DROP_AFTER]:
            del self.seen[key]

        for node_id in {node_id for _, node_id in self.seen}:
            self._request_uuid(node_id)

        esc_indices = sorted({idx for idx, _ in self.seen})
        collisions = {idx for idx in esc_indices
                      if sum(1 for i, _ in self.seen if i == idx) > 1}

        # Diff self._row_keys against self.seen instead of clearing/rebuilding the table every
        # tick -- avoids flicker and losing the user's scroll position/selection (see this
        # file's docstring: pattern borrowed from OpenCyphal-Garage/gui_tool's NodeTable._update).
        for row in range(len(self._row_keys) - 1, -1, -1):  # remove from the end while iterating
            if self._row_keys[row] not in self.seen:
                self.table.removeRow(row)
                del self._row_keys[row]

        displayed = set(self._row_keys)
        for key in sorted(self.seen.keys()):
            if key not in displayed:
                row = next((r for r, k in enumerate(self._row_keys) if k > key),
                           len(self._row_keys))
                self.table.insertRow(row)
                self._row_keys.insert(row, key)

        for row, key in enumerate(self._row_keys):
            idx, node_id = key
            rpm, volt, cur, temp, last_seen = self.seen[key]
            label, task = _label_and_task(idx)
            if idx in collisions:
                task = "COLLISION -- " + task
            uuid = self.uuids.get(node_id, "querying...")
            age = now - last_seen
            values = (str(idx), label, task, str(node_id), uuid, str(rpm), f"{volt:.1f}V",
                      f"{cur:.1f}A", f"{temp:.1f}K", f"{age:.1f}")
            for c, value in enumerate(values):
                self._set_cell(row, c, value, collision=idx in collisions, stale=age > STALE_AFTER)

        self.table.resizeColumnsToContents()
        note = f"{len(self._row_keys)} (esc_index, node_id) pair(s) seen on {self.iface}"
        if collisions:
            note += f" -- *** COLLISION on esc_index {sorted(collisions)} ***"
        self.window.statusBar().showMessage(note)

    def _set_cell(self, row: int, col: int, text: str, collision: bool, stale: bool) -> None:
        item = self.QtWidgets.QTableWidgetItem(text)
        if collision:
            item.setBackground(self.QtGui.QColor(255, 190, 190))
        elif stale:
            item.setForeground(self.QtGui.QColor(150, 150, 150))
        self.table.setItem(row, col, item)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--iface", default="can0")
    parser.add_argument("--node-id", type=int, default=98,
                         help="This tool's own DroneCAN node ID -- distinct from "
                         "can_vesc_test.py's 99 and the real vesc_dronecan_driver's 42 (see "
                         "docs/can_id_map.md), so all three can safely coexist on the bus.")
    args = parser.parse_args()

    try:
        import dronecan
    except ImportError:
        print("python module 'dronecan' not found. Install it with:\n\n"
              "  pip install --break-system-packages dronecan\n", file=sys.stderr)
        return 1

    try:
        from PyQt5 import QtCore, QtGui, QtWidgets
    except ImportError:
        print("python module 'PyQt5' not found. Install it with:\n\n"
              "  sudo apt install python3-pyqt5\n", file=sys.stderr)
        return 1

    _force_native_socketcan_driver()
    node = dronecan.make_node(args.iface, node_id=args.node_id, bitrate=1000000)

    app = QtWidgets.QApplication(sys.argv)
    window = MonitorWindow(QtWidgets, QtGui, QtCore, dronecan, node, args.iface)
    window.show()
    try:
        return app.exec_()
    finally:
        node.close()


if __name__ == "__main__":
    sys.exit(main())
