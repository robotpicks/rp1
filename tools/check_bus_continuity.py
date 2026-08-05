#!/usr/bin/env python3
"""Bench check: confirm the Waveshare USB-to-UART/I2C/SPI/JTAG converter's three buses
(SPI -> rp1_imu, I2C -> rp1_compass, UART1 -> rp1_gps; see rp1-specs/electrical_spec.md's
"Second-wave sensor physical interfaces") are wired up and alive, **before** any of the real
sensors are attached to them. This is deliberately not a chip test -- rp1_imu/rp1_compass
already do a WHO_AM_I check once a real sensor is on the bus (see their driver nodes' module
docstrings); this script exists for the step before that, where there's nothing on the other
end yet, so there's no WHO_AM_I to read.

Two levels of check, selected by --loopback:

  * Default (no jumper needed): open each bus and perform the smallest transfer that proves
    the kernel side of the link is alive (SPI: one xfer; I2C: a full address-range scan, since
    every address is expected to NAK with nothing attached; UART: open the port and read
    briefly). This confirms the /dev node exists, permissions are right, and the driver isn't
    wedged -- it does NOT prove the physical wires between the converter and the sensor
    headers are intact, since nothing is attached to loop the signal back through.
  * --loopback (bench jumper required): short MOSI to MISO (SPI) and TX to RX (UART) at the
    connector, then this sends a known byte pattern and checks it comes back unchanged. That
    exercises the actual copper, which is the only way to catch a cut trace, a bad crimp, or a
    swapped pin with no sensor attached to notice it for you. I2C has no loopback equivalent
    (SDA is a single bidirectional wire, not a TX/RX pair) -- the address scan is the whole
    check for that bus either way.

Design conventions match the other rp1 hardware drivers (icm20948_driver_node.py,
ist8310_driver_node.py, elrs_driver_node.py) where they apply: each transport lib is imported
lazily so this script only requires whichever libs the buses you actually test need, and a
missing lib fails with clear install instructions rather than a bare ImportError. UART still
goes through pyserial's normal /dev/tty* path. SPI and I2C do NOT, though -- both live on the
Waveshare converter's third USB interface (WCH CH347, Mode 1), which is not a generic Linux
spidev/i2c-dev device. It shows up as /dev/ch34x_pis0 via WCH's own out-of-tree kernel driver
(WCHSoftGroup/ch341par_linux's ch34x_pis.ko -- install per that repo's README, `make` + `sudo
make install` for it to persist across reboots; see also tools/99-ch34x-pis.rules for the udev
permission fix so this doesn't need root to run), and is spoken to through WCH's own
libch347.so via ch347lib.py (this directory) rather than spidev/smbus2 -- confirmed by checking
the mainline spi-ch341 kernel driver's own USB ID table, which only matches CH341A's PID
(0x5512), not this converter's PID (0x55db); forcing a match wouldn't help since CH347 speaks a
different wire protocol entirely (WCH's own out-of-tree driver has to branch on chip ID
internally for exactly this reason).

Run standalone or via `./robotpicks.sh smoke bus` (see robotpicks.sh).
"""

import argparse
import sys

PASS, WARN, FAIL, SKIP = "PASS", "WARN", "FAIL", "SKIP"


def _report(bus: str, status: str, detail: str) -> None:
    print(f"[{status:4s}] {bus}: {detail}")


def _missing_module(bus: str, module: str, pip_name: str) -> None:
    _report(bus, FAIL, f"python module '{module}' not found. Install it with:")
    print(f"\n  pip install --break-system-packages {pip_name}\n")


def _open_ch34x(label: str, device_path: str):
    try:
        import ch347lib  # lazy: not required unless SPI or I2C is actually checked
    except ImportError as exc:
        _report(label, FAIL, f"couldn't import ch347lib.py (same directory as this script): "
                f"{exc}")
        return None, None
    try:
        return ch347lib, ch347lib.Ch347Device(device_path)
    except OSError as exc:
        _report(label, FAIL, str(exc))
        return ch347lib, None


def check_spi(device_path: str, speed_hz: int, mode: int, loopback: bool) -> str:
    label = f"SPI ({device_path})"
    ch347lib, dev = _open_ch34x(label, device_path)
    if dev is None:
        return FAIL

    try:
        actual_hz = dev.spi_init(speed_hz, mode=mode)
        if loopback:
            pattern = bytes(range(32))
            echoed = dev.spi_write_read(pattern)
            if echoed == pattern:
                _report(label, PASS, f"({actual_hz} Hz) loopback pattern echoed back "
                        "unchanged -- MOSI/MISO continuity confirmed.")
                return PASS
            _report(label, FAIL, f"({actual_hz} Hz) loopback mismatch: sent "
                    f"{pattern[:8]!r}..., got {echoed[:8]!r}... -- check the MOSI-MISO jumper.")
            return FAIL
        else:
            dev.spi_write_read(b"\x00")
            _report(label, PASS, f"({actual_hz} Hz) device opened and a transfer completed "
                    "with no I/O error. This does not confirm the physical wiring to the "
                    "sensor header -- rerun with --loopback (MOSI-MISO jumper) for that.")
            return PASS
    except OSError as exc:
        _report(label, FAIL, f"transfer failed: {exc}")
        return FAIL
    finally:
        dev.close()


def check_i2c(device_path: str, mode: int) -> str:
    label = f"I2C ({device_path})"
    ch347lib, dev = _open_ch34x(label, device_path)
    if dev is None:
        return FAIL

    try:
        dev.i2c_set(mode)
        # Standard 7-bit address scan (skips the reserved 0x00-0x02 and 0x78-0x7F ranges, same
        # as i2cdetect). With nothing attached, every address is expected to NAK -- that's the
        # normal/no-device-attached result, not a failure. Unlike the smbus2 version of this
        # check, CH347StreamI2C's plain bool return can't distinguish "no device answered" from
        # a real bus fault (see ch347lib.Ch347Device.i2c_probe's docstring) -- so a stuck/shorted
        # bus shows up here as "no address ACKed" too, same as the expected no-device case. If
        # you need to tell those apart, check for a stuck-low SDA/SCL with a meter.
        found = [addr for addr in range(0x03, 0x78) if dev.i2c_probe(addr)]
    except OSError as exc:
        _report(label, FAIL, f"bus scan failed: {exc}")
        return FAIL
    finally:
        dev.close()

    if found:
        addrs = ", ".join(f"0x{a:02X}" for a in found)
        _report(label, PASS, f"bus scan completed; device(s) ACKed at: {addrs}.")
    else:
        _report(label, PASS, "bus scan completed, no address ACKed -- expected with no "
                "device attached. This confirms the bus opens and responds; it does not "
                "confirm SDA/SCL continuity to the sensor header (I2C has no loopback check, "
                "and this API can't distinguish 'no device' from a stuck bus -- see above).")
    return PASS


def check_uart(port: str, baud: int, loopback: bool) -> str:
    label = f"UART ({port} @ {baud})"
    try:
        import serial  # lazy: not required unless UART is actually checked
    except ImportError:
        _missing_module(label, "serial", "pyserial")
        return FAIL

    try:
        ser = serial.Serial(port, baudrate=baud, timeout=1.0)
    except FileNotFoundError:
        _report(label, FAIL, f"no {port} -- wrong port, converter not in UART mode, or "
                "driver not bound yet.")
        return FAIL
    except serial.SerialException as exc:
        _report(label, FAIL, f"couldn't open {port}: {exc}")
        return FAIL

    try:
        if loopback:
            pattern = bytes(range(32))
            ser.reset_input_buffer()
            ser.write(pattern)
            ser.flush()
            echoed = ser.read(len(pattern))
            if echoed == pattern:
                _report(label, PASS, "loopback pattern echoed back unchanged -- "
                        "TX/RX continuity confirmed.")
                return PASS
            _report(label, FAIL, f"loopback mismatch: sent {len(pattern)} bytes, got "
                    f"{len(echoed)} back ({echoed!r}) -- check the TX-RX jumper.")
            return FAIL
        else:
            _report(label, PASS, "port opened at the configured baud rate with no I/O error. "
                    "This does not confirm the physical wiring to the sensor header -- rerun "
                    "with --loopback (TX-RX jumper) for that.")
            return PASS
    finally:
        ser.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--ch34x-device", default="/dev/ch34x_pis0",
                         help="SPI and I2C both ride the converter's third USB interface "
                         "through this one device node (see ch347lib.py).")
    parser.add_argument("--spi-speed", type=int, default=1_000_000,
                         help="Snapped to the nearest of the fixed rates libch347.so supports "
                         "(see ch347lib.SPI_FREQUENCIES_HZ).")
    parser.add_argument("--spi-mode", type=int, default=0, choices=(0, 1, 2, 3))
    parser.add_argument("--skip-spi", action="store_true")
    parser.add_argument("--i2c-rate-mode", type=int, default=0x01,
                         help="Raw mode bits for CH347I2C_Set; default 0x01 = 100kHz standard "
                         "rate. See ch347_lib.h's CH347I2C_Set doc comment for the full table.")
    parser.add_argument("--skip-i2c", action="store_true")
    parser.add_argument("--uart-port", default="/dev/ttyACM0")
    parser.add_argument("--uart-baud", type=int, default=38400)
    parser.add_argument("--skip-uart", action="store_true")
    parser.add_argument("--loopback", action="store_true",
                         help="Assume a MOSI-MISO jumper (SPI) and TX-RX jumper (UART) at the "
                         "connector and verify a sent pattern echoes back unchanged. Without "
                         "this flag, buses are only checked for opening/transferring cleanly.")
    args = parser.parse_args()

    results = []
    if args.skip_spi:
        _report(f"SPI ({args.ch34x_device})", SKIP, "--skip-spi")
    else:
        results.append(check_spi(args.ch34x_device, args.spi_speed, args.spi_mode,
                                  args.loopback))

    if args.skip_i2c:
        _report(f"I2C ({args.ch34x_device})", SKIP, "--skip-i2c")
    else:
        results.append(check_i2c(args.ch34x_device, args.i2c_rate_mode))

    if args.skip_uart:
        _report(f"UART ({args.uart_port} @ {args.uart_baud})", SKIP, "--skip-uart")
    else:
        results.append(check_uart(args.uart_port, args.uart_baud, args.loopback))

    if not results:
        print("Nothing checked -- all buses skipped.")
        return 1
    return 0 if all(r == PASS for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
