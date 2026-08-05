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
ist8310_driver_node.py, elrs_driver_node.py): each transport lib (spidev / smbus2 / pyserial)
is imported lazily so this script only requires whichever libs the buses you actually test
need, and a missing lib fails with the pip install line rather than a bare ImportError.

Run standalone or via `./robotpicks.sh smoke bus` (see robotpicks.sh).
"""

import argparse
import errno
import sys

PASS, WARN, FAIL, SKIP = "PASS", "WARN", "FAIL", "SKIP"


def _report(bus: str, status: str, detail: str) -> None:
    print(f"[{status:4s}] {bus}: {detail}")


def _missing_module(bus: str, module: str, pip_name: str) -> None:
    _report(bus, FAIL, f"python module '{module}' not found. Install it with:")
    print(f"\n  pip install --break-system-packages {pip_name}\n")


def check_spi(bus_num: int, device: int, speed_hz: int, loopback: bool) -> str:
    label = f"SPI (bus={bus_num} device={device})"
    try:
        import spidev  # lazy: not required unless SPI is actually checked
    except ImportError:
        _missing_module(label, "spidev", "spidev")
        return FAIL

    try:
        spi = spidev.SpiDev()
        spi.open(bus_num, device)
        spi.max_speed_hz = speed_hz
        spi.mode = 0b00
    except FileNotFoundError:
        _report(label, FAIL, f"no /dev/spidev{bus_num}.{device} -- wrong bus/device number, "
                "converter not in SPI mode, or driver not bound yet.")
        return FAIL
    except PermissionError:
        _report(label, FAIL, f"/dev/spidev{bus_num}.{device} exists but isn't readable/writable "
                "by this user (add to the 'spi'/'dialout' group or check udev rules).")
        return FAIL

    try:
        if loopback:
            pattern = list(range(32))
            echoed = spi.xfer2(list(pattern))
            if echoed == pattern:
                _report(label, PASS, "loopback pattern echoed back unchanged -- "
                        "MOSI/MISO continuity confirmed.")
                return PASS
            _report(label, FAIL, f"loopback mismatch: sent {pattern[:8]}..., got "
                    f"{echoed[:8]}... -- check the MOSI-MISO jumper.")
            return FAIL
        else:
            spi.xfer2([0x00])
            _report(label, PASS, "device node opened and a transfer completed with no I/O "
                    "error. This does not confirm the physical wiring to the sensor header -- "
                    "rerun with --loopback (MOSI-MISO jumper) for that.")
            return PASS
    except OSError as exc:
        _report(label, FAIL, f"transfer failed: {exc}")
        return FAIL
    finally:
        spi.close()


def check_i2c(bus_num: int) -> str:
    label = f"I2C (bus={bus_num})"
    try:
        import smbus2  # lazy: not required unless I2C is actually checked
    except ImportError:
        _missing_module(label, "smbus2", "smbus2")
        return FAIL

    try:
        bus = smbus2.SMBus(bus_num)
    except FileNotFoundError:
        _report(label, FAIL, f"no /dev/i2c-{bus_num} -- wrong bus number, converter not in "
                "I2C mode, or driver not bound yet.")
        return FAIL
    except PermissionError:
        _report(label, FAIL, f"/dev/i2c-{bus_num} exists but isn't readable/writable by this "
                "user (add to the 'i2c' group or check udev rules).")
        return FAIL

    found = []
    unexpected_errno = None
    try:
        # Standard 7-bit address scan (skips the reserved 0x00-0x02 and 0x78-0x7F ranges, same
        # as i2cdetect). With nothing attached, every address is expected to NAK -- that's the
        # normal/no-device-attached result, not a failure. A bus that instead hangs or errors
        # with something other than "no device answered" points at a wiring problem (SDA/SCL
        # shorted or stuck).
        for addr in range(0x03, 0x78):
            try:
                bus.read_byte(addr)
                found.append(addr)
            except OSError as exc:
                if exc.errno not in (errno.ENXIO, errno.EREMOTEIO, errno.EIO):
                    unexpected_errno = exc
    finally:
        bus.close()

    if unexpected_errno is not None:
        _report(label, FAIL, f"bus scan hit an unexpected error ({unexpected_errno}) -- "
                "possible SDA/SCL short or stuck line.")
        return FAIL
    if found:
        addrs = ", ".join(f"0x{a:02X}" for a in found)
        _report(label, PASS, f"bus scan completed; device(s) ACKed at: {addrs}.")
    else:
        _report(label, PASS, "bus scan completed, no address ACKed -- expected with no "
                "device attached. This confirms the bus opens and doesn't hang; it does not "
                "confirm SDA/SCL continuity to the sensor header (I2C has no loopback check).")
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
    parser.add_argument("--spi-bus", type=int, default=0)
    parser.add_argument("--spi-device", type=int, default=0)
    parser.add_argument("--spi-speed", type=int, default=1_000_000)
    parser.add_argument("--skip-spi", action="store_true")
    parser.add_argument("--i2c-bus", type=int, default=1)
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
        _report(f"SPI (bus={args.spi_bus} device={args.spi_device})", SKIP, "--skip-spi")
    else:
        results.append(check_spi(args.spi_bus, args.spi_device, args.spi_speed, args.loopback))

    if args.skip_i2c:
        _report(f"I2C (bus={args.i2c_bus})", SKIP, "--skip-i2c")
    else:
        results.append(check_i2c(args.i2c_bus))

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
