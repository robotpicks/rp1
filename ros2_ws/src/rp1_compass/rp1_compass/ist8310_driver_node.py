"""IST8310 3-axis magnetometer driver over I2C.

This is the compass bundled on the GPS module's own breakout board (Dalang DL25F10NQ,
u-blox NEO-F10N GNSS + IST8310 compass on one PCB) -- a physically distinct chip from the
ICM20948's internal AK09916 magnetometer that rp1_imu already publishes on `imu/mag`. Both
happen to sit at I2C address 0x0C, but on two unrelated buses (the AK09916 is reached only
through the ICM20948's internal auxiliary-I2C passthrough over SPI; this chip is on the
Waveshare converter's external I2C bus) -- there is no real address collision.

UNVERIFIED AGAINST REAL HARDWARE. Written from the public IST8310 register map (and the
common single-measurement-per-cycle trigger pattern used by open-source IST8310 drivers,
e.g. ArduPilot's AP_Compass_IST8310) without a physical sensor to test against. Confirm
WHO_AM_I reads back 0x10 and sanity-check the field magnitude (~25-65 uT depending on
location) against real hardware before trusting this for anything closed-loop. The
AVGCNTL/PDCNTL register values below are the datasheet's own recommended defaults, not
independently verified -- the fiddliest/most-likely-wrong part of this driver, along with
the CNTL1 trigger-then-poll-DRDY-next-cycle timing (see _poll's comment).

Design conventions match icm20948_driver_node.py (rp1_imu) for consistency across rp1's
hardware drivers: the transport lib (smbus2) is imported lazily so the package builds and
dry-runs without it; require_i2c: false opens no device and logs intended reads instead
(handy for CI / bring-up without hardware); and all I2C I/O is wrapped so a hardware hiccup
logs but never kills the node.
"""

import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node as RosNode
from sensor_msgs.msg import MagneticField

# -- IST8310 register map -------------------------------------------------------------------
WAI = 0x00
WAI_EXPECTED = 0x10

STAT1 = 0x02       # bit0: DRDY (new data ready)
DRDY_BIT = 0x01

OUTPUT_X_L = 0x03   # 6 contiguous bytes: X, Y, Z, little-endian int16 each

STAT2 = 0x09        # bit3: DOR (data overrun)
DOR_BIT = 0x08

CNTL1 = 0x0A        # 0x00 standby, 0x01 single-measurement trigger
CNTL1_STANDBY = 0x00
CNTL1_SINGLE_MEASUREMENT = 0x01

CNTL2 = 0x0B        # bit0: SRST (soft reset, self-clearing)
CNTL2_SRST = 0x01

AVGCNTL = 0x41       # averaging control -- datasheet-recommended default below
PDCNTL = 0x42        # pulse duration control -- datasheet-recommended default below
AVGCNTL_DEFAULT = 0x00   # no averaging (datasheet default)
PDCNTL_DEFAULT = 0xC0    # datasheet's recommended normal-operation value

# IST8310: 0.3 uT/LSB, commonly cited sensitivity from the public datasheet.
MICROTESLA_PER_LSB = 0.3
TESLA_PER_LSB = MICROTESLA_PER_LSB * 1e-6


class Ist8310DriverNode(RosNode):

    def __init__(self):
        super().__init__('ist8310_driver')

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x0C)
        self.declare_parameter('require_i2c', True)
        self.declare_parameter('frame_id', 'compass_link')
        self.declare_parameter('rate_hz', 20.0)

        self._frame_id = self.get_parameter('frame_id').value
        self._require_i2c = self.get_parameter('require_i2c').value
        self._address = self.get_parameter('i2c_address').value

        self._bus = None
        self._mag_pub = self.create_publisher(MagneticField, 'compass/mag', 10)

        if self._require_i2c:
            self._bus = self._open_i2c()
            self._configure_chip()
            rate_hz = self.get_parameter('rate_hz').value
            self.create_timer(1.0 / rate_hz, self._poll)
        else:
            self.get_logger().warning(
                'require_i2c=false: dry-run, no I2C device opened; no data will be published')

    # -- I2C transport -----------------------------------------------------------------------

    def _open_i2c(self):
        import smbus2  # lazy: not required to import/build the package without hardware
        bus_num = self.get_parameter('i2c_bus').value
        bus = smbus2.SMBus(bus_num)
        self.get_logger().info(f'I2C open: bus={bus_num} address=0x{self._address:02X}')
        return bus

    def _write_reg(self, reg: int, value: int) -> None:
        self._bus.write_byte_data(self._address, reg, value)

    def _read_regs(self, reg: int, n: int) -> bytes:
        return bytes(self._bus.read_i2c_block_data(self._address, reg, n))

    # -- one-time chip configuration ----------------------------------------------------------

    def _configure_chip(self) -> None:
        who_am_i = self._read_regs(WAI, 1)[0]
        if who_am_i != WAI_EXPECTED:
            self.get_logger().error(
                f'IST8310 WHO_AM_I=0x{who_am_i:02X}, expected 0x{WAI_EXPECTED:02X} -- wrong '
                'chip, bad wiring, or wrong I2C bus/address. Continuing anyway; data is '
                'suspect until this is fixed.')

        self._write_reg(CNTL2, CNTL2_SRST)  # soft reset, self-clearing
        self._write_reg(AVGCNTL, AVGCNTL_DEFAULT)
        self._write_reg(PDCNTL, PDCNTL_DEFAULT)
        self._write_reg(CNTL1, CNTL1_SINGLE_MEASUREMENT)  # arm the first conversion

    # -- polling -------------------------------------------------------------------------------

    def _poll(self) -> None:
        """The IST8310 has no free-running continuous mode this driver relies on -- each cycle
        checks DRDY for the conversion triggered last cycle, publishes if ready, then triggers
        the next one. This trades a poll-cycle of latency (~1/rate_hz) for simplicity; fine at
        this driver's default 20Hz against the chip's ~3.6ms typical conversion time."""
        try:
            status = self._read_regs(STAT1, 1)[0]
            if status & DRDY_BIT:
                raw = self._read_regs(OUTPUT_X_L, 6)
                stat2 = self._read_regs(STAT2, 1)[0]
                self._write_reg(CNTL1, CNTL1_SINGLE_MEASUREMENT)  # arm the next conversion
                if stat2 & DOR_BIT:
                    self.get_logger().warning('IST8310 DOR (data overrun) -- reading discarded')
                    return
                self._publish(raw)
            else:
                self._write_reg(CNTL1, CNTL1_SINGLE_MEASUREMENT)  # not ready; re-arm anyway
        except Exception as exc:  # noqa: BLE001 - an I2C error must not kill the node
            self.get_logger().error(f'I2C read error: {exc}')

    def _publish(self, raw: bytes) -> None:
        mx, my, mz = struct.unpack('<3h', raw)
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.magnetic_field.x = mx * TESLA_PER_LSB
        msg.magnetic_field.y = my * TESLA_PER_LSB
        msg.magnetic_field.z = mz * TESLA_PER_LSB
        self._mag_pub.publish(msg)

    def destroy_node(self) -> bool:
        if self._bus is not None:
            try:
                self._bus.close()
            except Exception as exc:  # a failed close must not mask the shutdown itself
                self.get_logger().warning(f'error closing I2C bus: {exc}')
            self._bus = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Ist8310DriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
