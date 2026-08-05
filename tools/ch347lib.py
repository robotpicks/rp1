"""ctypes bindings for WCH's libch347.so, covering the SPI/I2C subset used by
check_bus_continuity.py against the Waveshare converter's /dev/ch34x_pis0 (Mode 1: UART1 +
SPI + I2C, see rp1-specs/electrical_spec.md). There is no spidev/smbus2 path for this device --
see the module docstring in check_bus_continuity.py for why -- so this exists to talk the
vendor's own protocol instead.

Struct layout and function signatures are transcribed directly from WCH's own ch347_lib.h
(WCHSoftGroup/ch341par_linux), not reverse-engineered -- keep this file in step with that
header if it ever changes. Only the handful of calls actually needed here are wrapped; see
ch347_lib.h itself (or WCHSoftGroup/ch341par_linux/demo/ch347/ch347_demo.c for real call-site
examples, e.g. the I2C address-byte format below) for the rest of the API (JTAG, GPIO, EEPROM).

libch347.so itself is a vendor binary, not something to vendor into this repo -- build/install
it per WCH's README:
    git clone https://github.com/WCHSoftGroup/ch341par_linux
    sudo cp ch341par_linux/lib/x64/dynamic/libch347.so /usr/lib/ && sudo ldconfig
(swap x64 for your arch's subdirectory). This module finds it via CH347_LIB_PATH if set,
otherwise the standard library search path (i.e. after the sudo cp above + ldconfig).
"""

import ctypes
import ctypes.util
import os

# SPI clock rates CH347SPI_SetFrequency actually accepts (Hz) -- ch347_lib.h's own comment on
# CH347SPI_SetFrequency lists exactly these; anything else is silently rejected by the vendor
# lib, so callers should snap to the nearest one rather than pass an arbitrary Hz value.
SPI_FREQUENCIES_HZ = (60_000_000, 30_000_000, 15_000_000, 7_500_000,
                      3_750_000, 1_875_000, 937_500, 468_750)


def nearest_spi_frequency(hz: int) -> int:
    return min(SPI_FREQUENCIES_HZ, key=lambda f: abs(f - hz))


class SpiConfig(ctypes.Structure):
    """Mirrors ch347_lib.h's `_SPI_CONFIG` (mSpiCfgS) -- field order and the header's own
    `#pragma pack(1)` both matter here since this crosses the ctypes/C boundary by layout."""
    _pack_ = 1
    _fields_ = [
        ("iMode", ctypes.c_uint8),                # 0-3: SPI mode 0/1/2/3
        ("iClock", ctypes.c_uint8),                # unused here -- frequency set via
                                                    # CH347SPI_SetFrequency instead (preferred
                                                    # per the header's own doc comment)
        ("iByteOrder", ctypes.c_uint8),             # 0: LSB, 1: MSB
        ("iSpiWriteReadInterval", ctypes.c_uint16),
        ("iSpiOutDefaultData", ctypes.c_uint8),
        ("iChipSelect", ctypes.c_uint32),           # BIT7: CS1 control, BIT15: CS2 control
        ("CS1Polarity", ctypes.c_uint8),
        ("CS2Polarity", ctypes.c_uint8),
        ("iIsAutoDeativeCS", ctypes.c_uint16),
        ("iActiveDelay", ctypes.c_uint16),
        ("iDelayDeactive", ctypes.c_uint32),
    ]


def _find_lib() -> str:
    override = os.environ.get("CH347_LIB_PATH")
    if override:
        return override
    found = ctypes.util.find_library("ch347")
    if found:
        return found
    # find_library() can miss it if ldconfig hasn't been rerun since install; try the most
    # common manual-install spot directly before giving up.
    for candidate in ("/usr/lib/libch347.so", "/usr/local/lib/libch347.so"):
        if os.path.exists(candidate):
            return candidate
    raise OSError(
        "libch347.so not found. Install WCH's userspace library:\n\n"
        "  git clone https://github.com/WCHSoftGroup/ch341par_linux\n"
        "  sudo cp ch341par_linux/lib/x64/dynamic/libch347.so /usr/lib/ && sudo ldconfig\n\n"
        "(swap x64 for your architecture's subdirectory under lib/). Set CH347_LIB_PATH to "
        "point at a non-standard location instead of installing system-wide.")


class Ch347Device:
    """One open /dev/ch34x_pis* handle. SPI and I2C share this handle/interface (Mode 1 puts
    both on the same USB vendor interface) -- init whichever bus you're about to use, no need
    to init both."""

    def __init__(self, path: str):
        self._lib = ctypes.CDLL(_find_lib())
        self._declare_prototypes()
        self.fd = self._lib.CH347OpenDevice(path.encode())
        if self.fd < 0:
            raise OSError(f"CH347OpenDevice({path!r}) failed -- device busy, wrong path, or "
                           "insufficient permission (see the udev rule in tools/99-ch34x-pis.rules).")

    def _declare_prototypes(self) -> None:
        lib = self._lib
        lib.CH347OpenDevice.argtypes = [ctypes.c_char_p]
        lib.CH347OpenDevice.restype = ctypes.c_int
        lib.CH347CloseDevice.argtypes = [ctypes.c_int]
        lib.CH347CloseDevice.restype = ctypes.c_bool
        lib.CH347SPI_SetFrequency.argtypes = [ctypes.c_int, ctypes.c_uint32]
        lib.CH347SPI_SetFrequency.restype = ctypes.c_bool
        lib.CH347SPI_Init.argtypes = [ctypes.c_int, ctypes.POINTER(SpiConfig)]
        lib.CH347SPI_Init.restype = ctypes.c_bool
        lib.CH347SPI_WriteRead.argtypes = [ctypes.c_int, ctypes.c_bool, ctypes.c_uint8,
                                            ctypes.c_int, ctypes.c_void_p]
        lib.CH347SPI_WriteRead.restype = ctypes.c_bool
        lib.CH347I2C_Set.argtypes = [ctypes.c_int, ctypes.c_int]
        lib.CH347I2C_Set.restype = ctypes.c_bool
        lib.CH347StreamI2C.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_void_p,
                                        ctypes.c_int, ctypes.c_void_p]
        lib.CH347StreamI2C.restype = ctypes.c_bool

    def close(self) -> None:
        self._lib.CH347CloseDevice(self.fd)

    # -- SPI --------------------------------------------------------------------------------

    def spi_init(self, freq_hz: int, mode: int = 0, chip_select: int = 0x80) -> int:
        """Returns the actual frequency configured (snapped to a supported value)."""
        actual_hz = nearest_spi_frequency(freq_hz)
        if not self._lib.CH347SPI_SetFrequency(self.fd, ctypes.c_uint32(actual_hz)):
            raise OSError("CH347SPI_SetFrequency failed")
        cfg = SpiConfig(iMode=mode, iByteOrder=1, iSpiOutDefaultData=0xFF,
                        iChipSelect=chip_select)
        if not self._lib.CH347SPI_Init(self.fd, ctypes.byref(cfg)):
            raise OSError("CH347SPI_Init failed")
        return actual_hz

    def spi_write_read(self, data: bytes, chip_select: int = 0x80) -> bytes:
        """Full-duplex transfer: data written on MOSI, simultaneously-clocked MISO bytes
        returned (same semantics as spidev's xfer2 -- this is why a MOSI-MISO loopback jumper
        echoes the same bytes back here too)."""
        buf = ctypes.create_string_buffer(data, len(data))
        if not self._lib.CH347SPI_WriteRead(self.fd, False, chip_select, len(data), buf):
            raise OSError("CH347SPI_WriteRead failed")
        return buf.raw

    # -- I2C --------------------------------------------------------------------------------

    def i2c_set(self, mode: int = 0x01) -> None:
        """mode: bits 0-2 select the SCL rate (0x01 = 100kHz standard rate; see CH347I2C_Set's
        doc comment in ch347_lib.h for the full table)."""
        if not self._lib.CH347I2C_Set(self.fd, mode):
            raise OSError("CH347I2C_Set failed")

    def i2c_probe(self, addr7: int) -> bool:
        """True if a device ACKs at 7-bit address addr7. The write buffer's first byte is the
        address pre-shifted left with the R/W bit in bit0 (0 = write) -- this is the vendor
        lib's raw-stream convention, confirmed against a real call site in WCH's own
        ch347_demo_test.c (`ibuf[0] = 0xA0` for a 7-bit address of 0x50). Unlike smbus2's
        errno-based scan, a plain bool return here can't distinguish "no device answered"
        (expected, nothing attached) from a real bus error -- there's no equivalent of
        smbus2's ENXIO/EREMOTEIO to inspect through this API."""
        addr_byte = bytes([(addr7 << 1) & 0xFF])
        return bool(self._lib.CH347StreamI2C(self.fd, 1, addr_byte, 0, None))
