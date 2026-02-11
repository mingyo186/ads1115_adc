"""ADS1115 I2C Driver - 16-bit 4-channel ADC."""

import random
import struct
import time


class FakeADS1115Driver:
    """Fake driver that generates random voltage data without I2C hardware."""

    def __init__(self, **kwargs):
        pass

    def read_channel(self, channel: int) -> float:
        """Return voltage in Volts for the given single-ended channel."""
        base = {0: 1.65, 1: 2.50, 2: 0.80, 3: 3.30}
        return base.get(channel, 1.0) + random.gauss(0.0, 0.01)

    def close(self):
        pass


class ADS1115Driver:
    """Low-level I2C driver for Texas Instruments ADS1115.

    Datasheet: ADS1115 (SBAS444C)
    16-bit delta-sigma ADC with programmable gain amplifier and
    internal oscillator. Supports 4 single-ended or 2 differential
    input channels.
    """

    # ── Register Map ──────────────────────────────────────────────
    REG_CONVERSION = 0x00   # 16-bit conversion result
    REG_CONFIG     = 0x01   # 16-bit configuration

    # ── MUX settings for single-ended channels (AINx vs GND) ────
    MUX_SINGLE = {
        0: 0x4000,   # AIN0 - GND
        1: 0x5000,   # AIN1 - GND
        2: 0x6000,   # AIN2 - GND
        3: 0x7000,   # AIN3 - GND
    }

    # ── PGA full-scale voltage and config bits ───────────────────
    PGA_TABLE = {
        0: (0x0000, 6.144),    # ±6.144 V
        1: (0x0200, 4.096),    # ±4.096 V
        2: (0x0400, 2.048),    # ±2.048 V (default)
        3: (0x0600, 1.024),    # ±1.024 V
        4: (0x0800, 0.512),    # ±0.512 V
        5: (0x0A00, 0.256),    # ±0.256 V
    }

    # ── Data rate config bits ────────────────────────────────────
    DR_TABLE = {
        0: 0x0000,   #   8 SPS
        1: 0x0020,   #  16 SPS
        2: 0x0040,   #  32 SPS
        3: 0x0060,   #  64 SPS
        4: 0x0080,   # 128 SPS (default)
        5: 0x00A0,   # 250 SPS
        6: 0x00C0,   # 475 SPS
        7: 0x00E0,   # 860 SPS
    }

    def __init__(self, bus: int = 1, address: int = 0x48,
                 pga_gain: int = 2, data_rate: int = 4):
        from smbus2 import SMBus

        self.address = address
        self.bus = SMBus(bus)

        self._pga_bits = self.PGA_TABLE[pga_gain][0]
        self._fs_voltage = self.PGA_TABLE[pga_gain][1]
        self._dr_bits = self.DR_TABLE[data_rate]

    # ── Single-shot read ─────────────────────────────────────────
    def read_channel(self, channel: int) -> float:
        """Perform single-shot conversion on the given channel.

        Args:
            channel: 0-3 for single-ended AIN0-AIN3 vs GND

        Returns:
            voltage in Volts
        """
        mux = self.MUX_SINGLE[channel]

        # Config: OS=1 (start), MUX, PGA, MODE=1 (single-shot),
        #         DR, COMP_QUE=11 (disable comparator)
        config = 0x8000 | mux | self._pga_bits | 0x0100 | \
                 self._dr_bits | 0x0003

        # Write config to start conversion
        self.bus.write_i2c_block_data(
            self.address, self.REG_CONFIG,
            [(config >> 8) & 0xFF, config & 0xFF])

        # Wait for conversion complete (OS bit = 1)
        for _ in range(100):
            buf = self.bus.read_i2c_block_data(
                self.address, self.REG_CONFIG, 2)
            if buf[0] & 0x80:
                break
            time.sleep(0.001)

        # Read conversion result
        raw = self.bus.read_i2c_block_data(
            self.address, self.REG_CONVERSION, 2)
        value = struct.unpack('>h', bytes(raw))[0]

        # Convert to voltage
        return value * self._fs_voltage / 32767.0

    def close(self):
        self.bus.close()
