# SPDX-FileCopyrightText: 2025 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_tlv320`
================================================================================

CircuitPython driver for the TLV320DAC3100 I2S DAC


* Author(s): Liz Clark

Implementation Notes
--------------------

**Hardware:**

* `Link Text <url>`_

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

import time

from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const

try:
    from typing import Any, Dict, List, Literal, Optional, Tuple, TypedDict, Union, cast

    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_TLV320.git"

# Register addresses
_PAGE_SELECT = const(0x00)
_RESET = const(0x01)
_OT_FLAG = const(0x03)
_CLOCK_MUX1 = const(0x04)
_PLL_PROG_PR = const(0x05)
_PLL_PROG_J = const(0x06)
_PLL_PROG_D_MSB = const(0x07)
_PLL_PROG_D_LSB = const(0x08)
_NDAC = const(0x0B)
_MDAC = const(0x0C)
_DOSR_MSB = const(0x0D)
_DOSR_LSB = const(0x0E)
_CODEC_IF_CTRL1 = const(0x1B)
_DAC_FLAG = const(0x25)
_DAC_FLAG2 = const(0x26)
_INT1_CTRL = const(0x30)
_INT2_CTRL = const(0x31)
_GPIO1_CTRL = const(0x33)
_DIN_CTRL = const(0x36)
_DAC_DATAPATH = const(0x3F)
_DAC_VOL_CTRL = const(0x40)
_DAC_LVOL = const(0x41)
_DAC_RVOL = const(0x42)
_HEADSET_DETECT = const(0x43)
_VOL_ADC_CTRL = const(0x74)  # VOL/MICDET-Pin SAR ADC Control ister
_VOL_ADC_READ = const(0x75)  # VOL/MICDET-Pin Gain Register

# Page 1 registers
_HP_SPK_ERR_CTL = const(0x1E)
_HP_DRIVERS = const(0x1F)
_SPK_AMP = const(0x20)
_HP_POP = const(0x21)
_PGA_RAMP = const(0x22)
_OUT_ROUTING = const(0x23)
_HPL_VOL = const(0x24)
_HPR_VOL = const(0x25)
_SPK_VOL = const(0x26)
_HPL_DRIVER = const(0x28)
_HPR_DRIVER = const(0x29)
_SPK_DRIVER = const(0x2A)
_HP_DRIVER_CTRL = const(0x2C)
_MICBIAS = const(0x2E)  # MICBIAS Configuration ister
_INPUT_CM = const(0x32)  # Input Common Mode Settings Register

# Page 3 registers
_TIMER_MCLK_DIV = const(0x10)  # Timer Clock MCLK Divider Register

# Default I2C address
I2C_ADDR_DEFAULT = const(0x18)

# Data format for I2S interface
FORMAT_I2S = const(0b00)  # I2S format
FORMAT_DSP = const(0b01)  # DSP format
FORMAT_RJF = const(0b10)  # Right justified format
FORMAT_LJF = const(0b11)  # Left justified format

# Data length for I2S interface
DATA_LEN_16 = const(0b00)  # 16 bits
DATA_LEN_20 = const(0b01)  # 20 bits
DATA_LEN_24 = const(0b10)  # 24 bits
DATA_LEN_32 = const(0b11)  # 32 bits

# GPIO1 pin mode options
GPIO1_DISABLED = const(0b0000)  # GPIO1 disabled (input and output buffers powered down)
GPIO1_INPUT_MODE = const(0b0001)  # Input mode (secondary BCLK/WCLK/DIN input or ClockGen)
GPIO1_GPI = const(0b0010)  # General-purpose input
GPIO1_GPO = const(0b0011)  # General-purpose output
GPIO1_CLKOUT = const(0b0100)  # CLKOUT output
GPIO1_INT1 = const(0b0101)  # INT1 output
GPIO1_INT2 = const(0b0110)  # INT2 output
GPIO1_BCLK_OUT = const(0b1000)  # Secondary BCLK output for codec interface
GPIO1_WCLK_OUT = const(0b1001)  # Secondary WCLK output for codec interface

# DAC channel data path options
DAC_PATH_OFF = const(0b00)  # DAC data path off
DAC_PATH_NORMAL = const(0b01)  # Normal path (L->L or R->R)
DAC_PATH_SWAPPED = const(0b10)  # Swapped path (R->L or L->R)
DAC_PATH_MIXED = const(0b11)  # Mixed L+R path

# DAC volume control soft stepping options
VOLUME_STEP_1SAMPLE = const(0b00)  # One step per sample
VOLUME_STEP_2SAMPLE = const(0b01)  # One step per two samples
VOLUME_STEP_DISABLED = const(0b10)  # Soft stepping disabled

# DAC volume control configuration options
VOL_INDEPENDENT = const(0b00)  # Left and right channels independent
VOL_LEFT_TO_RIGHT = const(0b01)  # Left follows right volume
VOL_RIGHT_TO_LEFT = const(0b10)  # Right follows left volume

# DAC output routing options
DAC_ROUTE_NONE = const(0b00)  # DAC not routed
DAC_ROUTE_MIXER = const(0b01)  # DAC routed to mixer amplifier
DAC_ROUTE_HP = const(0b10)  # DAC routed directly to HP driver

# Speaker amplifier gain options
SPK_GAIN_6DB = const(0b00)  # 6 dB gain
SPK_GAIN_12DB = const(0b01)  # 12 dB gain
SPK_GAIN_18DB = const(0b10)  # 18 dB gain
SPK_GAIN_24DB = const(0b11)  # 24 dB gain

# Headphone common mode voltage settings
HP_COMMON_1_35V = const(0b00)  # Common-mode voltage 1.35V
HP_COMMON_1_50V = const(0b01)  # Common-mode voltage 1.50V
HP_COMMON_1_65V = const(0b10)  # Common-mode voltage 1.65V
HP_COMMON_1_80V = const(0b11)  # Common-mode voltage 1.80V

# Headset detection debounce time options
DEBOUNCE_16MS = const(0b000)  # 16ms debounce (2ms clock)
DEBOUNCE_32MS = const(0b001)  # 32ms debounce (4ms clock)
DEBOUNCE_64MS = const(0b010)  # 64ms debounce (8ms clock)
DEBOUNCE_128MS = const(0b011)  # 128ms debounce (16ms clock)
DEBOUNCE_256MS = const(0b100)  # 256ms debounce (32ms clock)
DEBOUNCE_512MS = const(0b101)  # 512ms debounce (64ms clock)

# Button press debounce time options
BTN_DEBOUNCE_0MS = const(0b00)  # No debounce
BTN_DEBOUNCE_8MS = const(0b01)  # 8ms debounce (1ms clock)
BTN_DEBOUNCE_16MS = const(0b10)  # 16ms debounce (2ms clock)
BTN_DEBOUNCE_32MS = const(0b11)  # 32ms debounce (4ms clock)

# ruff: noqa: PLR0904, PLR0912, PLR0913, PLR0915, PLR0917


class PagedRegisterBase:
    """Base class for paged register access."""

    def __init__(self, i2c_device, page):
        """Initialize the paged register base.

        :param i2c_device: The I2C device
        :param page: The register page number
        """
        self._device = i2c_device
        self._page = page
        self._buffer = bytearray(2)

    def _write_register(self, register, value):
        """Write a value to a register.

        :param register: The register address
        :param value: The value to write
        """
        self._set_page()
        self._buffer[0] = register
        self._buffer[1] = value
        with self._device as i2c:
            i2c.write(self._buffer)

    def _read_register(self, register):
        """Read a value from a register.

        :param register: The register address
        :return: The register value
        """
        self._set_page()
        self._buffer[0] = register
        with self._device as i2c:
            i2c.write(self._buffer, end=1)
            i2c.readinto(self._buffer, start=0, end=1)
        return self._buffer[0]

    def _set_page(self):
        """Set the current register page."""
        self._buffer[0] = _PAGE_SELECT
        self._buffer[1] = self._page
        with self._device as i2c:
            i2c.write(self._buffer)

    def _get_bits(self, register, mask, shift):
        """Read specific bits from a register.

        :param register: The register address
        :param mask: The bit mask (after shifting)
        :param shift: The bit position (0 = LSB)
        :return: The extracted bits
        """
        value = self._read_register(register)
        return (value >> shift) & mask

    def _set_bits(self, register, mask, shift, value):
        """Set specific bits in a register.

        :param register: The register address
        :param mask: The bit mask (after shifting)
        :param shift: The bit position (0 = LSB)
        :param value: The value to set
        """
        reg_value = self._read_register(register)
        reg_value &= ~(mask << shift)  # Clear the bits
        reg_value |= (value & mask) << shift  # Set the new bits
        self._write_register(register, reg_value)


class Page0Registers(PagedRegisterBase):
    """Page 0 registers containing system configuration, clocking, etc."""

    def __init__(self, i2c_device):
        """Initialize Page 0 registers.

        :param i2c_device: The I2C device
        """
        super().__init__(i2c_device, 0)

    def _reset(self):
        """Perform a software _reset of the chip.

        :return: True if successful, False otherwise
        """
        self._write_register(_RESET, 1)
        time.sleep(0.01)
        return self._read_register(_RESET) == 0

    def _is_overtemperature(self):
        """Check if the chip is in an over-temperature condition.

        :return: True if overtemp condition exists, False if temperature is OK
        """
        return not ((self._read_register(_OT_FLAG) >> 1) & 0x01)

    def _set_int1_source(
        self, headset_detect, button_press, dac_drc, agc_noise, over_current, multiple_pulse
    ):
        """Configure the INT1 interrupt sources."""
        value = 0
        if headset_detect:
            value |= 1 << 7
        if button_press:
            value |= 1 << 6
        if dac_drc:
            value |= 1 << 5
        if over_current:
            value |= 1 << 3
        if agc_noise:
            value |= 1 << 2
        if multiple_pulse:
            value |= 1 << 0
        self._write_register(_INT1_CTRL, value)

    def _set_gpio1_mode(self, mode):
        """Set the GPIO1 pin mode."""
        return self._set_bits(_GPIO1_CTRL, 0x0F, 2, mode)

    def _set_headset_detect(self, enable, detect_debounce=0, button_debounce=0):
        """Configure headset detection settings."""
        value = (1 if enable else 0) << 7
        value |= (detect_debounce & 0x07) << 2
        value |= button_debounce & 0x03
        self._write_register(_HEADSET_DETECT, value)

    def _set_dac_data_path(
        self,
        left_dac_on,
        right_dac_on,
        left_path=DAC_PATH_NORMAL,
        right_path=DAC_PATH_NORMAL,
        volume_step=VOLUME_STEP_1SAMPLE,
    ):
        """Configure the DAC data path settings."""
        value = 0
        if left_dac_on:
            value |= 1 << 7
        if right_dac_on:
            value |= 1 << 6
        value |= (left_path & 0x03) << 4
        value |= (right_path & 0x03) << 2
        value |= volume_step & 0x03
        self._write_register(_DAC_DATAPATH, value)

    def _set_dac_volume_control(self, left_mute, right_mute, control=VOL_INDEPENDENT):
        """Configure the DAC volume control settings."""
        value = 0
        if left_mute:
            value |= 1 << 3
        if right_mute:
            value |= 1 << 2
        value |= control & 0x03
        self._write_register(_DAC_VOL_CTRL, value)

    def _set_channel_volume(self, right_channel, db):
        """Set DAC channel volume in dB."""
        if db > 24.0:
            db = 24.0
        if db < -63.5:
            db = -63.5
        reg_val = int(db * 2)
        if reg_val == 0x80 or reg_val > 0x30:
            return False

        if right_channel:
            self._write_register(_DAC_RVOL, reg_val & 0xFF)
        else:
            self._write_register(_DAC_LVOL, reg_val & 0xFF)

    def _get_dac_flags(self):
        """Get the DAC and output driver status flags.

        :return: Dictionary with status flags for various components
        """
        flag_reg = self._read_register(_DAC_FLAG)
        left_dac_powered = bool(flag_reg & (1 << 7))
        hpl_powered = bool(flag_reg & (1 << 5))
        left_classd_powered = bool(flag_reg & (1 << 4))
        right_dac_powered = bool(flag_reg & (1 << 3))
        hpr_powered = bool(flag_reg & (1 << 1))
        right_classd_powered = bool(flag_reg & (1 << 0))
        flag2_reg = self._read_register(_DAC_FLAG2)
        left_pga_gain_ok = bool(flag2_reg & (1 << 4))
        right_pga_gain_ok = bool(flag2_reg & (1 << 0))

        return {
            "left_dac_powered": left_dac_powered,
            "hpl_powered": hpl_powered,
            "left_classd_powered": left_classd_powered,
            "right_dac_powered": right_dac_powered,
            "hpr_powered": hpr_powered,
            "right_classd_powered": right_classd_powered,
            "left_pga_gain_ok": left_pga_gain_ok,
            "right_pga_gain_ok": right_pga_gain_ok,
        }

    def get_gpio1_input(self):
        """Get the current GPIO1 input value.

        :return: Current GPIO1 input state (True/False)
        """
        return bool(self._get_bits(_GPIO1_CTRL, 0x01, 1))

    def _get_din_input(self):
        """Get the current DIN input value.

        :return: Current DIN input state (True/False)
        """
        return bool(self._get_bits(_DIN_CTRL, 0x01, 0))

    def _get_codec_interface(self):
        """Get the current codec interface settings.

        :return: Dictionary with format, data_len, bclk_out, and wclk_out values
        """
        reg_value = self._read_register(_CODEC_IF_CTRL1)
        format_val = (reg_value >> 6) & 0x03  # bits 7:6
        data_len = (reg_value >> 4) & 0x03  # bits 5:4
        bclk_out = bool(reg_value & (1 << 3))  # bit 3
        wclk_out = bool(reg_value & (1 << 2))  # bit 2

        return {
            "format": format_val,
            "data_len": data_len,
            "bclk_out": bclk_out,
            "wclk_out": wclk_out,
        }

    def _get_dac_data_path(self):
        """Get the current DAC data path configuration.

        :return: Dictionary with DAC data path settings
        """
        reg_value = self._read_register(_DAC_DATAPATH)
        left_dac_on = bool(reg_value & (1 << 7))  # bit 7
        right_dac_on = bool(reg_value & (1 << 6))  # bit 6
        left_path = (reg_value >> 4) & 0x03  # bits 5:4
        right_path = (reg_value >> 2) & 0x03  # bits 3:2
        volume_step = reg_value & 0x03  # bits 1:0

        return {
            "left_dac_on": left_dac_on,
            "right_dac_on": right_dac_on,
            "left_path": left_path,
            "right_path": right_path,
            "volume_step": volume_step,
        }

    def _get_dac_volume_control(self):
        """Get the current DAC volume control configuration.

        :return: Dictionary with volume control settings
        """
        reg_value = self._read_register(_DAC_VOL_CTRL)
        left_mute = bool(reg_value & (1 << 3))  # bit 3
        right_mute = bool(reg_value & (1 << 2))  # bit 2
        control = reg_value & 0x03  # bits 1:0

        return {"left_mute": left_mute, "right_mute": right_mute, "control": control}

    def _get_channel_volume(self, right_channel):
        """Get DAC channel volume in dB.

        :param right_channel: True for right channel, False for left channel
        :return: Current volume in dB
        """
        reg = _DAC_RVOL if right_channel else _DAC_LVOL
        reg_val = self._read_register(reg)
        if reg_val & 0x80:
            steps = reg_val - 256
        else:
            steps = reg_val
        return steps * 0.5

    def _get_headset_status(self):
        """Get current headset detection status.

        :return: Integer value representing headset status (0=none, 1=without mic, 3=with mic)
        """
        status_bits = self._get_bits(_HEADSET_DETECT, 0x03, 5)
        return status_bits

    def _config_vol_adc(self, pin_control=False, use_mclk=False, hysteresis=0, rate=0):
        """Configure the Volume/MicDet pin ADC.

        :param pin_control: Enable pin control of DAC volume
        :param use_mclk: Use MCLK instead of internal RC oscillator
        :param hysteresis: ADC hysteresis setting (0-2)
        :param rate: ADC sampling rate (0-7)
        """
        value = (1 if pin_control else 0) << 7
        value |= (1 if use_mclk else 0) << 6
        value |= (hysteresis & 0x03) << 4
        value |= rate & 0x07
        self._write_register(_VOL_ADC_CTRL, value)

    def _read_vol_adc_db(self):
        """Read the current volume from the Volume ADC in dB.

        :return: Current volume in dB (+18 to -63 dB)
        """
        raw_val = self._read_register(_VOL_ADC_READ) & 0x7F
        if raw_val == 0x7F:
            return 0.0
        if raw_val <= 0x24:
            return 18.0 - (raw_val * 0.5)
        else:
            return -((raw_val - 0x24) * 0.5)

    def _set_int2_source(
        self,
        headset_detect=False,
        button_press=False,
        dac_drc=False,
        agc_noise=False,
        over_current=False,
        multiple_pulse=False,
    ):
        """Configure the INT2 interrupt sources.

        :param headset_detect: Enable headset detection interrupt
        :param button_press: Enable button press detection interrupt
        :param dac_drc: Enable DAC DRC signal power interrupt
        :param agc_noise: Enable DAC data overflow interrupt
        :param over_current: Enable short circuit interrupt
        :param multiple_pulse: If true, INT2 generates multiple pulses until flag read
        """
        value = 0
        if headset_detect:
            value |= 1 << 7
        if button_press:
            value |= 1 << 6
        if dac_drc:
            value |= 1 << 5
        if over_current:
            value |= 1 << 3
        if agc_noise:
            value |= 1 << 2
        if multiple_pulse:
            value |= 1 << 0

        self._write_register(_INT2_CTRL, value)

    def _set_codec_interface(self, format, data_len, bclk_out=False, wclk_out=False):
        """Set the codec interface parameters."""
        value = (format & 0x03) << 6
        value |= (data_len & 0x03) << 4
        value |= (1 if bclk_out else 0) << 3
        value |= (1 if wclk_out else 0) << 2

        self._write_register(_CODEC_IF_CTRL1, value)

    def _configure_clocks_for_sample_rate(self, mclk_freq: int, sample_rate: int, bit_depth: int):
        """Configure clock settings for the specified sample rate.

        :param mclk_freq: The main clock frequency in Hz, or 0 to use BCLK as PLL input
        :param sample_rate: The desired sample rate in Hz
        :param bit_depth: The bit depth (16, 20, 24, or 32)
        :return: True if successful, False otherwise
        """
        if bit_depth == 16:
            data_len = DATA_LEN_16
        elif bit_depth == 20:
            data_len = DATA_LEN_20
        elif bit_depth == 24:
            data_len = DATA_LEN_24
        else:
            data_len = DATA_LEN_32
        if mclk_freq == 0:
            self._set_codec_interface(FORMAT_I2S, data_len)
            self._set_bits(_CLOCK_MUX1, 0x03, 2, 0b01)
            self._set_bits(_CLOCK_MUX1, 0x03, 0, 0b11)
            p, r, j, d = 1, 2, 32, 0
            ndac = 8
            mdac = 2
            dosr = 128
            pr_value = ((p & 0x07) << 4) | (r & 0x0F)
            self._write_register(_PLL_PROG_PR, pr_value & 0x7F)
            self._write_register(_PLL_PROG_J, j & 0x3F)
            self._write_register(_PLL_PROG_D_MSB, (d >> 8) & 0xFF)
            self._write_register(_PLL_PROG_D_LSB, d & 0xFF)
            self._write_register(_NDAC, 0x80 | (ndac & 0x7F))
            self._write_register(_MDAC, 0x80 | (mdac & 0x7F))
            self._write_register(_DOSR_MSB, (dosr >> 8) & 0xFF)
            self._write_register(_DOSR_LSB, dosr & 0xFF)
            self._set_bits(_PLL_PROG_PR, 0x01, 7, 1)
            time.sleep(0.01)
        elif mclk_freq % (128 * sample_rate) == 0:
            div_ratio = mclk_freq // (128 * sample_rate)
            self._set_bits(_CLOCK_MUX1, 0x03, 0, 0b00)
            self._set_bits(_PLL_PROG_PR, 0x01, 7, 0)
            if div_ratio <= 128:
                self._write_register(_NDAC, 0x80 | (div_ratio & 0x7F))
                self._write_register(_MDAC, 0x81)
                self._write_register(_DOSR_MSB, 0)
                self._write_register(_DOSR_LSB, 128)
                self._set_codec_interface(FORMAT_I2S, data_len)
        elif mclk_freq == 12000000:
            if sample_rate == 22050:
                p, r, j, d = 1, 1, 7, 6144
                ndac = 8
                mdac = 1
                dosr = 128
            elif sample_rate == 44100:
                p, r, j, d = 1, 1, 7, 6144
                ndac = 4
                mdac = 1
                dosr = 128
            elif sample_rate == 48000:
                p, r, j, d = 1, 1, 8, 0
                ndac = 4
                mdac = 1
                dosr = 128
            elif sample_rate == 96000:
                p, r, j, d = 1, 1, 8, 0
                ndac = 2
                mdac = 1
                dosr = 128
            else:
                raise ValueError("Need a valid sample rate: 22050, 44100, 48000 or 96000")
        elif mclk_freq == 24000000:
            if sample_rate == 44100:
                p, r, j, d = 1, 2, 7, 6144
                ndac = 4
                mdac = 1
                dosr = 128
            elif sample_rate == 48000:
                p, r, j, d = 1, 2, 8, 0
                ndac = 4
                mdac = 1
                dosr = 128
            elif sample_rate == 96000:
                p, r, j, d = 1, 2, 8, 0
                ndac = 2
                mdac = 1
                dosr = 128
            else:
                raise ValueError("Need a valid sample rate: 44100, 48000 or 96000")
        else:
            raise ValueError("Need a valid MCLK frequency: 12MHz or 24MHz")
        if mclk_freq != 0:
            self._set_bits(_CLOCK_MUX1, 0x03, 2, 0b00)
            self._set_bits(_CLOCK_MUX1, 0x03, 0, 0b11)
            pr_value = ((p & 0x07) << 4) | (r & 0x0F)
            self._write_register(_PLL_PROG_PR, pr_value & 0x7F)
            self._write_register(_PLL_PROG_J, j & 0x3F)
            self._write_register(_PLL_PROG_D_MSB, (d >> 8) & 0xFF)
            self._write_register(_PLL_PROG_D_LSB, d & 0xFF)
            self._write_register(_NDAC, 0x80 | (ndac & 0x7F))
            self._write_register(_MDAC, 0x80 | (mdac & 0x7F))
            self._write_register(_DOSR_MSB, (dosr >> 8) & 0xFF)
            self._write_register(_DOSR_LSB, dosr & 0xFF)
            self._set_codec_interface(FORMAT_I2S, data_len)
            self._set_bits(_PLL_PROG_PR, 0x01, 7, 1)
            time.sleep(0.01)


class Page1Registers(PagedRegisterBase):
    """Page 1 registers containing analog output settings, HP/SPK controls, etc."""

    def __init__(self, i2c_device):
        """Initialize Page 1 registers.

        :param i2c_device: The I2C device
        """
        super().__init__(i2c_device, 1)

    def _get_speaker_enabled(self):
        """Check if speaker is enabled."""
        return bool(self._get_bits(_SPK_AMP, 0x01, 7))

    def _set_speaker_enabled(self, enable):
        """Enable or disable the Class-D speaker amplifier."""
        return self._set_bits(_SPK_AMP, 0x01, 7, 1 if enable else 0)

    def _configure_headphone_driver(
        self, left_powered, right_powered, common=HP_COMMON_1_35V, power_down_on_scd=False
    ):
        """Configure headphone driver settings."""
        value = 0x04  # bit 2 must be 1
        if left_powered:
            value |= 1 << 7
        if right_powered:
            value |= 1 << 6
        value |= (common & 0x03) << 3
        if power_down_on_scd:
            value |= 1 << 1

        self._write_register(_HP_DRIVERS, value)

    def _configure_analog_inputs(
        self,
        left_dac=DAC_ROUTE_NONE,
        right_dac=DAC_ROUTE_NONE,
        left_ain1=False,
        left_ain2=False,
        right_ain2=False,
        hpl_routed_to_hpr=False,
    ):
        """Configure DAC and analog input routing."""
        value = 0
        value |= (left_dac & 0x03) << 6
        if left_ain1:
            value |= 1 << 5
        if left_ain2:
            value |= 1 << 4
        value |= (right_dac & 0x03) << 2
        if right_ain2:
            value |= 1 << 1
        if hpl_routed_to_hpr:
            value |= 1

        self._write_register(_OUT_ROUTING, value)

    def _set_hpl_volume(self, route_enabled, gain=0x7F):
        """Set HPL analog volume control."""
        if gain > 0x7F:
            gain = 0x7F
        value = ((1 if route_enabled else 0) << 7) | (gain & 0x7F)
        self._write_register(_HPL_VOL, value)

    def _set_hpr_volume(self, route_enabled, gain=0x7F):
        """Set HPR analog volume control."""
        if gain > 0x7F:
            gain = 0x7F
        value = ((1 if route_enabled else 0) << 7) | (gain & 0x7F)
        self._write_register(_HPR_VOL, value)

    def _set_spk_volume(self, route_enabled, gain=0x7F):
        """Set Speaker analog volume control."""
        if gain > 0x7F:
            gain = 0x7F
        value = ((1 if route_enabled else 0) << 7) | (gain & 0x7F)
        self._write_register(_SPK_VOL, value)

    def _configure_hpl_pga(self, gain_db=0, unmute=True):
        """Configure HPL driver PGA settings."""
        if gain_db > 9:
            raise ValueError("Gain cannot be greater than 9")
        value = (gain_db & 0x0F) << 3
        if unmute:
            value |= 1 << 2
        self._write_register(_HPL_DRIVER, value)

    def _configure_hpr_pga(self, gain_db=0, unmute=True):
        """Configure HPR driver PGA settings."""
        if gain_db > 9:
            raise ValueError("Gain cannot be greater than 9")
        value = (gain_db & 0x0F) << 3
        if unmute:
            value |= 1 << 2
        self._write_register(_HPR_DRIVER, value)

    def _configure_spk_pga(self, gain=SPK_GAIN_6DB, unmute=True):
        """Configure Speaker driver settings."""
        value = (gain & 0x03) << 3
        if unmute:
            value |= 1 << 2
        self._write_register(_SPK_DRIVER, value)

    def _is_speaker_shorted(self):
        """Check if speaker short circuit is detected.

        :return: True if short circuit detected, False if not
        """
        return bool(self._get_bits(_SPK_AMP, 0x01, 0))

    def _is_hpl_gain_applied(self):
        """Check if all programmed gains have been applied to HPL.

        :return: True if gains applied, False if still ramping
        """
        return bool(self._get_bits(_HPL_DRIVER, 0x01, 0))

    def _is_hpr_gain_applied(self):
        """Check if all programmed gains have been applied to HPR.

        :return: True if gains applied, False if still ramping
        """
        return bool(self._get_bits(_HPR_DRIVER, 0x01, 0))

    def _is_spk_gain_applied(self):
        """Check if all programmed gains have been applied to Speaker.

        :return: True if gains applied, False if still ramping
        """
        return bool(self._get_bits(_SPK_DRIVER, 0x01, 0))

    def _reset_speaker_on_scd(self, reset):
        """Configure speaker reset behavior on short circuit detection.

        :param reset: True to reset speaker on short circuit, False to remain unchanged
        :return: True if successful
        """
        return self._set_bits(_HP_SPK_ERR_CTL, 0x01, 1, 0 if reset else 1)

    def _reset_headphone_on_scd(self, reset):
        """Configure headphone reset behavior on short circuit detection.

        :param reset: True to reset headphone on short circuit, False to remain unchanged
        :return: True if successful
        """
        # Register is inverse of parameter (0 = reset, 1 = no reset)
        return self._set_bits(_HP_SPK_ERR_CTL, 0x01, 0, 0 if reset else 1)

    def _configure_headphone_pop(self, wait_for_powerdown=True, powerup_time=0x07, ramp_time=0x03):
        """Configure headphone pop removal settings.

        :param wait_for_powerdown: Wait for amp powerdown before DAC powerdown
        :param powerup_time: Driver power-on time (0-11)
        :param ramp_time: Driver ramp-up step time (0-3)
        :return: True if successful
        """
        value = (1 if wait_for_powerdown else 0) << 7
        value |= (powerup_time & 0x0F) << 3
        value |= (ramp_time & 0x03) << 1
        self._write_register(_HP_POP, value)

    def _set_speaker_wait_time(self, wait_time=0):
        """Set speaker power-up wait time.

        :param wait_time: Speaker power-up wait duration (0-7)
        :return: True if successful
        """
        return self._set_bits(_PGA_RAMP, 0x07, 4, wait_time)

    def _headphone_lineout(self, left, right):
        """Configure headphone outputs as line-out.

        :param left: Configure left channel as line-out
        :param right: Configure right channel as line-out
        :return: True if successful
        """
        value = 0
        if left:
            value |= 1 << 2
        if right:
            value |= 1 << 1
        self._write_register(_HP_DRIVER_CTRL, value)

    def _config_mic_bias(self, power_down=False, always_on=False, voltage=0):
        """Configure MICBIAS settings."""
        value = (1 if power_down else 0) << 7
        value |= (1 if always_on else 0) << 3
        value |= voltage & 0x03
        self._write_register(_MICBIAS, value)  # Using constant instead of 0x2E

    def _set_input_common_mode(self, ain1_cm, ain2_cm):
        """Set analog input common mode connections."""
        value = 0
        if ain1_cm:
            value |= 1 << 7
        if ain2_cm:
            value |= 1 << 6
        self._write_register(_INPUT_CM, value)  # Using constant instead of 0x32


class Page3Registers(PagedRegisterBase):
    """Page 3 registers containing timer settings."""

    def __init__(self, i2c_device):
        """Initialize Page 3 registers.

        :param i2c_device: The I2C device
        """
        super().__init__(i2c_device, 3)

    def _config_delay_divider(self, use_mclk=True, divider=1):
        """Configure programmable delay timer clock source and divider."""
        value = (1 if use_mclk else 0) << 7
        value |= divider & 0x7F
        self._write_register(_TIMER_MCLK_DIV, value)


class TLV320DAC3100:
    """Driver for the TI TLV320DAC3100 Stereo DAC with Headphone Amplifier."""

    def __init__(self, i2c: I2C, address: int = 0x18) -> None:
        """Initialize the TLV320DAC3100.

        :param i2c: The I2C bus the device is connected to
        :param address: The I2C device address (default is 0x18)
        """
        self._device: I2CDevice = I2CDevice(i2c, address)

        # Initialize register page classes
        self._page0: "Page0Registers" = Page0Registers(self._device)
        self._page1: "Page1Registers" = Page1Registers(self._device)
        self._page3: "Page3Registers" = Page3Registers(self._device)

        # Initialize configuration tracking variables
        self._sample_rate: int = 44100  # Default
        self._bit_depth: int = 16  # Default
        self._mclk_freq: int = 0  # Default blck

        # Reset the device
        if not self.reset():
            raise RuntimeError("Failed to reset TLV320DAC3100")

    # Basic properties and methods

    def reset(self) -> bool:
        """Reset the device.

        :return: True if reset successful, False otherwise
        """
        return self._page0._reset()

    @property
    def overtemperature(self) -> bool:
        """Check if the chip is overheating.

        :return: True if overtemperature condition exists, False otherwise
        """
        return self._page0._is_overtemperature()

    def set_headset_detect(
        self, enable: bool, detect_debounce: int = 0, button_debounce: int = 0
    ) -> bool:
        """Configure headset detection settings.

        :param enable: Boolean to enable/disable headset detection
        :param detect_debounce: One of the DEBOUNCE_* constants for headset detect
        :param button_debounce: One of the BTN_DEBOUNCE_* constants for button press
        :raises ValueError: If debounce values are not valid constants
        :return: True if successful, False otherwise
        """
        valid_detect_debounce: List[int] = [
            DEBOUNCE_16MS,
            DEBOUNCE_32MS,
            DEBOUNCE_64MS,
            DEBOUNCE_128MS,
            DEBOUNCE_256MS,
            DEBOUNCE_512MS,
        ]
        valid_button_debounce: List[int] = [
            BTN_DEBOUNCE_0MS,
            BTN_DEBOUNCE_8MS,
            BTN_DEBOUNCE_16MS,
            BTN_DEBOUNCE_32MS,
        ]

        if detect_debounce not in valid_detect_debounce:
            raise ValueError(
                f"Invalid detect_debounce value: {detect_debounce}."
                + "Must be one of the DEBOUNCE_* constants."
            )

        if button_debounce not in valid_button_debounce:
            raise ValueError(
                f"Invalid button_debounce value: {button_debounce}."
                + "Must be one of the BTN_DEBOUNCE_* constants."
            )

        return self._page0._set_headset_detect(enable, detect_debounce, button_debounce)

    def int1_source(
        self,
        headset_detect: bool,
        button_press: bool,
        dac_drc: bool,
        agc_noise: bool,
        over_current: bool,
        multiple_pulse: bool,
    ) -> bool:
        """Configure the INT1 interrupt sources.

        :param headset_detect: Enable headset detection interrupt
        :param button_press: Enable button press detection interrupt
        :param dac_drc: Enable DAC DRC signal power interrupt
        :param agc_noise: Enable DAC data overflow interrupt
        :param over_current: Enable short circuit interrupt
        :param multiple_pulse: If true, INT1 generates multiple pulses until flag read
        :return: True if successful, False otherwise
        """
        return self._page0._set_int1_source(
            headset_detect, button_press, dac_drc, agc_noise, over_current, multiple_pulse
        )

    @property
    def left_dac(self) -> bool:
        """Get the left DAC enabled status.

        :return: True if left DAC is enabled, False otherwise
        """
        return self._page0._get_dac_data_path()["left_dac_on"]

    @left_dac.setter
    def left_dac(self, enabled: bool) -> None:
        """Set the left DAC enabled status.

        :param enabled: True to enable left DAC, False to disable
        """
        current: DACDataPath = self._page0._get_dac_data_path()
        self._page0._set_dac_data_path(
            enabled,
            current["right_dac_on"],
            current["left_path"],
            current["right_path"],
            current["volume_step"],
        )

    @property
    def right_dac(self) -> bool:
        """Get the right DAC enabled status.

        :return: True if right DAC is enabled, False otherwise
        """
        return self._page0._get_dac_data_path()["right_dac_on"]

    @right_dac.setter
    def right_dac(self, enabled: bool) -> None:
        """Set the right DAC enabled status.

        :param enabled: True to enable right DAC, False to disable
        """
        current: DACDataPath = self._page0._get_dac_data_path()
        self._page0._set_dac_data_path(
            current["left_dac_on"],
            enabled,
            current["left_path"],
            current["right_path"],
            current["volume_step"],
        )

    @property
    def left_dac_path(self) -> int:
        """Get the left DAC path setting.

        :return: One of the DAC_PATH_* constants
        """
        return self._page0._get_dac_data_path()["left_path"]

    @left_dac_path.setter
    def left_dac_path(self, path: int) -> None:
        """Set the left DAC path.

        :param path: One of the DAC_PATH_* constants
        :raises ValueError: If path is not a valid DAC_PATH_* constant
        """
        valid_paths: List[int] = [DAC_PATH_OFF, DAC_PATH_NORMAL, DAC_PATH_SWAPPED, DAC_PATH_MIXED]

        if path not in valid_paths:
            raise ValueError(
                f"Invalid DAC path value: {path}. Must be one of the DAC_PATH_* constants."
            )

        current: DACDataPath = self._page0._get_dac_data_path()
        self._page0._set_dac_data_path(
            current["left_dac_on"],
            current["right_dac_on"],
            path,
            current["right_path"],
            current["volume_step"],
        )

    @property
    def right_dac_path(self) -> int:
        """Get the right DAC path setting.

        :return: One of the DAC_PATH_* constants
        """
        return self._page0._get_dac_data_path()["right_path"]

    @right_dac_path.setter
    def right_dac_path(self, path: int) -> None:
        """Set the right DAC path.

        :param path: One of the DAC_PATH_* constants
        :raises ValueError: If path is not a valid DAC_PATH_* constant
        """
        valid_paths: List[int] = [DAC_PATH_OFF, DAC_PATH_NORMAL, DAC_PATH_SWAPPED, DAC_PATH_MIXED]

        if path not in valid_paths:
            raise ValueError(
                f"Invalid DAC path value: {path}. Must be one of the DAC_PATH_* constants."
            )

        current: DACDataPath = self._page0._get_dac_data_path()
        self._page0._set_dac_data_path(
            current["left_dac_on"],
            current["right_dac_on"],
            current["left_path"],
            path,
            current["volume_step"],
        )

    @property
    def dac_volume_step(self) -> int:
        """Get the DAC volume step setting.

        :return: One of the VOLUME_STEP_* constants
        """
        return self._page0._get_dac_data_path()["volume_step"]

    @dac_volume_step.setter
    def dac_volume_step(self, step: int) -> None:
        """Set the DAC volume step setting.

        :param step: One of the VOLUME_STEP_* constants
        :raises ValueError: If step is not a valid VOLUME_STEP_* constant
        """
        valid_steps: List[int] = [VOLUME_STEP_1SAMPLE, VOLUME_STEP_2SAMPLE, VOLUME_STEP_DISABLED]

        if step not in valid_steps:
            raise ValueError(
                f"Invalid volume step value: {step}. Must be one of the VOLUME_STEP_* constants."
            )

        current: DACDataPath = self._page0._get_dac_data_path()
        self._page0._set_dac_data_path(
            current["left_dac_on"],
            current["right_dac_on"],
            current["left_path"],
            current["right_path"],
            step,
        )

    def configure_analog_inputs(
        self,
        left_dac: int = 0,
        right_dac: int = 0,
        left_ain1: bool = False,
        left_ain2: bool = False,
        right_ain2: bool = False,
        hpl_routed_to_hpr: bool = False,
    ) -> bool:
        """Configure DAC and analog input routing.

        :param left_dac: One of the DAC_ROUTE_* constants for left DAC routing
        :param right_dac: One of the DAC_ROUTE_* constants for right DAC routing
        :param left_ain1: Boolean to route left AIN1 to output
        :param left_ain2: Boolean to route left AIN2 to output
        :param right_ain2: Boolean to route right AIN2 to output
        :param hpl_routed_to_hpr: Boolean to route HPL to HPR
        :raises ValueError: If DAC route values are not valid constants
        :return: True if successful, False otherwise
        """
        valid_dac_routes: List[int] = [DAC_ROUTE_NONE, DAC_ROUTE_MIXER, DAC_ROUTE_HP]

        if left_dac not in valid_dac_routes:
            raise ValueError(
                f"Invalid left_dac value: {left_dac}. Must be one of the DAC_ROUTE_* constants."
            )

        if right_dac not in valid_dac_routes:
            raise ValueError(
                f"Invalid right_dac value: {right_dac}. Must be one of the DAC_ROUTE_* constants."
            )

        return self._page1._configure_analog_inputs(
            left_dac, right_dac, left_ain1, left_ain2, right_ain2, hpl_routed_to_hpr
        )

    @property
    def left_dac_mute(self) -> bool:
        """Get the left DAC mute status.

        :return: True if left DAC is muted, False otherwise
        """
        return self._page0._get_dac_volume_control()["left_mute"]

    @left_dac_mute.setter
    def left_dac_mute(self, mute: bool) -> None:
        """Set the left DAC mute status.

        :param mute: True to mute left DAC, False to unmute
        """
        current: DACVolumeControl = self._page0._get_dac_volume_control()
        self._page0._set_dac_volume_control(mute, current["right_mute"], current["control"])

    @property
    def right_dac_mute(self) -> bool:
        """Get the right DAC mute status.

        :return: True if right DAC is muted, False otherwise
        """
        return self._page0._get_dac_volume_control()["right_mute"]

    @right_dac_mute.setter
    def right_dac_mute(self, mute: bool) -> None:
        """Set the right DAC mute status.

        :param mute: True to mute right DAC, False to unmute
        """
        current: DACVolumeControl = self._page0._get_dac_volume_control()
        self._page0._set_dac_volume_control(current["left_mute"], mute, current["control"])

    @property
    def dac_volume_control_mode(self) -> int:
        """Get the DAC volume control mode.

        :return: One of the VOL_* constants
        """
        return self._page0._get_dac_volume_control()["control"]

    @dac_volume_control_mode.setter
    def dac_volume_control_mode(self, mode: int) -> None:
        """Set the volume control mode.

        :param mode: One of the VOL_* constants for volume control mode
        :raises ValueError: If mode is not a valid VOL_* constant
        """
        valid_modes: List[int] = [VOL_INDEPENDENT, VOL_LEFT_TO_RIGHT, VOL_RIGHT_TO_LEFT]
        if mode not in valid_modes:
            raise ValueError(
                f"Invalid volume control mode: {mode}. Must be one of the VOL_* constants."
            )
        current: DACVolumeControl = self._page0._get_dac_volume_control()
        self._page0._set_dac_volume_control(current["left_mute"], current["right_mute"], mode)

    @property
    def left_dac_channel_volume(self) -> float:
        """Get left DAC channel volume in dB.

        :return: Volume in dB
        """
        return self._page0._get_channel_volume(False)

    @left_dac_channel_volume.setter
    def left_dac_channel_volume(self, db: float) -> None:
        """Set left DAC channel volume in dB.

        :param db: Volume in dB
        """
        self._page0._set_channel_volume(False, db)

    @property
    def right_dac_channel_volume(self) -> float:
        """Get right DAC channel volume in dB.

        :return: Volume in dB
        """
        return self._page0._get_channel_volume(True)

    @right_dac_channel_volume.setter
    def right_dac_channel_volume(self, db: float) -> None:
        """Set right DAC channel volume in dB.

        :param db: Volume in dB
        """
        self._page0._set_channel_volume(True, db)

    def manual_headphone_driver(
        self,
        left_powered: bool,
        right_powered: bool,
        common: int = 0,
        power_down_on_scd: bool = False,
    ) -> bool:
        """Configure headphone driver settings.

        :param left_powered: Boolean to power left headphone driver
        :param right_powered: Boolean to power right headphone driver
        :param common: One of the HP_COMMON_* constants for common mode voltage
        :param power_down_on_scd: Boolean to power down on short circuit detection
        :raises ValueError: If common is not a valid HP_COMMON_* constant
        :return: True if successful, False otherwise
        """
        valid_common_modes: List[int] = [
            HP_COMMON_1_35V,
            HP_COMMON_1_50V,
            HP_COMMON_1_65V,
            HP_COMMON_1_80V,
        ]

        if common not in valid_common_modes:
            raise ValueError(
                f"Invalid common mode value: {common}. Must be one of the HP_COMMON_* constants."
            )

        return self._page1._configure_headphone_driver(
            left_powered, right_powered, common, power_down_on_scd
        )

    def manual_headphone_left_volume(self, route_enabled: bool, gain: int = 0x7F) -> bool:
        """Set HPL analog volume control.

        :param route_enabled: Enable routing to HPL
        :param gain: Analog volume control value (0-127)
        :return: True if successful, False otherwise
        """
        return self._page1._set_hpl_volume(route_enabled, gain)

    def manual_headphone_right_volume(self, route_enabled: bool, gain: int = 0x7F) -> bool:
        """Set HPR analog volume control.

        :param route_enabled: Enable routing to HPR
        :param gain: Analog volume control value (0-127)
        :return: True if successful, False otherwise
        """
        return self._page1._set_hpr_volume(route_enabled, gain)

    @property
    def headphone_left_gain(self) -> int:
        """Get the left headphone gain in dB.

        :return: Gain value in dB
        """
        reg_value = self._page1._read_register(_HPL_DRIVER)
        return (reg_value >> 3) & 0x0F

    @headphone_left_gain.setter
    def headphone_left_gain(self, gain_db: int) -> None:
        """Set the left headphone gain in dB.

        :param gain_db: Gain value in dB
        """
        unmute = not self.headphone_left_mute
        self._page1._configure_hpl_pga(gain_db, unmute)

    @property
    def headphone_left_mute(self) -> bool:
        """Get the left headphone mute status.

        :return: True if left headphone is muted, False otherwise
        """
        reg_value = self._page1._read_register(_HPL_DRIVER)
        return not bool(reg_value & (1 << 2))

    @headphone_left_mute.setter
    def headphone_left_mute(self, mute: bool) -> None:
        """Set the left headphone mute status.

        :param mute: True to mute left headphone, False to unmute
        """
        gain = self.headphone_left_gain
        self._page1._configure_hpl_pga(gain, not mute)

    @property
    def headphone_right_gain(self) -> int:
        """Get the right headphone gain in dB.

        :return: Gain value in dB
        """
        reg_value = self._page1._read_register(_HPR_DRIVER)
        return (reg_value >> 3) & 0x0F

    @headphone_right_gain.setter
    def headphone_right_gain(self, gain_db: int) -> None:
        """Set the right headphone gain in dB.

        :param gain_db: Gain value in dB
        """
        unmute = not self.headphone_right_mute
        self._page1._configure_hpr_pga(gain_db, unmute)

    @property
    def headphone_right_mute(self) -> bool:
        """Get the right headphone mute status.

        :return: True if right headphone is muted, False otherwise
        """
        reg_value = self._page1._read_register(_HPR_DRIVER)
        return not bool(reg_value & (1 << 2))

    @headphone_right_mute.setter
    def headphone_right_mute(self, mute: bool) -> None:
        """Set the right headphone mute status.

        :param mute: True to mute right headphone, False to unmute
        """
        gain = self.headphone_right_gain
        self._page1._configure_hpr_pga(gain, not mute)

    @property
    def speaker_gain(self) -> int:
        """Get the speaker gain setting in dB.

        :return: The gain value in dB
        """
        reg_value = self._page1._read_register(_SPK_DRIVER)
        return (reg_value >> 3) & 0x03

    @speaker_gain.setter
    def speaker_gain(self, gain_db: int) -> None:
        """Set the speaker gain in dB.

        :param gain_db: Speaker gain in dB (6, 12, 18, or 24)
        :raises ValueError: If gain_db is not a valid value
        """
        # Convert dB to register value
        gain_mapping: List[int] = [SPK_GAIN_6DB, SPK_GAIN_12DB, SPK_GAIN_18DB, SPK_GAIN_24DB]

        if gain_db not in gain_mapping:
            raise ValueError(
                f"Invalid preset value: {gain_db}. Must be one of the SPK_GAIN_* constants."
            )
        unmute = not self.speaker_mute
        self._page1._configure_spk_pga(gain_db, unmute)

    @property
    def speaker_mute(self) -> bool:
        """Get the speaker mute status.

        :return: True if speaker is muted, False otherwise
        """
        reg_value = self._page1._read_register(_SPK_DRIVER)
        return not bool(reg_value & (1 << 2))

    @speaker_mute.setter
    def speaker_mute(self, mute: bool) -> None:
        """Set the speaker mute status.

        :param mute: True to mute speaker, False to unmute
        """
        gain = self.speaker_gain
        # Unmute is inverse of mute
        self._page1._configure_spk_pga(gain, not mute)

    @property
    def dac_flags(self) -> Dict[str, Any]:
        """Get the DAC and output driver status flags.

        :return: Dictionary with status flags
        """
        return self._page0._get_dac_flags()

    @property
    def gpio1_mode(self) -> int:
        """Get the current GPIO1 pin mode.

        :return: One of the GPIO1_* mode constants
        """
        value = self._page0._read_register(_GPIO1_CTRL)
        return (value >> 2) & 0x0F

    @gpio1_mode.setter
    def gpio1_mode(self, mode: int) -> None:
        """Set the GPIO1 pin mode.

        :param mode: One of the GPIO1_* mode constants
        :raises ValueError: If mode is not a valid GPIO1_* constant
        """
        valid_modes: List[int] = [
            GPIO1_DISABLED,
            GPIO1_INPUT_MODE,
            GPIO1_GPI,
            GPIO1_GPO,
            GPIO1_CLKOUT,
            GPIO1_INT1,
            GPIO1_INT2,
            GPIO1_BCLK_OUT,
            GPIO1_WCLK_OUT,
        ]

        if mode not in valid_modes:
            raise ValueError(f"Invalid GPIO1 mode: {mode}. Must be one of the GPIO1_* constants.")

        self._page0._set_gpio1_mode(mode)

    @property
    def din_input(self) -> int:
        """Get the current DIN input value.

        :return: The DIN input value
        """
        return self._page0._get_din_input()

    @property
    def codec_interface(self) -> Dict[str, Any]:
        """Get the current codec interface settings.

        :return: Dictionary with codec interface settings
        """
        return self._page0._get_codec_interface()

    @property
    def headphone_shorted(self) -> bool:
        """Check if headphone short circuit is detected.

        :return: True if headphone is shorted, False otherwise
        """
        return self._page1._is_headphone_shorted()

    @property
    def speaker_shorted(self) -> bool:
        """Check if speaker short circuit is detected.

        :return: True if speaker is shorted, False otherwise
        """
        return self._page1._is_speaker_shorted()

    @property
    def hpl_gain_applied(self) -> bool:
        """Check if all programmed gains have been applied to HPL.

        :return: True if gains are applied, False otherwise
        """
        return self._page1._is_hpl_gain_applied()

    @property
    def hpr_gain_applied(self) -> bool:
        """Check if all programmed gains have been applied to HPR.

        :return: True if gains are applied, False otherwise
        """
        return self._page1._is_hpr_gain_applied()

    @property
    def speaker_gain_applied(self) -> bool:
        """Check if all programmed gains have been applied to Speaker.

        :return: True if gains are applied, False otherwise
        """
        return self._page1._is_spk_gain_applied()

    @property
    def headset_status(self) -> int:
        """Get current headset detection status.

        :return: Integer value representing headset status (0=none, 1=without mic, 3=with mic)
        """
        return self._page0._get_headset_status()

    @property
    def reset_speaker_on_scd(self) -> bool:
        """Get the speaker reset behavior on short circuit detection.

        :return: True if speaker resets on short circuit, False otherwise
        """
        value = self._page1._read_register(_HP_SPK_ERR_CTL)
        return not bool((value >> 1) & 0x01)

    @reset_speaker_on_scd.setter
    def reset_speaker_on_scd(self, reset: bool) -> None:
        """Configure speaker reset behavior on short circuit detection.

        :param reset: True to reset speaker on short circuit, False to remain unchanged
        """
        self._page1._reset_speaker_on_scd(reset)

    @property
    def reset_headphone_on_scd(self) -> bool:
        """Get the headphone reset behavior on short circuit detection.

        :return: True if headphone resets on short circuit, False otherwise
        """
        value = self._page1._read_register(_HP_SPK_ERR_CTL)
        return not bool(value & 0x01)

    @reset_headphone_on_scd.setter
    def reset_headphone_on_scd(self, reset: bool) -> None:
        """Configure headphone reset behavior on short circuit detection.

        :param reset: True to reset headphone on short circuit, False to remain unchanged
        """
        self._page1._reset_headphone_on_scd(reset)

    def configure_headphone_pop(
        self, wait_for_powerdown: bool = True, powerup_time: int = 0x07, ramp_time: int = 0x03
    ) -> bool:
        """Configure headphone pop removal settings.

        :param wait_for_powerdown: Wait for amp powerdown before DAC powerdown
        :param powerup_time: Driver power-on time (0-11)
        :param ramp_time: Driver ramp-up step time (0-3)
        :return: True if successful, False otherwise
        """
        return self._page1._configure_headphone_pop(wait_for_powerdown, powerup_time, ramp_time)

    @property
    def speaker_wait_time(self) -> int:
        """Get the current speaker power-up wait time.

        :return: The wait time setting (0-7)
        """
        value = self._page1._read_register(_PGA_RAMP)
        return (value >> 4) & 0x07

    @speaker_wait_time.setter
    def speaker_wait_time(self, wait_time: int) -> None:
        """Set speaker power-up wait time.

        :param wait_time: Speaker power-up wait duration (0-7)
        """
        self._page1._set_speaker_wait_time(wait_time)

    @property
    def headphone_lineout(self) -> bool:
        """Get the current headphone line-out configuration.

        :return: True if both channels are configured as line-out, False otherwise
        """
        value = self._page1._read_register(_HP_DRIVER_CTRL)
        left = bool(value & (1 << 2))
        right = bool(value & (1 << 1))
        return left and right

    @headphone_lineout.setter
    def headphone_lineout(self, enabled: bool) -> None:
        """Configure headphone outputs as line-out.

        :param enabled: True to configure both channels as line-out, False otherwise
        """
        self._page1._headphone_lineout(enabled, enabled)

    def config_mic_bias(
        self, power_down: bool = False, always_on: bool = False, voltage: int = 0
    ) -> bool:
        """Configure MICBIAS settings.

        :param power_down: Enable software power down
        :param always_on: Keep MICBIAS on even without headset
        :param voltage: MICBIAS voltage setting (0-3)
        :return: True if successful, False otherwise
        """
        return self._page1._config_mic_bias(power_down, always_on, voltage)

    def set_input_common_mode(self, ain1_cm: bool, ain2_cm: bool) -> bool:
        """Set analog input common mode connections.

        :param ain1_cm: Connect AIN1 to common mode when unused
        :param ain2_cm: Connect AIN2 to common mode when unused
        :return: True if successful, False otherwise
        """
        return self._page1._set_input_common_mode(ain1_cm, ain2_cm)

    def config_delay_divider(self, use_mclk: bool = True, divider: int = 1) -> bool:
        """Configure programmable delay timer clock source and divider.

        :param use_mclk: True to use external MCLK, False for internal oscillator
        :param divider: Clock divider (1-127, or 0 for 128)
        :return: True if successful, False otherwise
        """
        return self._page3._config_delay_divider(use_mclk, divider)

    @property
    def vol_adc_pin_control(self) -> bool:
        """Get the volume ADC pin control status.

        :return: True if volume ADC pin control is enabled, False otherwise
        """
        reg_value = self._page0._read_register(_VOL_ADC_CTRL)
        return bool(reg_value & (1 << 7))

    @vol_adc_pin_control.setter
    def vol_adc_pin_control(self, enabled: bool) -> None:
        """Enable or disable volume ADC pin control.

        :param enabled: True to enable volume ADC pin control, False to disable
        """
        current_config = self._get_vol_adc_config()
        self._page0._config_vol_adc(
            enabled,
            current_config["use_mclk"],
            current_config["hysteresis"],
            current_config["rate"],
        )

    @property
    def vol_adc_use_mclk(self) -> bool:
        """Get the volume ADC use MCLK status.

        :return: True if volume ADC uses MCLK, False otherwise
        """
        reg_value = self._page0._read_register(_VOL_ADC_CTRL)
        return bool(reg_value & (1 << 6))

    @vol_adc_use_mclk.setter
    def vol_adc_use_mclk(self, use_mclk: bool) -> None:
        """Set whether to use MCLK for volume ADC.

        :param use_mclk: True to use MCLK, False to use internal oscillator
        """
        current_config = self._get_vol_adc_config()
        self._page0._config_vol_adc(
            current_config["pin_control"],
            use_mclk,
            current_config["hysteresis"],
            current_config["rate"],
        )

    @property
    def vol_adc_hysteresis(self) -> int:
        """Get the volume ADC hysteresis setting.

        :return: Hysteresis value (0-3)
        """
        reg_value = self._page0._read_register(_VOL_ADC_CTRL)
        return (reg_value >> 4) & 0x03

    @vol_adc_hysteresis.setter
    def vol_adc_hysteresis(self, hysteresis: int) -> None:
        """Set the volume ADC hysteresis.

        :param hysteresis: Hysteresis value (0-3)
        """
        current_config = self._get_vol_adc_config()
        self._page0._config_vol_adc(
            current_config["pin_control"],
            current_config["use_mclk"],
            hysteresis,
            current_config["rate"],
        )

    @property
    def vol_adc_rate(self) -> int:
        """Get the volume ADC sampling rate.

        :return: Rate value (0-7)
        """
        reg_value = self._page0._read_register(_VOL_ADC_CTRL)
        return reg_value & 0x07

    @vol_adc_rate.setter
    def vol_adc_rate(self, rate: int) -> None:
        """Set the volume ADC sampling rate.

        :param rate: Rate value (0-7)
        """
        current_config = self._get_vol_adc_config()
        self._page0._config_vol_adc(
            current_config["pin_control"],
            current_config["use_mclk"],
            current_config["hysteresis"],
            rate,
        )

    def _get_vol_adc_config(self) -> Dict[str, Any]:
        """Helper method to get the current volume ADC configuration.

        :return: Dictionary with current volume ADC configuration
        """
        reg_value = self._page0._read_register(_VOL_ADC_CTRL)
        return {
            "pin_control": bool(reg_value & (1 << 7)),
            "use_mclk": bool(reg_value & (1 << 6)),
            "hysteresis": (reg_value >> 4) & 0x03,
            "rate": reg_value & 0x07,
        }

    @property
    def vol_adc_db(self) -> float:
        """Read the current volume from the Volume ADC in dB.

        :return: Volume in dB
        """
        return self._page0._read_vol_adc_db()

    def int2_sources(
        self,
        headset_detect: bool = False,
        button_press: bool = False,
        dac_drc: bool = False,
        agc_noise: bool = False,
        over_current: bool = False,
        multiple_pulse: bool = False,
    ) -> bool:
        """Configure the INT2 interrupt sources.

        :param headset_detect: Enable headset detection interrupt
        :param button_press: Enable button press detection interrupt
        :param dac_drc: Enable DAC DRC signal power interrupt
        :param agc_noise: Enable DAC data overflow interrupt
        :param over_current: Enable short circuit interrupt
        :param multiple_pulse: If true, INT2 generates multiple pulses until flag read
        :return: True if successful, False otherwise
        """
        return self._page0._set_int2_source(
            headset_detect, button_press, dac_drc, agc_noise, over_current, multiple_pulse
        )

    def configure_clocks(
        self, sample_rate: int, bit_depth: int = 16, mclk_freq: Optional[int] = None
    ):
        """Configure the TLV320DAC3100 clock settings.

        This function configures all necessary clock settings including PLL, dividers,
        and interface settings to achieve the requested sample rate.

        :param sample_rate: The desired sample rate in Hz (e.g., 44100, 48000)
        :param bit_depth: The bit depth (16, 20, 24, or 32), defaults to 16
        :param mclk_freq: The main clock frequency in Hz (e.g., 12000000 for 12MHz)
                         If None (default), BCLK will be used as the PLL input source
        :return: True if successful, False otherwise
        """
        self._sample_rate = sample_rate
        self._bit_depth = bit_depth
        if mclk_freq is not None:
            self._mclk_freq = mclk_freq
        else:
            self._mclk_freq = 0  # Internally use 0 to indicate BCLK mode

        return self._page0._configure_clocks_for_sample_rate(
            self._mclk_freq, sample_rate, bit_depth
        )

    @property
    def headphone_output(self) -> bool:
        """Headphone output helper with quickstart settings for users.
        Get headphone output state (True if either left or right channel is powered).

        :return: True if headphone output is enabled, False otherwise
        """
        hp_drivers = self._page1._read_register(_HP_DRIVERS)
        left_powered = bool(hp_drivers & (1 << 7))
        right_powered = bool(hp_drivers & (1 << 6))
        return left_powered or right_powered

    @headphone_output.setter
    def headphone_output(self, enabled: bool) -> None:
        """Headphone output helper with quickstart settings for users.
        Enable or disable headphone outputs.

        :param enabled: True to enable headphone output, False to disable
        """
        if enabled:
            self._page1._configure_headphone_driver(
                left_powered=True, right_powered=True, common=HP_COMMON_1_65V
            )
            self.headphone_left_gain = 0
            self.headphone_right_gain = 0
            self.headphone_left_mute = False
            self.headphone_right_mute = False
            self._page1._configure_analog_inputs(left_dac=DAC_ROUTE_HP, right_dac=DAC_ROUTE_HP)
            self.left_dac = True
            self.right_dac = True
            self.left_dac_path = DAC_PATH_NORMAL
            self.right_dac_path = DAC_PATH_NORMAL
            self.headphone_volume = 0
            self.dac_volume_control_mode = VOL_INDEPENDENT
            self.left_dac_mute = False
            self.right_dac_mute = False
            self.left_dac_channel_volume = 0
            self.right_dac_channel_volume = 0
        else:
            self._page1._configure_headphone_driver(left_powered=False, right_powered=False)
            self._page1._configure_analog_inputs(left_dac=DAC_ROUTE_NONE, right_dac=DAC_ROUTE_NONE)

    @property
    def speaker_output(self) -> bool:
        """Speaker output helper with quickstart settings for users.
        Get speaker output state.

        :return: True if speaker output is enabled, False otherwise
        """
        return self._page1._get_speaker_enabled()

    @speaker_output.setter
    def speaker_output(self, enabled: bool) -> None:
        """Speaker output helper with quickstart settings for users.
        Enable or disable speaker output.

        :param enabled: True to enable speaker, False to disable
        """
        if enabled:
            self._page1._set_speaker_enabled(True)
            self._page1._configure_analog_inputs(
                left_dac=DAC_ROUTE_MIXER, right_dac=DAC_ROUTE_MIXER
            )
            self.left_dac = True
            self.right_dac = True
            self.speaker_volume = -10
            self.speaker_gain = SPK_GAIN_6DB
            self.speaker_mute = False
        else:
            self._page1._set_speaker_enabled(False)

    @property
    def headphone_volume(self) -> float:
        """Get the current headphone volume in dB.

        :return: The volume in dB (0 = max, -63.5 = min)
        """
        left_gain = self._page1._read_register(_HPL_VOL) & 0x7F
        right_gain = self._page1._read_register(_HPR_VOL) & 0x7F
        avg_gain = (left_gain + right_gain) / 2
        # Convert from register value to dB
        # 55 ≈ 0dB, 0 ≈ -63.5dB
        db = (avg_gain - 55) / 1.14
        return db

    @headphone_volume.setter
    def headphone_volume(self, db: float) -> None:
        """Set headphone volume in dB.

        :param db: Volume in dB (0 = max, -63.5 = min)
        """
        # Convert from dB to register gain value (0-127)
        # 0dB = ~55, -63.5dB = 0
        if db > 0:
            db = 0  # Limit to 0dB to prevent distortion
        gain = int(55 + (db * 1.14))
        gain = max(0, min(gain, 127))
        self._page1._set_hpl_volume(route_enabled=True, gain=gain)
        self._page1._set_hpr_volume(route_enabled=True, gain=gain)

    @property
    def speaker_volume(self) -> float:
        """Get the current speaker volume in dB.

        :return: The volume in dB (0 = max, -63.5 = min)
        """
        gain = self._page1._read_register(_SPK_VOL) & 0x7F
        # Convert from register value to dB
        # 55 ≈ 0dB, 0 ≈ -63.5dB
        db = (gain - 55) / 1.14
        return db

    @speaker_volume.setter
    def speaker_volume(self, db: float) -> None:
        """Set speaker volume in dB.

        :param db: Volume in dB (0 = max, -63.5 = min)
        """
        if db > 0:
            db = 0
        gain = int(55 + (db * 1.14))
        gain = max(0, min(gain, 127))
        self._page1._set_spk_volume(route_enabled=True, gain=gain)

    @property
    def sample_rate(self) -> int:
        """Configured sample rate in Hz.

        :return: The sample rate in Hz
        """
        return self._sample_rate

    @property
    def bit_depth(self) -> int:
        """Configured bit depth.

        :return: The bit depth
        """
        return self._bit_depth

    @property
    def mclk_freq(self) -> int:
        """Configured MCLK frequency in Hz.

        :return: The MCLK frequency in Hz
        """
        return self._mclk_freq
