# SPDX-FileCopyrightText: Copyright (c) 2025 Liz Clark for Adafruit Industries
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
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

import time
from micropython import const
from adafruit_bus_device.i2c_device import I2CDevice

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_TLV320.git"

# Register addresses
REG_PAGE_SELECT = const(0x00)
REG_RESET = const(0x01)
REG_OT_FLAG = const(0x03)
REG_CLOCK_MUX1 = const(0x04)
REG_PLL_PROG_PR = const(0x05)
REG_PLL_PROG_J = const(0x06)
REG_PLL_PROG_D_MSB = const(0x07)
REG_PLL_PROG_D_LSB = const(0x08)
REG_NDAC = const(0x0B)
REG_MDAC = const(0x0C)
REG_DOSR_MSB = const(0x0D)
REG_DOSR_LSB = const(0x0E)
REG_CLKOUT_MUX = const(0x19)
REG_CLKOUT_M = const(0x1A)
REG_CODEC_IF_CTRL1 = const(0x1B)
REG_DATA_SLOT_OFFSET = const(0x1C)
REG_BCLK_N = const(0x1E)
REG_DAC_FLAG = const(0x25)
REG_DAC_FLAG2 = const(0x26)
REG_INT1_CTRL = const(0x30)
REG_INT2_CTRL = const(0x31)
REG_GPIO1_CTRL = const(0x33)
REG_DIN_CTRL = const(0x36)
REG_DAC_PRB = const(0x3C)
REG_DAC_DATAPATH = const(0x3F)
REG_DAC_VOL_CTRL = const(0x40)
REG_DAC_LVOL = const(0x41)
REG_DAC_RVOL = const(0x42)
REG_HEADSET_DETECT = const(0x43)

# Page 1 registers
REG_HP_SPK_ERR_CTL = const(0x1E)
REG_HP_DRIVERS = const(0x1F)
REG_SPK_AMP = const(0x20)
REG_HP_POP = const(0x21)
REG_PGA_RAMP = const(0x22)
REG_OUT_ROUTING = const(0x23)
REG_HPL_VOL = const(0x24)
REG_HPR_VOL = const(0x25)
REG_SPK_VOL = const(0x26)
REG_HPL_DRIVER = const(0x28)
REG_HPR_DRIVER = const(0x29)
REG_SPK_DRIVER = const(0x2A)
REG_HP_DRIVER_CTRL = const(0x2C)

# Default I2C address
I2C_ADDR_DEFAULT = const(0x18)

# Data format for I2S interface
FORMAT_I2S = const(0b00)      # I2S format
FORMAT_DSP = const(0b01)      # DSP format
FORMAT_RJF = const(0b10)      # Right justified format
FORMAT_LJF = const(0b11)      # Left justified format

# Data length for I2S interface
DATA_LEN_16 = const(0b00)     # 16 bits
DATA_LEN_20 = const(0b01)     # 20 bits
DATA_LEN_24 = const(0b10)     # 24 bits
DATA_LEN_32 = const(0b11)     # 32 bits

# Clock source options for PLL_CLKIN
PLL_CLKIN_MCLK = const(0b00)   # MCLK pin is the source
PLL_CLKIN_BCLK = const(0b01)   # BCLK pin is the source
PLL_CLKIN_GPIO1 = const(0b10)  # GPIO1 pin is the source
PLL_CLKIN_DIN = const(0b11)    # DIN pin is the source

# Clock source options for CODEC_CLKIN
CODEC_CLKIN_MCLK = const(0b00)   # MCLK pin is the source
CODEC_CLKIN_BCLK = const(0b01)   # BCLK pin is the source
CODEC_CLKIN_GPIO1 = const(0b10)  # GPIO1 pin is the source
CODEC_CLKIN_PLL = const(0b11)    # PLL_CLK pin is the source

# GPIO1 pin mode options
GPIO1_DISABLED = const(0b0000)    # GPIO1 disabled (input and output buffers powered down)
GPIO1_INPUT_MODE = const(0b0001)  # Input mode (secondary BCLK/WCLK/DIN input or ClockGen)
GPIO1_GPI = const(0b0010)         # General-purpose input
GPIO1_GPO = const(0b0011)         # General-purpose output
GPIO1_CLKOUT = const(0b0100)      # CLKOUT output
GPIO1_INT1 = const(0b0101)        # INT1 output
GPIO1_INT2 = const(0b0110)        # INT2 output
GPIO1_BCLK_OUT = const(0b1000)    # Secondary BCLK output for codec interface
GPIO1_WCLK_OUT = const(0b1001)    # Secondary WCLK output for codec interface

# DAC channel data path options
DAC_PATH_OFF = const(0b00)      # DAC data path off
DAC_PATH_NORMAL = const(0b01)   # Normal path (L->L or R->R)
DAC_PATH_SWAPPED = const(0b10)  # Swapped path (R->L or L->R)
DAC_PATH_MIXED = const(0b11)    # Mixed L+R path

# DAC volume control soft stepping options
VOLUME_STEP_1SAMPLE = const(0b00)    # One step per sample
VOLUME_STEP_2SAMPLE = const(0b01)    # One step per two samples
VOLUME_STEP_DISABLED = const(0b10)   # Soft stepping disabled

# DAC volume control configuration options
VOL_INDEPENDENT = const(0b00)    # Left and right channels independent
VOL_LEFT_TO_RIGHT = const(0b01)  # Left follows right volume
VOL_RIGHT_TO_LEFT = const(0b10)  # Right follows left volume

# DAC output routing options
DAC_ROUTE_NONE = const(0b00)    # DAC not routed
DAC_ROUTE_MIXER = const(0b01)   # DAC routed to mixer amplifier
DAC_ROUTE_HP = const(0b10)      # DAC routed directly to HP driver

# Speaker amplifier gain options
SPK_GAIN_6DB = const(0b00)   # 6 dB gain
SPK_GAIN_12DB = const(0b01)  # 12 dB gain
SPK_GAIN_18DB = const(0b10)  # 18 dB gain
SPK_GAIN_24DB = const(0b11)  # 24 dB gain

# Headphone common mode voltage settings
HP_COMMON_1_35V = const(0b00)  # Common-mode voltage 1.35V
HP_COMMON_1_50V = const(0b01)  # Common-mode voltage 1.50V
HP_COMMON_1_65V = const(0b10)  # Common-mode voltage 1.65V
HP_COMMON_1_80V = const(0b11)  # Common-mode voltage 1.80V

# Headset detection debounce time options
DEBOUNCE_16MS = const(0b000)    # 16ms debounce (2ms clock)
DEBOUNCE_32MS = const(0b001)    # 32ms debounce (4ms clock)
DEBOUNCE_64MS = const(0b010)    # 64ms debounce (8ms clock)
DEBOUNCE_128MS = const(0b011)   # 128ms debounce (16ms clock)
DEBOUNCE_256MS = const(0b100)   # 256ms debounce (32ms clock)
DEBOUNCE_512MS = const(0b101)   # 512ms debounce (64ms clock)

# Button press debounce time options
BTN_DEBOUNCE_0MS = const(0b00)   # No debounce
BTN_DEBOUNCE_8MS = const(0b01)   # 8ms debounce (1ms clock)
BTN_DEBOUNCE_16MS = const(0b10)  # 16ms debounce (2ms clock)
BTN_DEBOUNCE_32MS = const(0b11)  # 32ms debounce (4ms clock)


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
        # First set the page
        self._set_page()
        
        # Then write to the register
        self._buffer[0] = register
        self._buffer[1] = value
        with self._device as i2c:
            i2c.write(self._buffer)
    
    def _read_register(self, register):
        """Read a value from a register.
        
        :param register: The register address
        :return: The register value
        """
        # First set the page
        self._set_page()
        
        # Then read the register
        self._buffer[0] = register
        with self._device as i2c:
            i2c.write(self._buffer, end=1)
            i2c.readinto(self._buffer, start=0, end=1)
        return self._buffer[0]
    
    def _set_page(self):
        """Set the current register page."""
        self._buffer[0] = REG_PAGE_SELECT
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
        :return: True if successful
        """
        reg_value = self._read_register(register)
        reg_value &= ~(mask << shift)  # Clear the bits
        reg_value |= (value & mask) << shift  # Set the new bits
        self._write_register(register, reg_value)
        return True


class Page0Registers(PagedRegisterBase):
    """Page 0 registers containing system configuration, clocking, etc."""
    
    def __init__(self, i2c_device):
        """Initialize Page 0 registers.
        
        :param i2c_device: The I2C device
        """
        super().__init__(i2c_device, 0)
    
    def reset(self):
        """Perform a software reset of the chip.
        
        :return: True if successful, False otherwise
        """
        # Set reset bit
        self._write_register(REG_RESET, 1)
        
        # Wait for reset to complete
        time.sleep(0.01)  # 10ms delay
        
        # Reset bit should be 0 after reset completes
        return self._read_register(REG_RESET) == 0
    
    def is_overtemperature(self):
        """Check if the chip is in an over-temperature condition.
        
        :return: True if overtemp condition exists, False if temperature is OK
        """
        # Bit 1 of register 3 is the overtemp flag (0 = overtemp, 1 = normal)
        return not ((self._read_register(REG_OT_FLAG) >> 1) & 0x01)
    
    # Methods with property potential
    
    def get_pll_clock_input(self):
        """Get the PLL clock input source."""
        return self._get_bits(REG_CLOCK_MUX1, 0x03, 2)
    
    def set_pll_clock_input(self, clkin):
        """Set the PLL clock input source."""
        return self._set_bits(REG_CLOCK_MUX1, 0x03, 2, clkin)
    
    def get_codec_clock_input(self):
        """Get the CODEC clock input source."""
        return self._get_bits(REG_CLOCK_MUX1, 0x03, 0)
    
    def set_codec_clock_input(self, clkin):
        """Set the CODEC clock input source."""
        return self._set_bits(REG_CLOCK_MUX1, 0x03, 0, clkin)
    
    def get_pll_power(self):
        """Get the PLL power state."""
        return bool(self._get_bits(REG_PLL_PROG_PR, 0x01, 7))
    
    def set_pll_power(self, on):
        """Set the PLL power state."""
        return self._set_bits(REG_PLL_PROG_PR, 0x01, 7, 1 if on else 0)
    
    # Other methods
    
    def set_pll_values(self, p, r, j, d):
        """Set the PLL P, R, J, and D values.
        
        :param p: PLL P value (1-8)
        :param r: PLL R value (1-16)
        :param j: PLL J value (1-63)
        :param d: PLL D value (0-9999)
        :return: True if successful, False if failure
        """
        # Validate all input ranges
        if p < 1 or p > 8:
            return False
        if r < 1 or r > 16:
            return False
        if j < 1 or j > 63:
            return False
        if d > 9999:
            return False
        
        # P & R register
        p_val = p % 8  # P values wrap at 8
        r_val = r % 16  # R values wrap at 16
        
        # Write P (bits 6:4) and R (bits 3:0) values
        pr_value = (p_val << 4) | r_val
        self._write_register(REG_PLL_PROG_PR, pr_value)
        
        # J register (bits 5:0)
        self._write_register(REG_PLL_PROG_J, j & 0x3F)
        
        # D MSB & LSB registers (14 bits total)
        self._write_register(REG_PLL_PROG_D_MSB, (d >> 8) & 0xFF)
        self._write_register(REG_PLL_PROG_D_LSB, d & 0xFF)
        
        return True
    
    def set_ndac(self, enable, val):
        """Set the NDAC value and enable/disable."""
        # Validate input range
        if val < 1 or val > 128:
            return False
        
        value = ((1 if enable else 0) << 7) | (val % 128)
        self._write_register(REG_NDAC, value)
        return True
    
    def set_mdac(self, enable, val):
        """Set the MDAC value and enable/disable."""
        # Validate input range
        if val < 1 or val > 128:
            return False
        
        value = ((1 if enable else 0) << 7) | (val % 128)
        self._write_register(REG_MDAC, value)
        return True
    
    def set_codec_interface(self, format, data_len, bclk_out=False, wclk_out=False):
        """Set the codec interface parameters."""
        value = (format & 0x03) << 6
        value |= (data_len & 0x03) << 4
        value |= (1 if bclk_out else 0) << 3
        value |= (1 if wclk_out else 0) << 2
        
        self._write_register(REG_CODEC_IF_CTRL1, value)
        return True
    
    def set_int1_source(self, headset_detect, button_press, dac_drc, 
                       agc_noise, over_current, multiple_pulse):
        """Configure the INT1 interrupt sources."""
        value = 0
        if headset_detect: value |= (1 << 7)
        if button_press: value |= (1 << 6)
        if dac_drc: value |= (1 << 5)
        if over_current: value |= (1 << 3)
        if agc_noise: value |= (1 << 2)
        if multiple_pulse: value |= (1 << 0)
        
        self._write_register(REG_INT1_CTRL, value)
        return True
    
    def set_gpio1_mode(self, mode):
        """Set the GPIO1 pin mode."""
        return self._set_bits(REG_GPIO1_CTRL, 0x0F, 2, mode)
    
    def set_headset_detect(self, enable, detect_debounce=0, button_debounce=0):
        """Configure headset detection settings."""
        value = (1 if enable else 0) << 7
        value |= (detect_debounce & 0x07) << 2
        value |= (button_debounce & 0x03)
        
        self._write_register(REG_HEADSET_DETECT, value)
        return True
    
    def set_dac_data_path(self, left_dac_on, right_dac_on, 
                         left_path=DAC_PATH_NORMAL, 
                         right_path=DAC_PATH_NORMAL, 
                         volume_step=VOLUME_STEP_1SAMPLE):
        """Configure the DAC data path settings."""
        value = 0
        if left_dac_on: value |= (1 << 7)
        if right_dac_on: value |= (1 << 6)
        value |= (left_path & 0x03) << 4
        value |= (right_path & 0x03) << 2
        value |= (volume_step & 0x03)
        
        self._write_register(REG_DAC_DATAPATH, value)
        return True
    
    def set_dac_volume_control(self, left_mute, right_mute, control=VOL_INDEPENDENT):
        """Configure the DAC volume control settings."""
        value = 0
        if left_mute: value |= (1 << 3)
        if right_mute: value |= (1 << 2)
        value |= (control & 0x03)
        
        self._write_register(REG_DAC_VOL_CTRL, value)
        return True
    
    def set_channel_volume(self, right_channel, db):
        """Set DAC channel volume in dB."""
        # Constrain input to valid range
        if db > 24.0:
            db = 24.0
        if db < -63.5:
            db = -63.5
        
        reg_val = int(db * 2)
        
        # Check for reserved values
        if reg_val == 0x80 or reg_val > 0x30:
            return False
        
        if right_channel:
            self._write_register(REG_DAC_RVOL, reg_val & 0xFF)
        else:
            self._write_register(REG_DAC_LVOL, reg_val & 0xFF)
        
        return True


class Page1Registers(PagedRegisterBase):
    """Page 1 registers containing analog output settings, HP/SPK controls, etc."""
    
    def __init__(self, i2c_device):
        """Initialize Page 1 registers.
        
        :param i2c_device: The I2C device
        """
        super().__init__(i2c_device, 1)
    
    def get_speaker_enabled(self):
        """Check if speaker is enabled."""
        return bool(self._get_bits(REG_SPK_AMP, 0x01, 7))
    
    def set_speaker_enabled(self, enable):
        """Enable or disable the Class-D speaker amplifier."""
        return self._set_bits(REG_SPK_AMP, 0x01, 7, 1 if enable else 0)
    
    def configure_headphone_driver(self, left_powered, right_powered, 
                                  common=HP_COMMON_1_35V, power_down_on_scd=False):
        """Configure headphone driver settings."""
        value = 0x04  # bit 2 must be 1
        if left_powered: value |= (1 << 7)
        if right_powered: value |= (1 << 6)
        value |= (common & 0x03) << 3
        if power_down_on_scd: value |= (1 << 1)
        
        self._write_register(REG_HP_DRIVERS, value)
        return True
    
    def configure_analog_inputs(self, left_dac=DAC_ROUTE_NONE, right_dac=DAC_ROUTE_NONE,
                              left_ain1=False, left_ain2=False, right_ain2=False, 
                              hpl_routed_to_hpr=False):
        """Configure DAC and analog input routing."""
        value = 0
        value |= (left_dac & 0x03) << 6
        if left_ain1: value |= (1 << 5)
        if left_ain2: value |= (1 << 4)
        value |= (right_dac & 0x03) << 2
        if right_ain2: value |= (1 << 1)
        if hpl_routed_to_hpr: value |= 1
        
        self._write_register(REG_OUT_ROUTING, value)
        return True
    
    def set_hpl_volume(self, route_enabled, gain=0x7F):
        """Set HPL analog volume control."""
        if gain > 0x7F: 
            gain = 0x7F
        
        value = ((1 if route_enabled else 0) << 7) | (gain & 0x7F)
        self._write_register(REG_HPL_VOL, value)
        return True
    
    def set_hpr_volume(self, route_enabled, gain=0x7F):
        """Set HPR analog volume control."""
        if gain > 0x7F: 
            gain = 0x7F
        
        value = ((1 if route_enabled else 0) << 7) | (gain & 0x7F)
        self._write_register(REG_HPR_VOL, value)
        return True
    
    def set_spk_volume(self, route_enabled, gain=0x7F):
        """Set Speaker analog volume control."""
        if gain > 0x7F: 
            gain = 0x7F
        
        value = ((1 if route_enabled else 0) << 7) | (gain & 0x7F)
        self._write_register(REG_SPK_VOL, value)
        return True
    
    def configure_hpl_pga(self, gain_db=0, unmute=True):
        """Configure HPL driver PGA settings."""
        if gain_db > 9:
            return False
        
        value = (gain_db & 0x0F) << 3
        if unmute: value |= (1 << 2)
        
        self._write_register(REG_HPL_DRIVER, value)
        return True
    
    def configure_hpr_pga(self, gain_db=0, unmute=True):
        """Configure HPR driver PGA settings."""
        if gain_db > 9:
            return False
        
        value = (gain_db & 0x0F) << 3
        if unmute: value |= (1 << 2)
        
        self._write_register(REG_HPR_DRIVER, value)
        return True
    
    def configure_spk_pga(self, gain=SPK_GAIN_6DB, unmute=True):
        """Configure Speaker driver settings."""
        value = (gain & 0x03) << 3
        if unmute: value |= (1 << 2)
        
        self._write_register(REG_SPK_DRIVER, value)
        return True


class TLV320DAC3100:
    """Driver for the TI TLV320DAC3100 Stereo DAC with Headphone Amplifier."""

    def __init__(self, i2c, address=I2C_ADDR_DEFAULT):
        """Initialize the TLV320DAC3100.
        
        :param i2c: The I2C bus the device is connected to
        :param address: The I2C device address (default is 0x18)
        """
        self._device = I2CDevice(i2c, address)
        
        # Initialize register page classes
        self._page0 = Page0Registers(self._device)
        self._page1 = Page1Registers(self._device)
        
        # Reset the device
        if not self.reset():
            raise RuntimeError("Failed to reset TLV320DAC3100")
    
    # Basic properties and methods
    
    def reset(self):
        """Reset the device."""
        return self._page0.reset()
    
    @property
    def overtemperature(self):
        """Check if the chip is overheating."""
        return self._page0.is_overtemperature()
    
    # PLL and Clock properties
    
    @property
    def pll_clock_input(self):
        """Get the PLL clock input source."""
        return self._page0.get_pll_clock_input()
    
    @pll_clock_input.setter
    def pll_clock_input(self, clkin):
        """Set the PLL clock input source."""
        self._page0.set_pll_clock_input(clkin)
    
    @property
    def codec_clock_input(self):
        """Get the CODEC clock input source."""
        return self._page0.get_codec_clock_input()
    
    @codec_clock_input.setter
    def codec_clock_input(self, clkin):
        """Set the CODEC clock input source."""
        self._page0.set_codec_clock_input(clkin)
    
    @property
    def pll_power(self):
        """Get the PLL power state."""
        return self._page0.get_pll_power()
    
    @pll_power.setter
    def pll_power(self, on):
        """Set the PLL power state."""
        self._page0.set_pll_power(on)
    
    # Speaker property
    
    @property
    def speaker_enabled(self):
        """Get the speaker amplifier state."""
        return self._page1.get_speaker_enabled()
    
    @speaker_enabled.setter
    def speaker_enabled(self, enable):
        """Enable or disable the speaker amplifier."""
        self._page1.set_speaker_enabled(enable)
    
    # Method-based API for complex operations
    
    def set_pll_values(self, p, r, j, d):
        """Set the PLL P, R, J, and D values."""
        return self._page0.set_pll_values(p, r, j, d)
    
    def power_pll(self, on):
        """Set the PLL power state."""
        return self._page0.set_pll_power(on)
    
    def set_ndac(self, enable, val):
        """Set the NDAC value and enable/disable."""
        return self._page0.set_ndac(enable, val)
    
    def set_mdac(self, enable, val):
        """Set the MDAC value and enable/disable."""
        return self._page0.set_mdac(enable, val)
    
    def set_codec_interface(self, format, data_len, bclk_out=False, wclk_out=False):
        """Set the codec interface parameters."""
        return self._page0.set_codec_interface(format, data_len, bclk_out, wclk_out)
    
    def set_headset_detect(self, enable, detect_debounce=0, button_debounce=0):
        """Configure headset detection settings."""
        return self._page0.set_headset_detect(enable, detect_debounce, button_debounce)
    
    def set_int1_source(self, headset_detect, button_press, dac_drc, 
                       agc_noise, over_current, multiple_pulse):
        """Configure the INT1 interrupt sources."""
        return self._page0.set_int1_source(headset_detect, button_press, dac_drc, 
                                        agc_noise, over_current, multiple_pulse)
    
    def set_gpio1_mode(self, mode):
        """Set the GPIO1 pin mode."""
        return self._page0.set_gpio1_mode(mode)

    def set_dac_data_path(self, left_dac_on, right_dac_on, 
                         left_path=DAC_PATH_NORMAL, 
                         right_path=DAC_PATH_NORMAL, 
                         volume_step=VOLUME_STEP_1SAMPLE):
        """Configure the DAC data path settings."""
        return self._page0.set_dac_data_path(left_dac_on, right_dac_on, 
                                          left_path, right_path, volume_step)
    
    def configure_analog_inputs(self, left_dac=DAC_ROUTE_NONE, right_dac=DAC_ROUTE_NONE,
                              left_ain1=False, left_ain2=False, right_ain2=False, 
                              hpl_routed_to_hpr=False):
        """Configure DAC and analog input routing."""
        return self._page1.configure_analog_inputs(left_dac, right_dac, 
                                               left_ain1, left_ain2, 
                                               right_ain2, hpl_routed_to_hpr)
    
    def set_dac_volume_control(self, left_mute, right_mute, control=VOL_INDEPENDENT):
        """Configure the DAC volume control settings."""
        return self._page0.set_dac_volume_control(left_mute, right_mute, control)
    
    def set_channel_volume(self, right_channel, db):
        """Set DAC channel volume in dB."""
        return self._page0.set_channel_volume(right_channel, db)
    
    def configure_headphone_driver(self, left_powered, right_powered, 
                                  common=HP_COMMON_1_35V, power_down_on_scd=False):
        """Configure headphone driver settings."""
        return self._page1.configure_headphone_driver(left_powered, right_powered, 
                                                   common, power_down_on_scd)
    
    def set_hpl_volume(self, route_enabled, gain=0x7F):
        """Set HPL analog volume control."""
        return self._page1.set_hpl_volume(route_enabled, gain)
    
    def configure_hpl_pga(self, gain_db=0, unmute=True):
        """Configure HPL driver PGA settings."""
        return self._page1.configure_hpl_pga(gain_db, unmute)
    
    def set_hpr_volume(self, route_enabled, gain=0x7F):
        """Set HPR analog volume control."""
        return self._page1.set_hpr_volume(route_enabled, gain)
    
    def configure_hpr_pga(self, gain_db=0, unmute=True):
        """Configure HPR driver PGA settings."""
        return self._page1.configure_hpr_pga(gain_db, unmute)
    
    def enable_speaker(self, enable):
        """Enable or disable the Class-D speaker amplifier."""
        return self._page1.set_speaker_enabled(enable)
    
    def configure_spk_pga(self, gain=SPK_GAIN_6DB, unmute=True):
        """Configure Speaker driver settings."""
        return self._page1.configure_spk_pga(gain, unmute)
    
    def set_spk_volume(self, route_enabled, gain=0x7F):
        """Set Speaker analog volume control."""
        return self._page1.set_spk_volume(route_enabled, gain)