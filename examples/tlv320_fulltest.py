# SPDX-FileCopyrightText: 2024 Your Name
# SPDX-License-Identifier: MIT

"""
Comprehensive test for TLV320DAC3100 CircuitPython driver.
Tests all implemented functionality and plays a sine wave.
"""

import time
import array
import math
import audiocore
import board
import busio
import audiobusio
import adafruit_tlv320

def print_test_header(name):
    """Print a test section header."""
    print("\n" + "="*40)
    print(f"Testing {name}")
    print("="*40)

def test_result(name, result):
    """Print result of a test."""
    if result:
        print(f"✓ {name}: Success")
    else:
        print(f"✗ {name}: Failure")
    return result

# Initialize I2C bus
print("Initializing I2C...")
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the TLV320DAC3100
print("Initializing TLV320DAC3100...")
try:
    tlv = adafruit_tlv320.TLV320DAC3100(i2c)
    print("DAC initialized successfully!")
except Exception as e:
    print(f"Failed to initialize TLV320DAC3100: {e}")
    raise

# Reset the device
print_test_header("Reset")
test_result("Reset DAC", tlv.reset())

# Test basic health functions
print_test_header("Basic Health")
print(f"Overtemperature status: {'ALERT!' if tlv.overtemperature else 'OK'}")

# Test Clock Configuration
print_test_header("Clock Configuration")

# Set PLL input and codec input
tlv.pll_clock_input = adafruit_tlv320.PLL_CLKIN_BCLK
tlv.codec_clock_input = adafruit_tlv320.CODEC_CLKIN_PLL
print(f"PLL clock input set to: {tlv.pll_clock_input}")
print(f"CODEC clock input set to: {tlv.codec_clock_input}")

# Configure clock divider input
tlv.clock_divider_input = adafruit_tlv320.CDIV_CLKIN_PLL
print(f"Clock divider input: {tlv.clock_divider_input}")

# Set PLL values
test_result("Set PLL values", tlv.set_pll_values(1, 1, 7, 6144))
pll_vals = tlv.get_pll_values()
print(f"PLL values: P={pll_vals[0]}, R={pll_vals[1]}, J={pll_vals[2]}, D={pll_vals[3]}")

# Set DAC clock dividers
test_result("Set NDAC", tlv.set_ndac(True, 4))
ndac_vals = tlv.get_ndac()
print(f"NDAC: enabled={ndac_vals[0]}, value={ndac_vals[1]}")

test_result("Set MDAC", tlv.set_mdac(True, 1))
mdac_vals = tlv.get_mdac()
print(f"MDAC: enabled={mdac_vals[0]}, value={mdac_vals[1]}")

test_result("Set DOSR", tlv.set_dosr(256))
dosr_val = tlv.get_dosr()
print(f"DOSR value: {dosr_val}")

# Power up the PLL
tlv.pll_power = True
print(f"PLL power state: {'ON' if tlv.pll_power else 'OFF'}")

# Test GPIO and interrupt configuration
print_test_header("GPIO and Interrupts")

test_result("Set GPIO1 mode", tlv.set_gpio1_mode(adafruit_tlv320.GPIO1_INT1))
test_result("Set INT1 sources", tlv.set_int1_source(True, False, False, False, False, False))
test_result("Set INT2 sources", tlv.set_int2_source(True, False, False, False, False, False))
print(f"GPIO1 input state: {tlv.get_gpio1_input()}")
print(f"DIN input state: {tlv.get_din_input()}")

# Test codec interface configuration
print_test_header("Codec Interface")

test_result("Set codec interface", tlv.set_codec_interface(adafruit_tlv320.FORMAT_I2S, adafruit_tlv320.DATA_LEN_16))
codec_if = tlv.get_codec_interface()
print(f"Codec interface: {codec_if}")

# Test DAC path configuration
print_test_header("DAC Configuration")

test_result("Set DAC data path", tlv.set_dac_data_path(True, True))
dac_path = tlv.get_dac_data_path()
print(f"DAC data path: {dac_path}")

test_result("Set DAC volume control", tlv.set_dac_volume_control(False, False, adafruit_tlv320.VOL_INDEPENDENT))
vol_ctrl = tlv.get_dac_volume_control()
print(f"DAC volume control: {vol_ctrl}")

test_result("Set left channel volume", tlv.set_channel_volume(False, 0))
left_vol = tlv.get_channel_volume(False)
print(f"Left DAC volume: {left_vol} dB")

test_result("Set right channel volume", tlv.set_channel_volume(True, 0))
right_vol = tlv.get_channel_volume(True)
print(f"Right DAC volume: {right_vol} dB")

# Test headphone and speaker configuration
print_test_header("Headphone and Speaker")

test_result("Configure headphone driver", tlv.configure_headphone_driver(True, True))
test_result("Configure analog inputs", tlv.configure_analog_inputs(adafruit_tlv320.DAC_ROUTE_HP, adafruit_tlv320.DAC_ROUTE_HP))

test_result("Set HPL volume", tlv.set_hpl_volume(True, 20))
test_result("Configure HPL PGA", tlv.configure_hpl_pga(9, True))
print(f"HPL gain applied: {tlv.is_hpl_gain_applied()}")

test_result("Set HPR volume", tlv.set_hpr_volume(True, 20))
test_result("Configure HPR PGA", tlv.configure_hpr_pga(9, True))
print(f"HPR gain applied: {tlv.is_hpr_gain_applied()}")

tlv.speaker_enabled = True
print(f"Speaker enabled: {tlv.speaker_enabled}")
test_result("Configure SPK PGA", tlv.configure_spk_pga(adafruit_tlv320.SPK_GAIN_6DB, True))
test_result("Set SPK volume", tlv.set_spk_volume(False, 20))
print(f"SPK gain applied: {tlv.is_spk_gain_applied()}")
print(f"Speaker shorted: {tlv.is_speaker_shorted()}")

# Test headset detection
print_test_header("Headset Detection")

test_result("Set headset detect", tlv.set_headset_detect(True))
headset_status = tlv.get_headset_status()
print(f"Headset status: {headset_status}")

# Test hardware configuration
print_test_header("Hardware Configuration")

test_result("Reset speaker on SCD", tlv.reset_speaker_on_scd(True))
test_result("Reset headphone on SCD", tlv.reset_headphone_on_scd(True))
test_result("Configure headphone pop", tlv.configure_headphone_pop(True, 0x07, 0x03))
test_result("Set speaker wait time", tlv.set_speaker_wait_time(0x02))
test_result("Configure headphone as lineout", tlv.headphone_lineout(False, False))
test_result("Configure mic bias", tlv.config_mic_bias(False, False, 0x01))
test_result("Set input common mode", tlv.set_input_common_mode(True, True))
test_result("Configure delay divider", tlv.config_delay_divider(True, 1))

# Test Volume ADC
print_test_header("Volume ADC")

test_result("Configure Volume ADC", tlv.config_vol_adc(False, True, 0, 0))
vol_adc = tlv.read_vol_adc_db()
print(f"Volume ADC reading: {vol_adc} dB")

# Get DAC flags
print_test_header("DAC Status Flags")

dac_flags = tlv.get_dac_flags()
print("DAC Flags:")
for key, value in dac_flags.items():
    print(f"  {key}: {value}")

# Initialize I2S for audio playback
print_test_header("Audio Playback")
print("Initializing I2S...")

try:
    # BCLK, WCLK/LRCLK, DATA - adjust pins as needed for your board
    audio = audiobusio.I2SOut(board.I2S_BCLK, board.I2S_WS, board.I2S_DIN)
    
    # Generate a sine wave
    tone_volume = 0.5  # Higher volume
    frequency = 440    # 440 Hz tone (A4)
    sample_rate = 44100  # CD-quality sample rate
    length = sample_rate // frequency
    sine_wave = array.array("h", [0] * length)
    for i in range(length):
        sine_wave[i] = int((math.sin(math.pi * 2 * i / length)) * tone_volume * (2 ** 15 - 1))
    
    sine_wave_sample = audiocore.RawSample(sine_wave, sample_rate=sample_rate)
    
    print("Starting audio playback...")
    
    # Continuous playback
    audio.play(sine_wave_sample, loop=True)
    
    # Main loop
    print("All tests completed! Playing tone continuously.")
    print("Press Ctrl+C to stop.")
    
    while True:
        # Check for overtemperature every 5 seconds
        if tlv.overtemperature:
            print("WARNING: DAC is overheating!")
        time.sleep(5)
        
except Exception as e:
    print(f"Audio error: {e}")
    
    # If audio setup fails, just loop
    while True:
        time.sleep(1)
