# SPDX-FileCopyrightText: 2024 Liz Clark for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
CircuitPython TLV320DAC3100 I2S Tone playback example.
Configures the TLV320DAC3100 DAC via I2C, then plays a tone
for one second on, one second off, in a loop.
"""
import time
import array
import math
import audiocore
import board
import busio
import audiobusio
import adafruit_tlv320

# Initialize I2C for DAC configuration
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize and configure the TLV320DAC3100
print("Initializing TLV320DAC3100...")
tlv = adafruit_tlv320.TLV320DAC3100(i2c)

# Reset configuration
tlv.reset()

# Configure the codec interface for I2S mode with 16-bit data
print("Configuring codec interface...")
tlv.set_codec_interface(adafruit_tlv320.FORMAT_I2S, adafruit_tlv320.DATA_LEN_16)

# Configure PLL with BCLK as input
print("Configuring clocks...")
tlv.pll_clock_input = adafruit_tlv320.PLL_CLKIN_BCLK
tlv.codec_clock_input = adafruit_tlv320.CODEC_CLKIN_PLL

# For a standard 44.1kHz sample rate with 16-bit stereo, BCLK is 1.4112MHz
# We need to configure PLL to generate the appropriate internal clocks
# These PLL settings are very dependent on your exact I2S configuration
# You may need to experiment with these values
tlv.set_pll_values(1, 1, 7, 6144)  # Different PLL values to try
tlv.set_ndac(True, 4)
tlv.set_mdac(True, 1)

# Power up the PLL
print("Powering up PLL...")
tlv.pll_power = True
time.sleep(0.1)  # Give PLL time to stabilize

# Set up DAC data path - explicitly enable both channels
print("Setting up DAC data path...")
tlv.set_dac_data_path(True, True, 
                     adafruit_tlv320.DAC_PATH_NORMAL, 
                     adafruit_tlv320.DAC_PATH_NORMAL)

# Configure volume - ensure we're unmuted and at a reasonable level
print("Setting DAC volume...")
tlv.set_dac_volume_control(False, False)  # Make sure both channels are unmuted
tlv.set_channel_volume(False, 0)  # Left channel at 0dB (max)
tlv.set_channel_volume(True, 0)   # Right channel at 0dB (max)

# Set up analog routing - route DAC to headphone
print("Configuring analog inputs...")
tlv.configure_analog_inputs(
    adafruit_tlv320.DAC_ROUTE_HP,  # Route left DAC directly to headphone
    adafruit_tlv320.DAC_ROUTE_HP   # Route right DAC directly to headphone
)

# Configure headphone driver - ensure it's powered up and unmuted
print("Configuring headphone drivers...")
tlv.configure_headphone_driver(True, True)  # Power up both left and right drivers

# Explicitly set headphone volume to a high level
tlv.set_hpl_volume(True, 0)  # Enable route with gain of 20
tlv.set_hpr_volume(True, 0)  # Enable route with gain of 20

# Ensure the headphone drivers are unmuted with gain
tlv.configure_hpl_pga(-1, True)  # Max gain (9dB), unmuted
tlv.configure_hpr_pga(-1, True)  # Max gain (9dB), unmuted

print("DAC configuration complete!")

# Initialize I2S for audio playback
# Depending on your board, these pins may be different
# BCLK, WCLK/LRCLK, DATA
audio = audiobusio.I2SOut(board.I2S_BCLK, board.I2S_WS, board.I2S_DIN)

# Generate a sine wave at a higher volume
tone_volume = 0.1  # Increased volume
frequency = 440    # 440 Hz tone (A4)
sample_rate = 8000  # Sample rate in Hz
length = sample_rate // frequency
sine_wave = array.array("h", [0] * length)
for i in range(length):
    sine_wave[i] = int((math.sin(math.pi * 2 * i / length)) * tone_volume * (2 ** 15 - 1))

sine_wave_sample = audiocore.RawSample(sine_wave, sample_rate=sample_rate)

print("Starting audio playback...")

# Play the tone without stopping
audio.play(sine_wave_sample, loop=True)

# Keep the program running
while True:
    time.sleep(1)
    print("Tone is playing... Ctrl+C to stop")