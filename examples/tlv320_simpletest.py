# SPDX-FileCopyrightText: 2025 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

import array
import math
import time

import audiobusio
import audiocore
import board
import busio

import adafruit_tlv320

i2c = busio.I2C(board.SCL, board.SDA)
dac = adafruit_tlv320.TLV320DAC3100(i2c)

# set mclk, sample rate & bit depth
dac.configure_clocks(mclk_freq=12000000, sample_rate=44100, bit_depth=16)

# use headphones
dac.headphone_output = True
dac.headphone_volume = -20  # dB
# or use speaker
# dac.speaker_output = True
# dac.speaker_volume = -15 # dB

audio = audiobusio.I2SOut(board.I2S_BCLK, board.I2S_WS, board.I2S_DIN)
# generate a sine wave
tone_volume = 0.5
frequency = 440
sample_rate = dac.sample_rate
length = sample_rate // frequency
sine_wave = array.array("h", [0] * length)
for i in range(length):
    sine_wave[i] = int((math.sin(math.pi * 2 * i / length)) * tone_volume * (2**15 - 1))
sine_wave_sample = audiocore.RawSample(sine_wave, sample_rate=sample_rate)

while True:
    audio.stop()
    time.sleep(1)
    audio.play(sine_wave_sample, loop=True)
    time.sleep(1)
