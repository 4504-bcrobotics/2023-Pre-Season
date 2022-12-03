# Copyright (c) 2016 Adafruit Industries
# Author: Tony DiCola
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
from __future__ import division
import logging
import time
import math

import smbus

class pca9685:
    # Registers/etc:
    PCA9685_ADDRESS    = 0x40
    MODE1              = 0x00
    MODE2              = 0x01
    SUBADR1            = 0x02
    SUBADR2            = 0x03
    SUBADR3            = 0x04
    PRESCALE           = 0xFE
    LED0_ON_L          = 0x06
    LED0_ON_H          = 0x07
    LED0_OFF_L         = 0x08
    LED0_OFF_H         = 0x09
    ALL_LED_ON_L       = 0xFA
    ALL_LED_ON_H       = 0xFB
    ALL_LED_OFF_L      = 0xFC
    ALL_LED_OFF_H      = 0xFD

    # Bits:
    RESTART            = 0x80
    SLEEP              = 0x10
    ALLCALL            = 0x01
    INVRT              = 0x10
    OUTDRV             = 0x04

    def __init__(self, address=0x40, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        self.set_all_pwm(0, 0)
        self.bus.write_byte_data(self.address, self.MODE2, self.OUTDRV)
        self.bus.write_byte_data(self.address, self.MODE1, self.ALLCALL)
        time.sleep(0.005)  # wait for oscillator
        mode1 = self.bus.read_byte_data(self.address, self.MODE1)
        mode1 = mode1 & ~self.SLEEP  # wake up (reset sleep)
        self.bus.write_byte_data(self.address, self.MODE1, mode1)
        time.sleep(0.005)  # wait for oscillator


    def set_pwm_freq(self, freq_hz):
        """Set the PWM frequency to the provided value in hertz."""
        prescaleval = 25000000.0    # 25MHz
        prescaleval /= 4096.0       # 12-bit
        prescaleval /= float(freq_hz)
        prescaleval -= 1.0
        prescale = int(math.floor(prescaleval + 0.5))
        oldmode = self.bus.read_byte_data(self.address, self.MODE1);
        newmode = (oldmode & 0x7F) | 0x10    # sleep
        self.bus.write_byte_data(self.address, self.MODE1, newmode)  # go to sleep
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)
        self.bus.write_byte_data(self.address, self.MODE1, oldmode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, self.MODE1, oldmode | 0x80)

    def set_pwm(self, channel, on, off):
        """Sets a single PWM channel."""
        self.bus.write_byte_data(self.address, self.LED0_ON_L+4*channel, on & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_ON_H+4*channel, on >> 8)
        self.bus.write_byte_data(self.address, self.LED0_OFF_L+4*channel, off & 0xFF)
        self.bus.write_byte_data(self.address, self.LED0_OFF_H+4*channel, off >> 8)

    def set_all_pwm(self, on, off):
        """Sets all PWM channels."""
        self.bus.write_byte_data(self.address, self.ALL_LED_ON_L, on & 0xFF)
        self.bus.write_byte_data(self.address, self.ALL_LED_ON_H, on >> 8)
        self.bus.write_byte_data(self.address, self.ALL_LED_OFF_L, off & 0xFF)
        self.bus.write_byte_data(self.address, self.ALL_LED_OFF_H, off >> 8)

if __name__ == '__main__':

    # Simple demo of of the PCA9685 PWM servo/LED controller library.
    # This will move channel 0 from min to max position repeatedly.
    # Author: Tony DiCola
    # License: Public Domain

    # Initialise the PCA9685 using the default address (0x40).
    pwm = pca9685()

    # Alternatively specify a different address and/or bus:
    #pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

    # Configure min and max servo pulse lengths
    servo_min = 150  # Min pulse length out of 4096
    servo_max = 600  # Max pulse length out of 4096

    # Helper function to make setting a servo pulse width simpler.
    def set_servo_pulse(channel, pulse):
        pulse_length = 1000000    # 1,000,000 us per second
        pulse_length //= 60       # 60 Hz
        print('{0}us per period'.format(pulse_length))
        pulse_length //= 4096     # 12 bits of resolution
        print('{0}us per bit'.format(pulse_length))
        pulse *= 1000
        pulse //= pulse_length
        pwm.set_pwm(channel, 0, pulse)

    # Set frequency to 60hz, good for servos.
    pwm.set_pwm_freq(60)

    print('Moving servo on channel 0, press Ctrl-C to quit...')
    while True:
        # Move servo on channel O between extremes.
        pwm.set_pwm(0, 0, servo_min)
        time.sleep(1)
        pwm.set_pwm(1, 0, servo_min)
        time.sleep(1)
        pwm.set_pwm(0, 0, servo_max)
        time.sleep(1)
        pwm.set_pwm(1, 0, servo_max)
        time.sleep(1)