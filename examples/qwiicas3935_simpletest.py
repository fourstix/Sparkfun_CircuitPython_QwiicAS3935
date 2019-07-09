#  This is example is for the SparkFun Qwiic Relay.
#  SparkFun sells these at its website: www.sparkfun.com
#  Do you like this library? Help support SparkFun. Buy a board!
#  https://www.sparkfun.com/products/15168

"""
 Qwiic AS3935 Lightning Detector Simple Test - qwiicas3935_simpletest.py
 Written by Gaston Williams, July 4th, 2019
 The Qwiic AS3935 is an I2C controlled lightning detector.

 Simple Test:
 This program uses the Qwiic AS3935 CircuitPython Library to check
 that status of the Qwiic AS3935 Lightning Detector.
"""

import board
import busio
import sparkfun_qwiicas3935

# Create bus object using our board's I2C port
i2c = busio.I2C(board.SCL, board.SDA)
# Create an as3935 library object
lightning = sparkfun_qwiicas3935.Sparkfun_QwiicAS3935_I2C(i2c)

# OR create a library object using the Bus SPI port
# spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
# cs = digitalio.DigitalInOut(board.D8)
# lightning = sparkfun_qwiicas3935.Sparkfun_QwiicAS3935_SPI(spi, cs)

# Check if connected
if lightning.connected:
    print('AS3935 Lightning Detector connected.')
else:
    print('Lightning Detector does not appear to be connected. Please check wiring.')
    exit()

# Read the Lightning Detector AFE mode and print it out.
mode = lightning.indoor_outdoor

if mode == lightning.OUTDOOR:
    print('The Lightning Detector is in the Outdoor mode.')
elif mode == lightning.INDOOR:
    print('The Lightning Detector is in the Indoor mode.')
else:
    print('The Lightning Detector is in an Unknown mode.')
