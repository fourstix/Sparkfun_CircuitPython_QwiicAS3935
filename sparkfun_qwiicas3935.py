# The MIT License (MIT)
#
# Copyright (c) 2019 Gaston Williams
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
"""
`sparkfun_qwiicas3935`
================================================================================

CircuitPython driver library for the Sparkfun AS3935 Lightning Detector


* Author(s): Gaston Williams

* Based on the Arduino library for the Qwiic AS3935 Lightning Detector
  Written by Elias Santistevan @ SparkFun Electronics in January, 2019

* I2C Register read functions based on code written by Phil Fenstermacher
  on December, 2014 as part of the RPi_AS3935 python library.

Implementation Notes
--------------------

**Hardware:**

*  This is library is for the SparkFun Qwiic AS3935 Lightning Detector.
*  SparkFun sells these at its website: www.sparkfun.com
*  Do you like this library? Help support SparkFun. Buy a board!
   https://www.sparkfun.com/products/15276

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
"""

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/fourstix/Sparkfun_CircuitPython_QwiicAS3935.git"

# imports
from abc import ABC, abstractmethod
from time import sleep
from micropython import const

# public constants
DEFAULT_I2C_ADDR = const(0x03)
"""Default I2C address for AS3935"""
# private constants

_DIRECT_COMMAND = const(0x96)

# data registers

_AFE_GAIN = const(0x00)
_THRESHOLD = const(0x01)
_LIGHTNING_REG = const(0x02)
_INT_MASK_ANT = const(0x03)
_ENERGY_LIGHT_LSB = const(0x04)
_ENERGY_LIGHT_MSB = const(0x05)
_ENERGY_LIGHT_MMSB = const(0x06)
_DISTANCE = (0x07)
_FREQ_DISP_IRQ = (0x08)

# calibration registers
_CALIB_TRCO = const(0x3A)
_CALIB_SRCO = const(0x3B)
_DEFAULT_RESET = const(0x3C)
_CALIB_RCO = const(0x3D)

# bit mask constants

_POWER_MASK = const(0x01)
_GAIN_MASK = const(0x3E)
_SPIKE_MASK = const(0x0F)
_IO_MASK = const(0xC1)
_DISTANCE_MASK = const(0xC0)
_INT_MASK = const(0xF0)
_THRESH_MASK = const(0x0F)
_R_SPIKE_MASK = const(0xF0)
_ENERGY_MASK = const(0xF0)
_CAP_MASK = const(0x0F)
_LIGHT_MASK = const(0xCF)
_DISTURB_MASK = const(0xDF)
_NOISE_FLOOR_MASK = const(0x70)
_OSC_MASK = const(0xE0)
_SPI_READ_MASK = const(0x40)
_CALIB_MASK = const(0x7F)
_DIV_MASK = const(0x3F)

# abstract base class
class Sparkfun_QwiicAS3935(ABC):
    """Abstract base class for Sparkfun AS3935 Lightning Detector"""
    # pylint: disable=too-many-instance-attributes

    # class constants
    INDOOR = const(0x12)
    """AS3235 AFE setting for indoor use."""
    OUTDOOR = const(0x0E)
    """AS3235 AFE setting for outdoor use."""

    # interrupt status values
    NOISE = const(0x01)
    """Interrupt register value for a noise too high interrupt."""
    DISTURBER = const(0x04)
    """Interrupt register value for a disturber detected interrupt."""
    LIGHTNING = const(0x08)
    """Interrupt register value for a lightning detected interrupt."""

    # Antenna frequency
    ANTENNA_FREQ = const(0x03)
    """Select the Antenna oscillator frequency"""

    def __init__(self, debug=False):
        self._debug = debug

    def power_down(self):
        """This breakout board consumes 1-2uA while powered down.
        If the board is powered down then the TRCO will need to be recalibrated:
        REG0x08[5] = 1, wait 2 ms, REG0x08[5] = 0.
        SPI and I2-C remain active when the chip is powered down."""
        #REG0x00, bit[0], manufacturer default: 0.
        self._write_register_bits(_AFE_GAIN, _POWER_MASK, 1, 0)

    def wake_up(self):
        """This register holds the state of the timer RC oscillator (TRCO),
        after it has been calibrated. The TRCO will need to be recalibrated
        after power down. The following function wakes the IC, sends the Direct Command to
        CALIB_RCO register REG0x3D, waits a bit and then checks that it has been successfully
        calibrated. Note that SPI and I2C are active during power down."""
        # REG0x3A bit[7].

        # Set the power down bit to zero to wake it up
        self._write_register_bits(_AFE_GAIN, _POWER_MASK, 0, 0)
        # Send command to calibrate the oscillators
        self._write_register(_CALIB_RCO, _DIRECT_COMMAND)
        # Give time for the internal oscillators to start up.
        sleep(0.002)

        # According to data sheet pg 23, section 8.11, one must
        # write a 1 to reg 0x08 bit 5 wait 2ms and then write a zero
        self._write_register_bits(_FREQ_DISP_IRQ, _OSC_MASK, 1, 5)
        sleep(0.002)
        self._write_register_bits(_FREQ_DISP_IRQ, _OSC_MASK, 0, 5)
        sleep(0.002)

        # Check callibration
        calibrated = self._read_byte(_CALIB_SRCO) & _CALIB_MASK

        return bool(calibrated)

    def clear_statistics(self):
        """This register clears the number of lightning strikes that has been
         read in the last 15 minute block."""
        # REG0x02, bit [6], manufacturer default: 1.
        mask = 1 << 6
        # Write high, then low, then high to clear.
        self._write_register_bits(_LIGHTNING_REG, mask, 1, 6)
        self._write_register_bits(_LIGHTNING_REG, mask, 0, 6)
        self._write_register_bits(_LIGHTNING_REG, mask, 1, 6)

    def read_interrupt_register(self):
        """When there is an event that exceeds the watchdog threshold, the
        register is written with the type of event. This consists of two
        messages: INT_D (disturber detected) and INT_L (Lightning detected).
        A third interrupt INT_NH (noise level too HIGH) indicates that the
        noise level has been exceeded and will persist until the noise has
        ended. Events are active HIGH. There is a one second window of time to
        read the interrupt register after lightning is detected, and 1.5
        seconds after disturber."""
        # REG0x03, bits [3:0], manufacturer default: 0.

        #A 20 ms delay is added to allow for the memory register to be populated
        # after the interrupt pin goes HIGH. See "Interrupt Management" in datasheet.
        sleep(0.020)

        value = self._read_register(_INT_MASK_ANT)
        # Only need the first four bits [3:0]
        value &= ~_INT_MASK
        return value

    def display_oscillator(self, state, osc):
        """This will send the frequency of the oscillators to the IRQ pin.
        osc 1, bit[5] = TRCO - Timer RCO Oscillators 1.1MHz
        osc 2, bit[6] = SRCO - System RCO at 32.768kHz
        osc 3, bit[7] = LCO - Frequency of the Antenna
        State is True/False for on/off and
        Oscillator value must be between 1 and 3"""
        # REG0x08, bits [5,6,7], manufacturer default: 0.
        # Check the state to turn on or off
        if state:
            value = 1
        else:
            value = 0

        if osc == 1:
            self._write_register_bits(_FREQ_DISP_IRQ, _OSC_MASK, value, 5)
        elif osc == 2:
            self._write_register_bits(_FREQ_DISP_IRQ, _OSC_MASK, value, 6)
        elif osc == 3:
            self._write_register_bits(_FREQ_DISP_IRQ, _OSC_MASK, value, 7)
        else:
            raise ValueError('Oscillator value must be between 1 and 3')

    def reset(self):
        """Reset all the device registers to initial power-on default values"""
        self._write_register(_DEFAULT_RESET, _DIRECT_COMMAND)

    def calibrate(self):
        """Send command to calibrate the oscillators"""
        self._write_register(_CALIB_RCO, _DIRECT_COMMAND)
        # Give time for the internal oscillators to start up.
        sleep(0.002)

        # According to data sheet pg 23, section 8.11, one must
        # write a 1 to reg 0x08 bit 5 wait 2ms and then write a zero
        self._write_register_bits(_FREQ_DISP_IRQ, _OSC_MASK, 1, 5)
        sleep(0.002)
        self._write_register_bits(_FREQ_DISP_IRQ, _OSC_MASK, 0, 5)
        sleep(0.002)

        # Check callibration
        calibrated = self._read_byte(_CALIB_SRCO) & _CALIB_MASK

        return bool(calibrated)

    # properites (read-only)

    @property
    def distance_to_storm(self):
        """Returns the distance to the front of the storm and not the
        distance to a particular lightning strike."""
        # REG0x07, bit [5:0], manufacturer default: 0.
        distance = self._read_register(_DISTANCE)
        return distance & ~_DISTANCE_MASK

    @property
    def lightning_energy(self):
        """This returns a 20 bit value that represents the energy of the
        lightning strike. According to the datasheet this is only a pure value
        that doesn't have any physical meaning."""
        # LSB =  REG0x04, bits[7:0]
        # MSB =  REG0x05, bits[7:0]
        # MMSB = REG0x06, bits[4:0]
        value = self._read_register(_ENERGY_LIGHT_MMSB)
        # Only first four bits of MMSB are valid
        value &= ~_ENERGY_MASK
        energy = value << 16
        # Get the MSB
        value = self._read_register(_ENERGY_LIGHT_MSB)
        energy |= value << 8
        # Get the LSB
        value = self._read_register(_ENERGY_LIGHT_LSB)
        energy |= value
        return energy

    @property
    def connected(self):
        """Verify the AS3935 is connected to the bus."""
        value = self._read_register(_AFE_GAIN)
        value &= ~_IO_MASK
        mode = value >> 1
        return mode == self.INDOOR or mode == self.OUTDOOR

    # properties (read-write)

    @property
    def indoor_outdoor(self):
        """This funciton changes toggles the chip's analog front end settings
         for Indoors or Outdoors operation."""
        # REG0x00, bits [5:1], manufacturer default: 10010 (INDOOR).
        value = self._read_register(_AFE_GAIN)
        value &= ~_IO_MASK
        return value >> 1

    @indoor_outdoor.setter
    def indoor_outdoor(self, value):
        """This funciton changes the chip's settings for Indoors and Outdoors.
        Only values of either INDOOR (0x12) or OUTDOOR (OX0E) are allowed."""
        # REG0x00, bits [5:1], manufacturer default: 10010 (INDOOR).
        if value == self.INDOOR:
            self._write_register_bits(_AFE_GAIN, _GAIN_MASK, self.INDOOR, 1)
        elif value == self.OUTDOOR:
            self._write_register_bits(_AFE_GAIN, _GAIN_MASK, self.OUTDOOR, 1)
        else:
            raise ValueError("Only values of either INDOOR (0x12) or OUTDOOR (OX0E) are allowed.")

    @property
    def watchdog_threshold(self):
        """REG0x01, bits[3:0], manufacturer default: 0010 (2).
        This function returns the threshold for events that trigger the IRQ Pin."""
        value = self._read_register(_THRESHOLD)
        return value & _THRESH_MASK

    @watchdog_threshold.setter
    def watchdog_threshold(self, value):
        """This function returns the threshold for events that trigger the IRQ
         Pin. Only sensitivity threshold values 1 to 10 allowed."""
        # REG0x01, bits[3:0], manufacturer default: 0010 (2).
        if value < 1 or value > 10:
            raise ValueError('Only sensitivity threshold values 1 to 10 allowed.')

        self._write_register_bits(_THRESHOLD, _THRESH_MASK, value, 0)

    @property
    def noise_level(self):
        """This function will return the value of noise level threshold. The
        default is 2."""
        # REG0x01, bits [6:4], manufacturer default: 010 (2).
        value = self._read_register(_THRESHOLD)
        value &= _NOISE_FLOOR_MASK
        return value >> 4

    @noise_level.setter
    def noise_level(self, value):
        """The noise floor level is compared to a known reference voltage. If
        this level is exceeded the chip will issue an interrupt to the IRQ pin,
        broadcasting that it can not operate properly due to noise (INT_NH).
        Check datasheet for specific noise level tolerances when setting this
         register."""
        # REG0x01, bits [6:4], manufacturer default: 010 (2).
        if value < 1 or value > 7:
            raise ValueError('Only noise levels of 1 to 7 are allowed.')

        self._write_register_bits(_THRESHOLD, _NOISE_FLOOR_MASK, value, 4)

    @property
    def spike_rejection(self):
        """Return the value of the spike rejection register. This value helps
        to differentiate between events and acutal lightning, by analyzing the
        shape of the spike during chip's signal validation routine. Increasing
        this value increases robustness at the cost of sensitivity to distant
        events."""
        # REG0x02, bits [3:0], manufacturer default: 0010 (2).
        value = self._read_register(_LIGHTNING_REG)
        value &= ~_R_SPIKE_MASK
        return value


    @spike_rejection.setter
    def spike_rejection(self, value):
        """This setting, like the watchdog threshold, can help determine
        between false events and actual lightning. The shape of the spike is
        analyzed during the chip's signal validation routine. Increasing this
        value increases robustness at the cost of sensitivity to distant
        events. The Spike rejection value must be from 1 to 11."""
        # REG0x02, bits [3:0], manufacturer default: 0010 (2).
        if value < 1 or value > 11:
            raise ValueError("Spike rejection value must be from 1 to 11.")

        self._write_register_bits(_LIGHTNING_REG, _SPIKE_MASK, value, 0)

    @property
    def lightning_threshold(self):
        """This function will return the number of lightning strikes must
        strike within a 15 minute window before it triggers an event on the
        IRQ pin. Default is 1."""
        # REG0x02, bits [5:4], manufacturer default: 0 (single strike).
        value = self._read_register(_LIGHTNING_REG)
        value &= ~_LIGHT_MASK
        value >>= 4

        # calculate the number of strikes based on two-bit value
        if value == 0:
            strikes = 1
        elif value == 1:
            strikes = 5
        elif value == 2:
            strikes = 9
        else:
            strikes = 16

        return strikes

    @lightning_threshold.setter
    def lightning_threshold(self, value):
        """The number of lightning events before IRQ is set high. 15 minutes
        is the window of time before the number of detected lightning events is
        reset. The number of lightning strikes can only be set to 1, 5, 9 or 16."""
        # REG0x02, bits [5:4], manufacturer default: 0 (single strike).
        # bit mask 0x30
        mask = (1<<5)|(1<<4)

        if value == 1:
            self._write_register_bits(_LIGHTNING_REG, mask, 0, 4)
        elif value == 5:
            self._write_register_bits(_LIGHTNING_REG, mask, 1, 4)
        elif value == 9:
            self._write_register_bits(_LIGHTNING_REG, mask, 1, 5)
        elif value == 16:
            self._write_register_bits(_LIGHTNING_REG, mask, 3, 4)
        else:
            raise ValueError('Lightning threshold can only be set to 1, 5, 9 or 16.')

    @property
    def mask_disturber(self):
        """This setting will return whether or not disturbers trigger the
        IRQ pin."""
        # REG0x03, bit [5], manufacturere default: 0.
        value = self._read_register(_INT_MASK_ANT)
        value &= ~_DISTURB_MASK
        return value >> 5


    @mask_disturber.setter
    def mask_disturber(self, value):
        """Setting this True or False will change whether or not disturbers
        trigger the IRQ pin."""
        # REG0x03, bit [5], manufacturere default: 0.
        # bit mask 0x10
        mask = 1<<5
        if value:
            self._write_register_bits(_INT_MASK_ANT, mask, 1, 5)
        else:
            self._write_register_bits(_INT_MASK_ANT, mask, 0, 5)

    @property
    def division_ratio(self):
        """This function returns the current division ratio of the resonance frequency.
        The antenna resonance frequency should be within 3.5 percent of 500kHz, and
        so when modifying the resonance frequency with the internal capacitors
        (tuneCap()) it's important to keep in mind that the displayed frequency on
        the IRQ pin is divided by this number."""
        # REG0x03, bit [7:6], manufacturer default: 0 (16 division ratio).
        value = self._read_register(_INT_MASK_ANT)
        value &= ~_DIV_MASK

        # translate the upper two-bit values to the division ratio
        if value == 0:
            ratio = 16
        elif value == 0x40:
            ratio = 32
        elif value == 0x80:
            ratio = 64
        else:
            ratio = 128

        return ratio

    @division_ratio.setter
    def division_ratio(self, value):
        """The antenna is designed to resonate at 500kHz and so can be tuned
        with the following setting. The accuracy of the antenna must be within
        3.5 percent of that value for proper signal validation and distance
        estimation. The division ratio can only be set to 16, 32, 64 or 128."""
        # REG0x03, bit [7:6], manufacturer default: 0 (16 division ratio).
        # bit mask 0xC0
        mask = (1<<7)|(1<<6)

        if value == 16:
            self._write_register_bits(_INT_MASK_ANT, mask, 0, 6)
        elif value == 32:
            self._write_register_bits(_INT_MASK_ANT, mask, 1, 6)
        elif value == 64:
            self._write_register_bits(_INT_MASK_ANT, mask, 1, 7)
        elif value == 128:
            self._write_register_bits(_INT_MASK_ANT, mask, 3, 6)
        else:
            raise ValueError('The division ratio can only be set to 16, 32, 64 or 128.')

    @property
    def tune_cap(self):
        """This setting will return the capacitance of the internal capacitors. It will
        return a value from one to 15 multiplied by the 8pF steps of the internal
        capacitance."""
        # REG0x08, bits [3:0], manufacturer default: 0.

        value = self._read_register(_FREQ_DISP_IRQ)
        value &= _CAP_MASK
        # Tune cap is 4-bit value multiplied by 8pF
        return value * 8


    @tune_cap.setter
    def tune_cap(self, value):
        """This setting will add capacitance to the series RLC antenna on the
        product. It's possible to add 0-120pF in steps of 8pF to the antenna.
        The Tuning Cap value must be between 0 and 15."""
        # REG0x08, bits [3:0], manufacturer default: 0.
        if value < 0 or value > 15:
            raise ValueError('The Tuning Cap value must be between 0 and 15.')

        self._write_register_bits(_FREQ_DISP_IRQ, _CAP_MASK, value, 0)

    # abstract methods
    @abstractmethod
    def _read_register(self, register):
        pass

    @abstractmethod
    def _read_byte(self, register):
        pass

    @abstractmethod
    def _write_register(self, register, value):
        pass

    # private functions

    def _write_register_bits(self, reg, mask, bits, start_position):
        # Mask the part of the register value that coincides with the setting,
        # then write the given bits to the register at the given start position.

        # Get the current value of the register
        value = self._read_register(reg)
        # Mask the position we want to write to
        value &= (~mask)
        # Set the given bits in the variable
        value |= (bits << start_position)
        # Write the updated variable value back to the register
        self._write_register(reg, value)

# concrete subclass for I2C
class Sparkfun_QwiicAS3935_I2C(Sparkfun_QwiicAS3935):
    """Driver for Sparkfun AS3935 Lightning Detector over I2C"""
    def __init__(self, i2c, address=DEFAULT_I2C_ADDR, debug=False):
        import adafruit_bus_device.i2c_device as i2c_device
        self._i2c = i2c_device.I2CDevice(i2c, address)
        super().__init__(debug)

    def _read_register(self, register):
        # Read a data register in the 0x00 to 0x08 range.
        # Use read_byte function to read registers above this range.
        if register < _AFE_GAIN or register > _FREQ_DISP_IRQ:
            raise ValueError("Register value must be in the range of 0x00 to 0x08")

        with self._i2c as i2c:
            i2c.write(bytes([0x00]), stop=False)
            # Write to the base address, then read all data registers in a
            # single block read. Then return the desired value from the list.
            # Successive individual byte reads, tend to fail. This trick
            # was taken from pcfens/RPi-AS335 python libray on github.
            #
            # In the original commments, Phil Fenstermacher (pcfens) says this
            # trick is required because smbus doesn't support repeated I2C
            # starts to read the registers directly (singularly) on the sensor.
            result = bytearray(9)
            i2c.readinto(result)
            if self._debug:
                print([hex(i) for i in result])
                print("$%02X => %s" % (register, hex(result[register])))

            return result[register]

    def _read_byte(self, register):
        # Read all the registers and get byte values above 0x08.  This range
        # contains the lightning look-up tables and calibration registers.
        # The read_register is more efficent for more frequent data registers.
        with self._i2c as i2c:
            i2c.write(bytes([0x00]), stop=False)
            # Write to the base address, then read all data registers in a
            # single block read. Then return the desired value from the list.
            # Successive individual byte reads, tend to fail. This trick
            # was taken from pcfens/RPi-AS335 python libray on github.
            # Note that to get to the calibration registers, we have to read
            # 63 bytes (ranging from 0x00 to 0x3D).
            #
            # In the original commments, Phil Fenstermacher (pcfens) says this
            # trick is required because smbus doesn't support repeated I2C
            # starts to read the registers directly (singularly) on the sensor.
            result = bytearray(0x3E)
            i2c.readinto(result)
            if self._debug:
                print([hex(i) for i in result])
                print("$%02X => %s" % (register, hex(result[register])))
        return result[register]

    def _write_register(self, register, value):
        with self._i2c as i2c:
            i2c.write(bytes([register & 0xFF, value & 0xFF]))
            if self._debug:
                print("$%02X <= 0x%02X" % (register, value))

# concrete subclass for SPI
class Sparkfun_QwiicAS3935_SPI(Sparkfun_QwiicAS3935):
    """Driver for Sparkfun AS3935 Lightning Detector over SPI"""
    def __init__(self, spi, cs, debug=False):
        # We can't use SPIDevice becasue of the required cha-cha-cha
        # on the CS line (CS=High, Low, High) after each read
        self._spi = spi
        # needed for managing the spi read/writes
        self._cs = cs
        # set cs line high initially
        self._cs.value = True
        super().__init__(debug)


    def _read_register(self, register):
        # set the address read bits
        addr = (register | _SPI_READ_MASK) & 0xFF
        try:
            while not self._spi.try_lock():
                pass

            # configure for SPI mode 1
            self._spi.configure(baudrate=2000000, phase=1, polarity=0)
            # start the read
            self._cs.value = False
            # write msb first
            self._spi.write(bytearray([addr]))  #pylint: disable=no-member
            # read the next byte afte writing the address
            result = bytearray(1)
            self._spi.readinto(result)
            if self._debug:
                print("$%02X => %s" % (register, [hex(i) for i in result]))

            return result[0]
        #the finally block always executes before return
        finally:
            # per datasheet CS = HIGH, LOW, HIGH signals the end of a read
            self._cs.value = True
            self._cs.value = False
            self._cs.value = True
            self._spi.unlock()

    def _read_byte(self, register):
        # For SPI read_byte function is same as read_register register
        return self._read_register(register)

    def _write_register(self, register, value):
        register &= 0x3F  # Write, bit 7, 6  low.

        try:
            while not self._spi.try_lock():
                pass

            # configure for SPI mode 1
            self._spi.configure(baudrate=2000000, phase=1, polarity=0)
            # start the write
            self._cs.value = False
            # write msb first
            self._spi.write(bytearray([register, value]))  #pylint: disable=no-member

            if self._debug:
                print("$%02X <= 0x%02X" % (register, value))

        # the finally block always executes before return
        finally:
            # raise the cs line once we are done and release the spi bus
            self._cs.value = True
            self._spi.unlock()
