Introduction
============
.. image:: https://readthedocs.org/projects/sparkfun-circuitpython-qwiicas3935/badge/?version=latest
    :target: https://sparkfun-circuitpython-qwiicas3935.readthedocs.io/en/latest/
    :alt: Documentation Status


.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord


.. image:: https://github.com/fourstix/Sparkfun_CircuitPython_QwiicAS3935/workflows/Build%20CI/badge.svg
    :target: https://github.com/fourstix/Sparkfun_CircuitPython_QwiicAS3935/actions
    :alt: Build Status


.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
    :target: https://github.com/psf/black
    :alt: Code Style: Black

CircuitPython library for Sparkfun Qwiic AS3935 Lightning Detector.  This library is ported from
`SparkFun Qwiic AS3935 Lightning Detector Arduino Library <https://github.com/sparkfun/SparkFun_AS3935_Lightning_Detector_Arduino_Library>`_

.. image:: https://cdn.sparkfun.com//assets/parts/1/3/7/5/1/15276-SparkFun_Lightning_Detector_-_AS3935__Qwiic_-01.jpg
    :target: https://www.sparkfun.com/products/15276
    :alt: SparkFun Qwiic AS3935 Lightning Detector (SEN-15276)

`SparkFun Qwiic AS3935 Lightning Detector (SEN-15276) <https://www.sparkfun.com/products/15276>`_



Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Adafruit Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
* `Sparkfun Qwiic AS3935 Lightning Detector Hardware <https://github.com/sparkfun/SparkFun_AS3935_Lightning_Detector>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_
or individual libraries can be installed using
`circup <https://github.com/adafruit/circup>`_.

Raspberry Pi Setup
------------------
Adafruit has an excellent tutorial on `Installing CircuitPython Libraries on Raspberry Pi
<https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi/>`_.

Quick Start Summary:

* Start with the latest version of Raspbian with Wifi configured.

* Enable SSH, I2C and SPI.

.. code-block:: shell

    sudo raspi-config

* Update your system to the latest version.

.. code-block:: shell

    sudo apt-get update
    sudo apt-get upgrade

* Update the python tools

.. code-block:: shell

    sudo pip3 install --upgrade setuptools

(If pip3 is not installed, install it and rerun the command)

.. code-block:: shell

    sudo apt-get install python3-pip

* Install the CircuitPython libraries

.. code-block:: shell

    pip3 install RPI.GPIO
    pip3 install adafruit-blinka

Installing from PyPI
=====================

On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/Sparkfun-circuitpython-qwiicas3935/>`_.
To install for current user:

.. code-block:: shell

    pip3 install sparkfun-circuitpython-qwiicas3935

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install sparkfun-circuitpython-qwiicas3935

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .env
    source .env/bin/activate
    pip3 install sparkfun-circuitpython-qwiicas3935



Installing to a Connected CircuitPython Device with Circup
==========================================================

Make sure that you have ``circup`` installed in your Python environment.
Install it with the following command if necessary:

.. code-block:: shell

    pip3 install circup

With ``circup`` installed and your CircuitPython device connected use the
following command to install:

.. code-block:: shell

    circup install qwiicas3935

Or the following command to update an existing version:

.. code-block:: shell

    circup update

Usage Example
=============
* `Qwiic AS3935 Lightning Detector Hookup Guide <https://learn.sparkfun.com/tutorials/sparkfun-qwiic-as3935-lightning-detector-hookup-guide>`_ - The Arduino examples in the Hookup Guide are available for Python with this library
* `CircuitPython on a Raspberry Pi <https://learn.adafruit.com/circuitpython-on-raspberrypi-linux>`_ - Basic information on how to install CircuitPython on a Raspberry Pi.

* Code Example:

.. code-block:: shell

   # import the CircuitPython board and busio libraries
   import board
   import sparkfun_qwiicas3935

   # Create bus object using the board's I2C port
   i2c = board.I2C()

   as3935 = QwiicAS3935_I2C(i2c)  # default address is 0x03

   # OR create a library object using the board's SPI port
   # spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
   # cs = digitalio.DigitalInOut(board.CE0)
   # cs.direction = digitalio.Direction.OUTPUT
   # as3935 = QwiicAS3935_SPI(spi, cs)

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/fourstix/Sparkfun_CircuitPython_QwiicAS3935/blob/HEAD/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Documentation
=============

For information on building library documentation, please check out
`this guide <https://learn.adafruit.com/creating-and-sharing-a-circuitpython-library/sharing-our-docs-on-readthedocs#sphinx-5-1>`_.

Building locally
================

Zip release files
-----------------

To build this library locally you'll need to install the
`circuitpython-build-tools <https://github.com/adafruit/circuitpython-build-tools>`_ package.

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install circuitpython-build-tools

Once installed, make sure you are in the virtual environment:

.. code-block:: shell

    source .env/bin/activate

Then run the build:

.. code-block:: shell

    circuitpython-build-bundles --filename_prefix sparkfun-circuitpython-qwiicas3935 --library_location .

Sphinx documentation
-----------------------

Sphinx is used to build the documentation based on rst files and comments in the code. First,
install dependencies (feel free to reuse the virtual environment from above):

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install Sphinx sphinx-rtd-theme

Now, once you have the virtual environment activated:

.. code-block:: shell

    cd docs
    sphinx-build -E -W -b html . _build/html

This will output the documentation to ``docs/_build/html``. Open the index.html in your browser to
view them. It will also (due to -W) error out on any warning like Travis will. This is a good way to
locally verify it will pass.

License Information
-----------------------
This product is **open source**!

Please review the LICENSE.md file for license information.

Please use, reuse, and modify these files as you see fit.

Please maintain the attributions to SparkFun Electronics and Adafruit and release any derivative under the same license.

Distributed as-is; no warranty is given.
