# SPDX-FileCopyrightText: 2020 Diego Elio Pettenò
#
# SPDX-License-Identifier: Unlicense
"""A setuptools based setup module.

See:
https://packaging.python.org/en/latest/distributing.html
https://github.com/pypa/sampleproject
"""

from setuptools import setup, find_packages

# To use a consistent encoding
from codecs import open
from os import path

here = path.abspath(path.dirname(__file__))

# Get the long description from the README file
with open(path.join(here, "README.rst"), encoding="utf-8") as f:
    long_description = f.read()

setup(
    name="sparkfun-circuitpython-qwiicas3935",
    use_scm_version={
        # This is needed for the PyPI version munging in the Github Actions release.yml
        "git_describe_command": "git describe --tags --long",
        "local_scheme": "no-local-version",
    },
    setup_requires=["setuptools_scm"],
    description="CircuitPython driver library for the Sparkfun AS3935 Lightning Detector",
    long_description=long_description,
    long_description_content_type="text/x-rst",
    # The project's main homepage.
    url="https://github.com/fourstix/Sparkfun_CircuitPython_QwiicAS3935",
    # Author details
    author="Gaston Williams",
    author_email="fourstix@gmail.com",
    install_requires=[
        "Adafruit-Blinka",
        "adafruit-circuitpython-busdevice",
    ],
    # Choose your license
    license="MIT",
    # See https://pypi.python.org/pypi?%3Aaction=list_classifiers
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Topic :: Software Development :: Libraries",
        "Topic :: System :: Hardware",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.4",
        "Programming Language :: Python :: 3.5",
    ],
    # What does your project relate to?
    keywords="adafruit blinka circuitpython micropython qwiicas3935 sparkfun as3935 lightning",
    # You can just specify the packages manually here if your project is
    # simple. Or you can use find_packages().
    py_modules=["sparkfun_qwiicas3935"],
)

[tool.black]
target-version = ['py35']
