#!/usr/bin/python
# -*- coding: utf-8 -*-
# Author: Marton

from ..suffix import *
import pytest


@pytest.mark.parametrize('number, expected', [
    # Test all supported suffixes
    (0,       "0"),
    (10**0,   "1"),
    (10**3,   "1 Kilo"),
    (10**6,   "1 Mega"),
    (10**9,   "1 Giga"),
    (10**12,  "1 Tera"),
    (10**15,  "1 Peta"),
    (10**18,  "1 Exa"),
    (10**21,  "1 Zetta"),
    (10**24,  "1 Yotta"),
    (10**-3,  "1 Milli"),
    (10**-6,  "1 Micro"),
    (10**-9,  "1 Nano"),
    (10**-12, "1 Pico"),
    (10**-15, "1 Femto"),
    (10**-18, "1 Atto"),
    (10**-21, "1 Zepto"),
    (10**-24, "1 Yocto"),
    # Test fractions
    (1234.56789, "1.23456789 Kilo"),
    # Test negative values
    (-1234567, "-1.234567 Mega"),
    # Tests from the assignment
    (123,      "123"),
    (1234,     "1.234 Kilo"),
    (12345,    "12.345 Kilo"),
    (1234567,  "1.234567 Mega"),
    (12345678, "12.345678 Mega"),
    # Numbers between 0 and 1
    (0.1,        "100 Milli"),
    (0.001,      "1 Milli"),
    (0.00000123, "1.23 Micro"),
    # Error handling
    (None, "ERROR"),
    ("1234", "ERROR")
])
def test_suffixWithUnit(number, expected):
    assert suffixWithUnit(number) == expected
