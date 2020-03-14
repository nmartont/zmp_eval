#!/usr/bin/python
# -*- coding: utf-8 -*-
# Author: Marton

import math
import argparse

suffixes = [
    (0,   ''),
    (3,   'Kilo'),
    (6,   'Mega'),
    (9,   'Giga'),
    (12,  'Tera'),
    (15,  'Peta'),
    (18,  'Exa'),
    (21,  'Zetta'),
    (24,  'Yotta'),
    (-3,  'Milli'),
    (-6,  'Micro'),
    (-9,  'Nano'),
    (-12, 'Pico'),
    (-15, 'Femto'),
    (-18, 'Atto'),
    (-21, 'Zepto'),
    (-24, 'Yocto'),
]
# It is assumed that the suffixes are sorted by the suffix exponent, in reverse
suffixes.sort(key=lambda x: x[0], reverse=True)


def suffixWithUnit(number):
    """
    Returns the string representation of the given number, using suffixes such as Kilo, Mega, etc

    suffixWithUnit(123) => 123
    suffixWithUnit(1234) => 1.234 Kilo
    suffixWithUnit(12345) => 12.345 Kilo
    suffixWithUnit(1234567) => 1.234567 Mega
    suffixWithUnit(12345678) => 12.345678 Mega

    Args:
        number (float): The number of which to return the string representation, using suffixes.

    Returns:
        str: the string representation of the given number, using suffixes
    """
    # 0 is a special case
    if number == 0:
        return "0"

    # log10 of the given number's absolute value
    try:
        log_10 = math.log10(math.fabs(number))
    except TypeError:
        # math.log10 raises TypeError if the given number cannot be log10-ed
        return "ERROR"

    # Choose the most fitting suffix for the given number, based on the log10 of the number
    # Note: this will work even if the suffixes are extended, eg. hecto, deca, deci, etc..
    suffix_exponent, suffix_name = next(((exp, name) for exp, name in suffixes
                                         if exp <= log_10),
                                        suffixes[-1])  # Default is the lowest possible suffix in the list

    # Divide the number with 10^suffix_exponent to get the number to display in the return string
    display_number = number/10**suffix_exponent

    # Display text is the display number, and the suffix name.
    # If display number%1==0, no decimals are shown. Otherwise all useful decimals are visible
    display_text = "{:.16g} {}".format(display_number, suffix_name).strip()  # Python float is accurate to 16 digits

    return display_text


if __name__ == "__main__":
    # Parse the command line argument for the number
    parser = argparse.ArgumentParser()
    parser.add_argument("number", type=float,
                        help="The number of which to return the string representation, using suffixes.")
    args = parser.parse_args()

    # Call the suffix function, print out result
    print(suffixWithUnit(args.number))
