#!/usr/bin/python
# -*- coding: utf-8 -*-
# Author: Marton

import argparse
import os
import sys
import json

CATEGORY_TEXT = "locker"
ATTRIBUTES = ["light", "unlock", "door", "item"]
SEPARATOR = "_"


def convert_JSON_A_to_JSON_B(json_a):
    """Converts the given dictionary from format JSON_A to JSON_B.

    JSON_A:
    {"locker_5_light": 1,
     "locker_5_unlock": 2,
     "locker_6_light": 1,
     "locker_6_unlock": 1,
     "locker_1_door": 1,
     ...}

    JSON_B:
    {'1': {door: 1, item: 1, light: 0, unlock: 1},
     '2': {door: 1, item: 2, light: 1, unlock: 1},
      ...}

    Args:
        json_a (dict): Dictionary in JSON_A format

    Returns:
        dict or str: Dictionary in JSON_B format, converted from json_a; or error string if the conversion failed.
    """
    json_b = dict()

    # Iterate over key/value pairs in json_a. FIXME this syntax only works on Python3
    for key, value in json_a.items():
        try:
            # split key by separator
            category, number, attribute = tuple(key.split(sep=SEPARATOR))
        except ValueError:
            return "Format error"

        if category != CATEGORY_TEXT:
            return "Category error"

        if attribute not in ATTRIBUTES:
            return "Attribute error"

        # If the given number is not in the json_b dict already, put a new dict under the number key
        if number not in json_b:
            json_b[number] = dict()

        # Get the current sub-dictionary for the current number key
        current = json_b[number]

        # Put the attribute and value into the current dictionary
        current[attribute] = value

    return json_b


if __name__ == "__main__":
    # Parse the command line argument for the file name
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=str,
                        help="The path to the JSON_A object to be converted to JSON_B")
    args = parser.parse_args()
    path = args.path

    # Check if file exists
    if not os.path.exists(path):
        print("File does not exist: {}".format(path))
        sys.exit(os.EX_IOERR)

    # Print out current filename, for easy "debugging"
    print("Working on file {}.....".format(path))

    # Load file
    with open(args.path, "r") as f:
        # Decode JSON file
        try:
            a = json.load(f)
        except json.decoder.JSONDecodeError:
            print("JSON decode error")
            sys.exit(os.EX_DATAERR)

    # Convert JSON_A to JSON_B
    b = convert_JSON_A_to_JSON_B(a)

    # Error handling
    if type(b) is str:
        print(b)
        sys.exit(os.EX_SOFTWARE)

    # Dump json_b into the original file name
    with open(args.path, "w+") as f:
        json.dump(b, f, sort_keys=True, indent=2)

    print("Success!")
