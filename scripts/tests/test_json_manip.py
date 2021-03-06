#!/usr/bin/python
# -*- coding: utf-8 -*-
# Author: Marton

from ..json_manip import *
import pytest


@pytest.mark.parametrize('json_a, json_b', [
    # Simple test case
    ({"locker_1_door": 1,
      "locker_5_unlock": 2,
      "locker_1_item": 1},
     {"1": {"door": 1, "item": 1},
      "5": {"unlock": 2}}),
    # Empty dictionaries
    ({}, {}),
    # Error handling
    ({"lol_5_light": "ayy"}, "Category error"),
    ({"lol_5_light_wontwork": "hahayes"}, "Format error"),
    ({"locker_5_hey": None}, "Attribute error"),
    # Assignment test
    ({"locker_5_light": 1,
      "locker_5_unlock": 2,
      "locker_6_light": 1,
      "locker_6_unlock": 1,
      "locker_1_door": 1,
      "locker_1_item": 1,
      "locker_2_door": 1,
      "locker_2_item": 2,
      "locker_5_door": 1,
      "locker_5_item": 5,
      "locker_6_door": 1,
      "locker_6_item": 1,
      "locker_1_light": 0,
      "locker_1_unlock": 1,
      "locker_2_light": 1,
      "locker_2_unlock": 1,
      "locker_3_light": 1,
      "locker_3_unlock": 1,
      "locker_4_light": 1,
      "locker_4_unlock": 1,
      "locker_3_door": 1,
      "locker_3_item": 3,
      "locker_4_door": 1,
      "locker_4_item": 0},
     {'1': {'door': 1,
            'item': 1,
            'light': 0,
            'unlock': 1},
      '2': {'door': 1,
            'item': 2,
            'light': 1,
            'unlock': 1},
      '3': {'door': 1,
            'item': 3,
            'light': 1,
            'unlock': 1},
      '4': {'door': 1,
            'item': 0,
            'light': 1,
            'unlock': 1},
      '5': {'door': 1,
            'item': 5,
            'light': 1,
            'unlock': 2},
      '6': {'door': 1,
            'item': 1,
            'light': 1,
            'unlock': 1}})
])
def test_convert_JSON_A_to_JSON_B(json_a, json_b):
    assert convert_JSON_A_to_JSON_B(json_a) == json_b
