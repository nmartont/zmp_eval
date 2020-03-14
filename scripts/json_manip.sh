#!/usr/bin/env bash
for entry in "json_data"/*
do
  python json_manip.py "$entry"
done
