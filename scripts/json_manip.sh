#!/usr/bin/env bash
for entry in "json_data"/*.json
do
  [[ -f "$entry" ]] || break
  python json_manip.py "$entry"
done
