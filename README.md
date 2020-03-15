# Evaluation test for ZMP

This repository contains my attempts at solving the recruitment tasks given by ZMP.

## Introduction

My name is Márton, I currently work and live in Japan, and I am originally from Hungary.
I am a robotics engineer, with an MSc in Mechatronics Engineering.
I am currently working on a case picking palletizing system,
and my tasks range from designing and implementing the box stacking algorithm,
to working on the ROS-based robot system.

Your colleague Yuuji (I call him Abu) introduced me to your company.
He and I used to work for the same company, he was doing vision and I was doing the ROS-system for a piece picking robot.
He painted ZMP in a very good light, and your autonomous vehicle projects seem very interesting to me.
So I am excited to give this a shot, and thank you very much for giving me a chance.

My main relevant skill-set revolves around ROS-based industrial robot systems.
I have experience in the architecture design of ROS-systems,
and also Python/C++ software implementation, motion planning, testing, continuous integration, etc.
I also have a vast experience in software engineering.
I have carried out many projects from the ground up,
during which I picked up many good practices for clean code.

The rest of this README will detail each completed task.

## Setup

For the python scripts, please install the libraries inside `requirements.txt`.
A `Python 3.5` virtual environment is intended for script execution, but `Python 2.7` should also work fine.

For the ground plane task, the `pcl` library is necessary for making the project,
along with a C++14 compiler.

The python unit tests will always contain the example data from the assignment pdf.

## Number with Suffix

#### Usage
```bash
python scripts/suffix.py [-h] number

positional arguments:
  number      The number of which to return the string representation, using
              suffixes.
```

Running tests:
```bash
pytest scripts/tests/test_suffix.py
```

#### Description

I found this task quite interesting, so I tried to make the solution as universal as possible.
* Possible to define any number of suffixes (in the source file, I defined from Yocto to Yotta).
* Integers and fractions are both supported
* Negative numbers are also supported
* Error handling
* Unittesting

#### Examples
```bash
python scripts/suffix.py 123456.789  ----> 123.456789 Kilo
python scripts/suffix.py -0.00012345 ----> -123.45 Micro
```

## JSON manipulation

#### Usage
```bash
python scripts/json_manip.py [-h] path

positional arguments:
  path        The path to the JSON_A file to be converted to JSON_B
```

Running tests:
```bash
pytest scripts/tests/test_json_manip.py
```

#### Description

Fairly simple script written in Python, I tried to write it as cleanly as possible.
* Format checking
* Error handling
* Unittesting

#### Bash scripting

I wrote the following bash script to convert json files in the `json_data` folder:

```bash
#!/usr/bin/env bash
for entry in "json_data"/*.json
do
  [[ -f "$entry" ]] || break
  python json_manip.py "$entry"
done
```

This script will only process files with `*.json` extension.
The original files will be overwritten.

How to make it faster: parallel execution of the `for loop` inside the bash script is possible.
That way, the conversion tasks don't need to wait for each other, and the runtime can be greatly reduced.

Parallelly executing script:
```bash
#!/usr/bin/env bash
for entry in "json_data"/*.json
do
  [[ -f "$entry" ]] || break
  python json_manip.py "$entry" &
done
```

## Ground Plane Detection

% TODO