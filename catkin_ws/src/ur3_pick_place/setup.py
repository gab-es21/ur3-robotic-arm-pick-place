#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Exposes src/ur3_pick_place/ (e.g. gripper_interface.py) as an importable
# module shared by scripts/static/ and scripts/dynamic/, alongside the
# auto-generated ur3_pick_place.msg package.
d = generate_distutils_setup(
    packages=["ur3_pick_place"],
    package_dir={"": "src"},
)

setup(**d)
