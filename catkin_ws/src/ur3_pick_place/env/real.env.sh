#!/usr/bin/env bash
# Source before roslaunching against the real arm:
#   source env/real.env.sh && roslaunch ur3_pick_place static/real.launch
# Read by launch/common/*.launch via $(optenv ...).
#
# Values below match the socialab lab setup; override per-site.

export ROS_MASTER_URI=http://localhost:11311   # roscore runs on this machine
export ROS_IP=192.168.0.10                     # this machine's IP on the robot's subnet

export UR3_USE_SIM=false
export UR3_ROBOT_IP=192.168.0.2                # UR3 controller IP
# Per-robot calibration file produced once via:
#   roslaunch ur_calibration calibration_correction.launch \
#     robot_ip:=$UR3_ROBOT_IP target_filename:="$UR3_KINEMATICS_CONFIG"
export UR3_KINEMATICS_CONFIG="$HOME/ur3_calibration.yaml"
