#!/usr/bin/env bash
# Source before roslaunching against Gazebo:
#   source env/sim.env.sh && roslaunch ur3_pick_place static/sim.launch
# Read by launch/common/*.launch via $(optenv ...).

export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1

export UR3_USE_SIM=true
export UR3_ROBOT_IP=""            # no physical controller in simulation
export UR3_KINEMATICS_CONFIG=""   # simulated arm needs no per-unit calibration
