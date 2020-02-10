#!/usr/bin/env python
# coding: utf8
"""Thin wrapper around a GripperCommand action server, shared by both
pick-and-place scripts."""
import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

OPEN_POSITION = 0.0     # meters
CLOSED_POSITION = 0.04  # meters, closes around a thin object (pen/cutlery)


class Gripper(object):
    def __init__(self, action_ns="gripper_controller/gripper_cmd"):
        self._client = actionlib.SimpleActionClient(action_ns, GripperCommandAction)
        self._client.wait_for_server(rospy.Duration(5.0))

    def _send(self, position, max_effort=40.0):
        goal = GripperCommandGoal()
        goal.command.position = position
        goal.command.max_effort = max_effort
        self._client.send_goal(goal)
        self._client.wait_for_result(rospy.Duration(3.0))

    def open(self):
        self._send(OPEN_POSITION)

    def close(self):
        self._send(CLOSED_POSITION)
