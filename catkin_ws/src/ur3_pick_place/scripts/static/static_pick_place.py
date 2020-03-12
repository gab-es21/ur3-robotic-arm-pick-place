#!/usr/bin/env python
# coding: utf8
"""
Approach 1 - static pick and place.

Pick and place poses are fixed ahead of time (config/static/poses.yaml);
no perception is involved. Works identically against the Gazebo bringup
or the real driver bringup -- this script only talks to MoveIt and the
gripper action server, both of which present the same interface either way.
"""
import copy
import yaml
import rospy
import rospkg
import moveit_commander
from geometry_msgs.msg import Pose

from ur3_pick_place.gripper_interface import Gripper

MOVE_GROUP = "manipulator"


def load_poses():
    path = rospkg.RosPack().get_path("ur3_pick_place") + "/config/static/poses.yaml"
    with open(path) as f:
        return yaml.safe_load(f)


def to_pose(msg):
    pose = Pose()
    pose.position.x = msg["position"]["x"]
    pose.position.y = msg["position"]["y"]
    pose.position.z = msg["position"]["z"]
    pose.orientation.x = msg["orientation"]["x"]
    pose.orientation.y = msg["orientation"]["y"]
    pose.orientation.z = msg["orientation"]["z"]
    pose.orientation.w = msg["orientation"]["w"]
    return pose


class StaticPickPlace(object):
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander(MOVE_GROUP)
        self.gripper = Gripper()
        self.cfg = load_poses()

    def _go_to_pose(self, pose):
        self.group.set_pose_target(pose)
        ok = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        return ok

    def _lifted(self, pose, dz):
        lifted = copy.deepcopy(pose)
        lifted.position.z += dz
        return lifted

    def run(self):
        approach_dz = self.cfg["approach_offset_z"]
        retreat_dz = self.cfg["retreat_offset_z"]

        self.group.go(self.cfg["home_joint_positions"], wait=True)
        self.gripper.open()

        pick = to_pose(self.cfg["pick_pose"])
        place = to_pose(self.cfg["place_pose"])

        rospy.loginfo("Approaching pick pose")
        self._go_to_pose(self._lifted(pick, approach_dz))
        self._go_to_pose(pick)
        self.gripper.close()

        rospy.loginfo("Lifting and moving to place pose")
        self._go_to_pose(self._lifted(pick, retreat_dz))
        self._go_to_pose(self._lifted(place, approach_dz))
        self._go_to_pose(place)
        self.gripper.open()

        rospy.loginfo("Retreating to home")
        self._go_to_pose(self._lifted(place, retreat_dz))
        self.group.go(self.cfg["home_joint_positions"], wait=True)

        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node("static_pick_place")
    StaticPickPlace().run()
