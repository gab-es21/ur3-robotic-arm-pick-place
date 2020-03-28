#!/usr/bin/env python
# coding: utf8
"""
Approach 2 (dynamic), step 3 - camera-guided pick and place.

Consumes /detected_objects (pointcloud_object_detector.py) and, for each
detected pen or cutlery item, picks it up and drops it into the sorting
box defined in config/dynamic/sorting_box.yaml.
"""
import copy
import Queue
import yaml
import rospy
import rospkg
import moveit_commander
from geometry_msgs.msg import Pose

from ur3_pick_place.gripper_interface import Gripper
from ur3_pick_place.msg import DetectedObject

MOVE_GROUP = "manipulator"
GRASP_APPROACH_Z = 0.10
GRASP_DEPTH_MARGIN = 0.01  # close the gripper slightly above the measured centroid


def load_box_pose():
    path = rospkg.RosPack().get_path("ur3_pick_place") + "/config/dynamic/sorting_box.yaml"
    with open(path) as f:
        cfg = yaml.safe_load(f)

    pose = Pose()
    pose.position.x = cfg["box_pose"]["position"]["x"]
    pose.position.y = cfg["box_pose"]["position"]["y"]
    pose.position.z = cfg["box_pose"]["position"]["z"]
    pose.orientation.x = cfg["box_pose"]["orientation"]["x"]
    pose.orientation.y = cfg["box_pose"]["orientation"]["y"]
    pose.orientation.z = cfg["box_pose"]["orientation"]["z"]
    pose.orientation.w = cfg["box_pose"]["orientation"]["w"]
    return pose, cfg["approach_offset_z"]


class DynamicPickPlace(object):
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander(MOVE_GROUP)
        self.gripper = Gripper()
        self.box_pose, self.box_approach_dz = load_box_pose()
        self.queue = Queue.Queue()
        rospy.Subscriber("/detected_objects", DetectedObject, self.queue.put)

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

    def _pick(self, detection):
        grasp_pose = Pose()
        grasp_pose.position.x = detection.pose.pose.position.x
        grasp_pose.position.y = detection.pose.pose.position.y
        grasp_pose.position.z = detection.pose.pose.position.z + GRASP_DEPTH_MARGIN
        grasp_pose.orientation.x = 1.0  # tool z-axis pointing straight down

        self.gripper.open()
        self._go_to_pose(self._lifted(grasp_pose, GRASP_APPROACH_Z))
        self._go_to_pose(grasp_pose)
        self.gripper.close()
        self._go_to_pose(self._lifted(grasp_pose, GRASP_APPROACH_Z))

    def _drop_in_box(self):
        self._go_to_pose(self._lifted(self.box_pose, self.box_approach_dz))
        self._go_to_pose(self.box_pose)
        self.gripper.open()
        self._go_to_pose(self._lifted(self.box_pose, self.box_approach_dz))

    def run(self):
        rospy.loginfo("Waiting for detections on /detected_objects ...")
        while not rospy.is_shutdown():
            try:
                detection = self.queue.get(timeout=1.0)
            except Queue.Empty:
                continue
            rospy.loginfo("Picking up detected %s (confidence %.2f)",
                           detection.object_class, detection.confidence)
            self._pick(detection)
            self._drop_in_box()
        moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    rospy.init_node("dynamic_pick_place")
    DynamicPickPlace().run()
