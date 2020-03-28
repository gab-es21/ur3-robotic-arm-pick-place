#!/usr/bin/env python
# coding: utf8
"""
Approach 2 (dynamic), step 1 - camera-to-robot extrinsic calibration.

The ArUco board (config/dynamic/aruco_markers.yaml, id 582) is held by the gripper
and moved through a handful of known joint configurations. At each stop,
the board pose reported by aruco_ros (in the camera frame) is paired with
the end-effector pose from forward kinematics (in base_link) to solve for
the fixed camera_link -> base_link transform. The result is broadcast as a
static TF and written to config/dynamic/camera_extrinsics.yaml.
"""
import yaml
import rospy
import rospkg
import numpy as np
import tf2_ros
import moveit_commander
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix

MOVE_GROUP = "manipulator"

# Joint configurations that keep the board inside the camera's field of
# view while spreading it out enough for a well-conditioned solve.
CALIBRATION_JOINT_POSES = [
    [0.00, -1.40, 1.20, -1.40, -1.57, 0.00],
    [0.20, -1.30, 1.10, -1.50, -1.57, 0.00],
    [-0.20, -1.30, 1.10, -1.50, -1.57, 0.00],
    [0.00, -1.50, 1.30, -1.30, -1.57, 0.20],
    [0.00, -1.50, 1.30, -1.30, -1.57, -0.20],
]


def pose_to_matrix(pose):
    m = quaternion_matrix([pose.orientation.x, pose.orientation.y,
                            pose.orientation.z, pose.orientation.w])
    m[0, 3] = pose.position.x
    m[1, 3] = pose.position.y
    m[2, 3] = pose.position.z
    return m


class CameraCalibrator(object):
    def __init__(self):
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander(MOVE_GROUP)
        self.marker_pose = None
        rospy.Subscriber("/aruco_single/pose", PoseStamped, self._on_marker)
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def _on_marker(self, msg):
        self.marker_pose = msg

    def _wait_for_marker(self, timeout=5.0):
        self.marker_pose = None
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while self.marker_pose is None and rospy.Time.now() < deadline:
            rate.sleep()
        return self.marker_pose

    def run(self):
        samples_ee = []
        samples_marker = []

        for joints in CALIBRATION_JOINT_POSES:
            self.group.go(joints, wait=True)
            rospy.sleep(0.5)
            marker = self._wait_for_marker()
            if marker is None:
                rospy.logwarn("Marker not visible at this pose, skipping")
                continue
            ee_pose = self.group.get_current_pose().pose
            samples_ee.append(pose_to_matrix(ee_pose))
            samples_marker.append(pose_to_matrix(marker.pose))

        if len(samples_ee) < 3:
            rospy.logerr("Only %d valid samples, need at least 3 - aborting", len(samples_ee))
            return

        translation, quat = self._solve_fixed_transform(samples_ee, samples_marker)
        self._publish_and_save(translation, quat)

    def _solve_fixed_transform(self, ee_poses, marker_poses):
        """Averages the camera->base_link estimate implied by each sample
        pair (board held rigidly at the known end-effector pose)."""
        translations, quats = [], []
        for T_base_ee, T_cam_marker in zip(ee_poses, marker_poses):
            T_cam_base = T_base_ee.dot(np.linalg.inv(T_cam_marker))
            translations.append(T_cam_base[:3, 3])
            quats.append(quaternion_from_matrix(T_cam_base))

        avg_t = np.mean(translations, axis=0)
        avg_q = np.mean(quats, axis=0)
        avg_q /= np.linalg.norm(avg_q)
        return avg_t, avg_q

    def _publish_and_save(self, translation, quat):
        msg = TransformStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.child_frame_id = "camera_link"
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = translation
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = quat
        self.tf_broadcaster.sendTransform(msg)

        out = {
            "parent_frame": "base_link",
            "child_frame": "camera_link",
            "translation": {"x": float(translation[0]), "y": float(translation[1]), "z": float(translation[2])},
            "rotation": {"x": float(quat[0]), "y": float(quat[1]), "z": float(quat[2]), "w": float(quat[3])},
        }
        path = rospkg.RosPack().get_path("ur3_pick_place") + "/config/dynamic/camera_extrinsics.yaml"
        with open(path, "w") as f:
            yaml.safe_dump(out, f)
        rospy.loginfo("Saved camera_link -> base_link to %s", path)


if __name__ == "__main__":
    rospy.init_node("camera_calibration")
    CameraCalibrator().run()
