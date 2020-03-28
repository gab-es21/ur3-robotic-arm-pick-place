#!/usr/bin/env python
# coding: utf8
"""
Approach 2 (dynamic), step 2 - point cloud object detection.

Crops the raw depth camera cloud to the workspace above the table, removes
the table plane with a RANSAC fit, clusters what's left into individual
objects, and classifies each cluster against config/dynamic/object_classes.yaml
by its bounding-box dimensions (pen vs. cutlery). Publishes one DetectedObject
per cluster on /detected_objects, transformed into base_link using the
camera_link -> base_link transform solved by camera_calibration.py.
"""
import yaml
import rospy
import rospkg
import numpy as np
import tf2_ros
import tf2_sensor_msgs
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import PoseStamped, Vector3

from ur3_pick_place.msg import DetectedObject

WORKSPACE_BOUNDS = {  # base_link frame, meters
    "x": (0.15, 0.55),
    "y": (-0.40, 0.40),
    "z": (0.00, 0.30),
}
PLANE_DISTANCE_THRESHOLD = 0.008   # m, RANSAC inlier threshold for the table
CLUSTER_RADIUS = 0.02              # m, neighbour radius for clustering
MIN_CLUSTER_POINTS = 30


def load_classes():
    path = rospkg.RosPack().get_path("ur3_pick_place") + "/config/dynamic/object_classes.yaml"
    with open(path) as f:
        return yaml.safe_load(f)


def fit_plane_ransac(points, threshold, iterations=200):
    """Minimal RANSAC plane fit; returns the inlier mask for the largest
    plane found (the table)."""
    best_inliers = None
    n = points.shape[0]
    for _ in range(iterations):
        sample = points[np.random.choice(n, 3, replace=False)]
        v1, v2 = sample[1] - sample[0], sample[2] - sample[0]
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal /= norm
        d = -normal.dot(sample[0])
        dist = np.abs(points.dot(normal) + d)
        inliers = dist < threshold
        if best_inliers is None or inliers.sum() > best_inliers.sum():
            best_inliers = inliers
    return best_inliers


def euclidean_clusters(points, radius, min_points):
    """Radius-based clustering: flood-fill neighbours over a KD-tree."""
    tree = cKDTree(points)
    visited = np.zeros(len(points), dtype=bool)
    clusters = []
    for i in range(len(points)):
        if visited[i]:
            continue
        stack, members = [i], []
        visited[i] = True
        while stack:
            idx = stack.pop()
            members.append(idx)
            for j in tree.query_ball_point(points[idx], radius):
                if not visited[j]:
                    visited[j] = True
                    stack.append(j)
        if len(members) >= min_points:
            clusters.append(points[members])
    return clusters


def classify(dimensions, classes_cfg):
    sorted_dims = sorted(dimensions)
    length = sorted_dims[-1]
    width = sorted_dims[0] if sorted_dims[0] > 1e-4 else 1e-4
    elongation = length / width
    for name, rule in classes_cfg["classes"].items():
        lo, hi = rule["length_range_m"]
        if lo <= length <= hi and width <= rule["max_width_m"] and elongation >= rule["elongation_min"]:
            return name
    return classes_cfg["default_class"]


class PointCloudObjectDetector(object):
    def __init__(self):
        self.classes_cfg = load_classes()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher("/detected_objects", DetectedObject, queue_size=10)
        rospy.Subscriber("/camera/depth/points", PointCloud2, self._on_cloud, queue_size=1)

    def _on_cloud(self, msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                "base_link", msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(5, "camera_link -> base_link not available yet "
                                       "(run calibrate_camera.launch)")
            return

        cloud_base = tf2_sensor_msgs.do_transform_cloud(msg, transform)
        points = np.array(list(point_cloud2.read_points(
            cloud_base, field_names=("x", "y", "z"), skip_nans=True)))
        if points.shape[0] < MIN_CLUSTER_POINTS:
            return

        mask = (
            (points[:, 0] > WORKSPACE_BOUNDS["x"][0]) & (points[:, 0] < WORKSPACE_BOUNDS["x"][1]) &
            (points[:, 1] > WORKSPACE_BOUNDS["y"][0]) & (points[:, 1] < WORKSPACE_BOUNDS["y"][1]) &
            (points[:, 2] > WORKSPACE_BOUNDS["z"][0]) & (points[:, 2] < WORKSPACE_BOUNDS["z"][1])
        )
        workspace_points = points[mask]
        if workspace_points.shape[0] < MIN_CLUSTER_POINTS:
            return

        plane_inliers = fit_plane_ransac(workspace_points, PLANE_DISTANCE_THRESHOLD)
        above_table = workspace_points[~plane_inliers] if plane_inliers is not None else workspace_points

        for cluster in euclidean_clusters(above_table, CLUSTER_RADIUS, MIN_CLUSTER_POINTS):
            self._publish_cluster(cluster, msg.header.stamp)

    def _publish_cluster(self, cluster, stamp):
        centroid = cluster.mean(axis=0)
        dims = cluster.max(axis=0) - cluster.min(axis=0)
        object_class = classify(dims, self.classes_cfg)
        if object_class == self.classes_cfg["default_class"]:
            return

        detection = DetectedObject()
        detection.object_class = object_class
        detection.pose = PoseStamped()
        detection.pose.header.frame_id = "base_link"
        detection.pose.header.stamp = stamp
        detection.pose.pose.position.x = centroid[0]
        detection.pose.pose.position.y = centroid[1]
        detection.pose.pose.position.z = centroid[2]
        detection.pose.pose.orientation.w = 1.0
        detection.dimensions = Vector3(*dims)
        detection.confidence = min(1.0, len(cluster) / 200.0)

        self.pub.publish(detection)


if __name__ == "__main__":
    rospy.init_node("pointcloud_object_detector")
    PointCloudObjectDetector()
    rospy.spin()
