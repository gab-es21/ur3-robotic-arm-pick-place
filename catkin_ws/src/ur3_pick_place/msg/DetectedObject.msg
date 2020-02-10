# One clustered, classified object found in the workspace point cloud.

string object_class          # "pen", "cutlery", or "unknown"
geometry_msgs/PoseStamped pose   # centroid, in base_link
geometry_msgs/Vector3 dimensions # axis-aligned bounding box size, meters
float32 confidence            # 0-1, driven by cluster point count
