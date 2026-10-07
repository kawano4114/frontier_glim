# Completed submap. header is in the configured odom frame and is stamped at
# the submap origin frame. The pose places cloud's local frame in odom.
std_msgs/Header header
int32 id
geometry_msgs/Pose origin_pose

# Points remain in the local submap frame named by cloud.header.frame_id.
sensor_msgs/PointCloud2 cloud

# Optimized LiDAR poses in the same local frame, one per estimation frame.
# The merged cloud does not retain scan-to-point associations for raycasting.
nav_msgs/Path lidar_trajectory
