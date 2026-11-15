# ORB features extracted from lidar intensity image

# Number of features
uint32 num_features

# Image dimensions
uint32 image_width
uint32 image_height

# Keypoints: flattened [x, y, size, angle, response] per feature (5 * N floats)
float32[] keypoint_data

# Keypoint octaves (N ints)
int32[] keypoint_octaves

# ORB descriptors: flattened (N * 32 bytes)
uint8[] descriptors

# Descriptor width (typically 32 for ORB)
uint32 descriptor_cols

# 3D positions: flattened [x, y, z] per feature (3 * N doubles)
float64[] points_3d

# Per-feature timestamps (nanoseconds, int64, N values)
int64[] timestamps

# Covariance of each 3D position [m^2], same frame as points_3d:
# flattened row-major 3x3 per feature (9 * N doubles, empty = not available)
float64[] covariances