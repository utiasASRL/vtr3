# Merged visual landmark map built from lidar intensity-image features

# voxel size
float32 dl

# max ORB Hamming distance to merge as the same landmark
float32 descriptor_merge_dist

# number of landmarks
uint32 num_landmarks

# landmark positions in the map frame: flattened [x, y, z] (3 * N doubles)
float64[] points

# ORB descriptors: flattened (N * descriptor_cols bytes)
uint8[] descriptors

# descriptor width (typically 32 for ORB)
uint32 descriptor_cols

# detector responses (N floats)
float32[] responses

# remaining life times (N floats)
float32[] life_times

# observation counts (N ints)
int32[] num_obs

#
uint64 vertex_id

#
vtr_common_msgs/LieGroupTransform t_vertex_this
