# map version
uint32 INITIAL = 0
uint32 INTRA_EXP_MERGED = 1
uint32 DYNAMIC_REMOVED = 2
uint32 INTER_EXP_MERGED = 3
uint32 version

# voxel size
float32 dl

#
sensor_msgs/PointCloud2 point_cloud

#
uint64 vertex_id

#
vtr_common_msgs/LieGroupTransform t_vertex_this
