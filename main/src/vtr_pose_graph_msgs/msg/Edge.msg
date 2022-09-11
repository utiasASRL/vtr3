# Type of the edge: temporal or spatial
EdgeType type

# Mode of the edge: autonomous or manual
EdgeMode mode

# Id of the originating vertex - assumes that the originating run Id is the current run
uint64 from_id

# Id of the terminating vertex
uint64 to_id

# Transformation matrix that takes points from the reference frame of
# originating vertex into the reference frame of the terminating vertex.
vtr_common_msgs/LieGroupTransform t_to_from