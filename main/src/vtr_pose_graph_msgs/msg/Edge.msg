# Type of the edge: temporal or spatial
EdgeType type

# Mode of the edge: AUTONOMOUS or MANUAL
EdgeMode mode

# Id of the originating vertex - assumes that the originating run Id is the current run
uint32 from_id

# Id of the terminating vertex
uint32 to_id

# id of the terminating run - this is for spatial edge only, temporal edge is always -1
int32 to_run_id -1

# Transformation matrix that takes points from the reference frame of
# originating vertex into the reference frame of the terminating vertex.
VectorXd t_to_from
VectorXd t_to_from_cov