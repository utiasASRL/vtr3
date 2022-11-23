# Id of the edge
uint32 id 0

# Mode of the edge: AUTONOMOUS or MANUAL
vtr_messages/GraphEdgeMode mode

# Id of the originating vertex
# Assumes that the originating run Id is the current run
uint32 from_id

# Id of the terminating vertex
uint32 to_id

# Id of the terminating run (might not be set)
int32 to_run_id -1

# Index into non-sequential data "streams"
vtr_messages/UtilPoint[] point_idx

# Transformation matrix that takes points from the reference frame of
# originating vertex into the reference frame of the terminating vertex.
vtr_messages/UtilVectorXd t_to_from
vtr_messages/UtilVectorXd t_to_from_cov