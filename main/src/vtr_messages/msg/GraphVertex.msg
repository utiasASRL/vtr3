# Id of the vertex
uint32 id 0

# Start and end times of the data bubble around this stream
vtr_messages/UtilInterval stream_time

# Index ranges into the sequential stream data
vtr_messages/UtilIntervalNamed[] stream_idx

# Index into run-level edge data stream name array
vtr_messages/UtilPoint[] point_idx

# Transformation matrix that takes points from the reference frame of the world
# to the reference frame of the vertex
vtr_messages/UtilVectorXd t_vertex_world
vtr_messages/UtilVectorXd t_vertex_world_cov