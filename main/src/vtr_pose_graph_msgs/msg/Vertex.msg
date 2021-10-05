# Id of the vertex
uint32 id 0

# Start and end times of the data bubble around this stream
Timestamp keyframe_time

# Start and end times of the data bubble around this stream
TimestampRange time_range

# Transformation matrix that takes points from the reference frame of the world
# to the reference frame of the vertex
VectorXd t_vertex_world
VectorXd t_vertex_world_cov