# Id of the the run that these edges belong to
# This property allows us to do a backwards check to ensure we are using the
# correct edge/vertex file
uint32 run_id

# List of stream names for rosbag data
# Vertices translate numerical ids into names using this map.
# Not all vertices have the same streams, but they all use this list.
# DO NOT MODIFY this property without modifying the vertices
string[] stream_names