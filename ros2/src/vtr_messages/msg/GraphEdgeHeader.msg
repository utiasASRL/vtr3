# Id of the the run that these edges belong to
# This property allows us to do a backwards check to ensure we are using the
# correct edge/vertex file
uint32 run_id

# Type of edges contained in this file
# This property allows us to change filenames and maintain a mapping between
# files and data types
vtr_messages/GraphEdgeType type

# List of stream names for rosbag data
# Edges translate numerical ids into names using this map.
# Not all edges have the same streams, but they all use this list.
# DO NOT MODIFY this property without modifying the edges
string[] stream_names