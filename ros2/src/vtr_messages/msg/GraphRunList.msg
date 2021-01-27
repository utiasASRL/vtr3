# The graph id
# This field allows us to differentiate between different graphs in the same index tree
uint32 graph_id
# List of paths to folders containing runs
string[] run_rpath
# Stores the last run id used, in case runs are missing (might not be set)
int32 last_run -1
# Parameters required for drawing the graph on a map
vtr_messages/GraphMapInfo map