# The graph id
# This field allows us to differentiate between different graphs in the same index tree
uint32 graph_id
# List of paths to folders containing runs
string[] run_rpath
# Stores the last run id used, in case runs are missing
uint32 last_run