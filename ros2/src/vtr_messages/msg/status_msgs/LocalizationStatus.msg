# timestamp of the keyframe
uint64 keyframe_time

# Vertex ID of the (autonomous) query vertex
uint64 query_id

# Vertex ID of the (privileged) map vertex
uint64 map_id

# Transform that takes points from the map to the query.
LgTransform t_query_map

# Localization success flag
bool success

# Localization success flag
uint32[] inlier_channel_matches

# Localization computation time
float64 localization_computation_time_ms

uint32 window_temporal_depth

uint32 window_num_vertices

CostTerms cost_terms