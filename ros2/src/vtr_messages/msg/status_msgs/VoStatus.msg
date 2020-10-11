# timestamp of the keyframe
uint64 leaf_image_stamp

# timestamp of the keyframe
uint64 leaf_processed_stamp

# frame-to-keyframe VO computation time
float64 computation_time_ms

# Vertex ID of the twig vertex (most recent in live run)
int64 twig_id

# Vertex ID of the branch (privileged) map vertex
int64 branch_id

# Vertex ID of the trunk (privileged) map vertex
int64 trunk_id

# Points from the leaf to the trunk.
LgTransform t_leaf_trunk

# Points from the leaf to the trunk. extrapolated to leaf_processed_stamp
LgTransform t_leaf_trunk_extrapolated

# Points from the leaf to the twig.
LgTransform t_leaf_twig

# Points from the twig to the branch.
LgTransform t_twig_branch

# Points from the leaf to the trunk.
LgTransform t_branch_trunk

# VO success flag
bool success

# Is a keyframe being created from this leaf?
bool keyframe_flag