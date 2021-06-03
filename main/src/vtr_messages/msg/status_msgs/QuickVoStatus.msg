# Vertex ID of the previous keyframe
uint64 keyframe_vid

# Was this frame was a success (matched and optimized)
bool success

# Is this frame going to be a keyframe
bool is_kf

# The number of inlier matches to the keyframe
uint32 inlier_count

# The number of landmarks (triangulated inliers)
uint32 landmark_count

# TIMING
# The timestamp of the live image
uint64 image_stamp

# The delay from image stamp to finishing
float64 latency_ms

# The time spend computing
float64 compute_ms

# TRANSFORMS
# The extrapolated T_frame_keyframe prior
LgTransform t_f_kf_prior

# Final optimized T_frame_keyframe
LgTransform t_f_kf_optimized

CostTerms cost_terms