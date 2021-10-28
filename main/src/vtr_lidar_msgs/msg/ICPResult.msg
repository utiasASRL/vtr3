# nanoseconds since epoch
int64 timestamp

# which version of the map is being localized to
uint32 map_version

# matching indices
uint64[] nn_inds

# matching distance (squared)
float32[] nn_dists

# matching planar distance
float32[] nn_planar_dists