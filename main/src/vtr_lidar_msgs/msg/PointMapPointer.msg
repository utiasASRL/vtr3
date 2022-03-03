# vertex id associated with this message
uint64 this_vid

# vertex id this message points to
uint64 map_vid

# transformation from map_vid to this_vid
vtr_common_msgs/LieGroupTransform t_v_this_map
