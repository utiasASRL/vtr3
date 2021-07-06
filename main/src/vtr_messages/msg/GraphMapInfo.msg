# whether or not the map is set (needed because ros msg has no optional values)
bool set false



## The following entries define the pose of the graph in satellite map
# Root vertex to expand from when applying this transformation
uint64 root_vertex
# The UTM zone used for GPS projections, if applicable
uint32 utm_zone 0
# Transformation from local (VTR) coordinates to map coordinates (T_root_map)
vtr_messages/UtilVectorXd t_map_vtr
# Map scaling, when custom map images are used
float64 scale 1.0

# additional pins 
vtr_messages/GraphPin[] pins

# Path to custom map image tiles, if used
string tile_path
