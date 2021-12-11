# whether or not the map is set (needed because ros msg has no optional values)
bool set false

## The following entries define the pose of the graph in satellite map
# Root vertex to expand from when applying this transformation
uint64 root_vid 0

# The UTM zone used for GPS projections, if applicable
uint32 utm_zone 0

# Transformation from local (VTR) frame to map frame
VectorXd tf_map_root

# Map scaling, when custom map images are used
float64 scale 1.0

# # additional pins
# Pin[] pins