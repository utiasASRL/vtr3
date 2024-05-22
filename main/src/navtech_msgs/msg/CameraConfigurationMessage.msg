# A ROS message to define camera configuration information

# add a header message to hold message timestamp
std_msgs/Header header

# Image height
uint32 height

# Image width
uint32 width

# Image number of color channels
uint32 channels

# Camera frames per second
uint32 fps