# A ROS message based on a configuration data message from a radar, with corrected types

# add a header message to hold message timestamp
std_msgs/Header header

# azimuth_samples (uint16)
uint16 azimuth_samples

# encoder_size (uint16)
uint16 encoder_size

# user-provided azimuth offset (uint16)
uint16 azimuth_offset

# bin_size (float64)
float32 bin_size

# range_in_bins (uint16)
uint16 range_in_bins

# expected_rotation_rate (uint16)
uint16 expected_rotation_rate

# range_gain (float32)
float32 range_gain

# range_offset (float32)
float32 range_offset
