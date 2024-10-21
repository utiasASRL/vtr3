# A ROS message based on an FFT data message from a radar, with corrected types

# add a header message to hold message timestamp
std_msgs/Header header

# angle (float64) represented as a network order (uint8) byte array
float64 angle

# azimuth (uint16) represented as a network order (uint8) byte array
uint16 azimuth

# sweep_counter (uint16) represented as a network order (uint8) byte array
uint16 sweep_counter

# ntp_seconds (uint32) represented as a network order (uint8) byte array
uint32 ntp_seconds

# ntp_split_seconds (uint32) represented as a network order (uint8) byte array
uint32 ntp_split_seconds

# data (uint8) represented as a network order (uint8) byte array
uint8[] data

# data_length (uint16) represented as a network order (uint8) byte array
uint16 data_length
