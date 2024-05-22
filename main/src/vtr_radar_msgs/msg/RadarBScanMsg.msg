# A ROS message carrying a B Scan and its associated metadata (e.g. timestamps, encoder IDs)

# B Scan from one rotation of the radar, also holds the time stamp information
sensor_msgs/Image b_scan_img

# The encoder values encompassed by the b scan
uint16[] encoder_values

# The timestamps of each azimuth in the scan
uint64[] timestamps
