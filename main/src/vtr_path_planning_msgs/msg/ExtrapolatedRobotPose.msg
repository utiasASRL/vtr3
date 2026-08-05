# The MPC's extrapolated robot pose (T_p_r_extp) and the timing it was built_from
std_msgs/Header header

# T_planning_robot, extrapolated forward from the last localization
geometry_msgs/Transform t_p_r_extp

# Timestamp of the LiDAR message T_p_r_extp was extrapolated from
builtin_interfaces/Time lidar_stamp

# Wall-clock time used to compute the extrapolation
builtin_interfaces/Time extrap_wall_time
