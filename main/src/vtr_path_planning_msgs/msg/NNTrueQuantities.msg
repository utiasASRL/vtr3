# The raw, unmodified quantities that a corresponding NNInpute message's
# model inputs were DERIVED from -- lets NN inputs be independently
# recomputed and cross-checked against what was actually fed to the model
# (e.g. catching r_ab_inb()/r_ba_ina() convention mistakes). Stamped
# identically to its corresponding NNInpute message (same chain leaf stamp)
# for post-hoc association.
std_msgs/Header header

builtin_interfaces/Time lidar_stamp

builtin_interfaces/Time extrap_wall_time

uint32 sid

geometry_msgs/Transform t_p_r

geometry_msgs/Transform t_p_r_extp

# T_w_p = T_world_planning, raw.
geometry_msgs/Transform t_w_p

# Raw odometry twist w_p_r_in_r (6,), BEFORE the negation odom_vel applies.
float32[] w_p_r_in_r

# Reference poses for this horizon, BEFORE any error-predictor correction
# was applied -- one full Transform per horizon step.
geometry_msgs/Transform[] reference_poses
