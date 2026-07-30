# Every input fed to a NumericalErrorPredictorNetwork / ImageErrorPredictorNetwork
# forward pass, and the resulting output, for offline visualization/debugging.
# Stamped with the localization chain's leaf stamp (the LiDAR cloud timestamp
# this control cycle is based on) so this can be associated post-hoc with the
# corresponding point cloud / graph vertex.
std_msgs/Header header

# Teach-path sequence id the chain was localized against this cycle.
uint32 sid

# --- Model inputs, exactly as constructed and fed to the model ---

# loc_res: [x_in_path, y_in_path, z_in_path, rx, ry, rz] (6,)
float32[] loc_res

# odom_vel: body-frame velocity twist, -w_p_r_in_r (6,)
float32[] odom_vel

# grav_vec: gravity estimate rotated into the robot body frame (3,)
float32[] grav_vec

# sequence: per-horizon-step reference pose, flattened (T*6,) as
# [tx,ty,tz,phix,phiy,phiz] per step, in row-major step order.
# In path frame
float32[] sequence
uint32 sequence_horizon  # T, i.e. sequence.size() / 6

# --- Model outputs ---

# Raw model output per horizon step, BEFORE invert_correction/smoothing
# post-processing (T,) each.
float32[] raw_dx
float32[] raw_dy
float32[] raw_dyaw

# Final correction actually used (after invert_correction sign flip and
# temporal smoothing, if enabled), before any lateral-only projection.
float32[] final_dx
float32[] final_dy
float32[] final_dyaw
