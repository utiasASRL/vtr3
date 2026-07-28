std_msgs/Header header

# Per-horizon-step predicted tracking error correction, in the teach-path/
# trunk frame -- one entry per MPC horizon step, same (dx, dy, dyaw)
# convention used throughout vtr_path_planning/error_predictors
# (see applyPoseCorrection). Published regardless of whether the correction
# was actually applied to the reference trajectory this cycle.
float32[] dx
float32[] dy
float32[] dyaw
