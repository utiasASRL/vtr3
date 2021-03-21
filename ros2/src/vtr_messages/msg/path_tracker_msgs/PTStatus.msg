# @brief Collection of time-based matching metrics between a vertex and other runs
# Transform info
LgTransform t_leaf_trunk     # The pose estimate from VO
uint64 vo_time_stamp                    # The time-stamp from the last VO update
uint64 vo_trunk_id                      # Vertex ID of the trunk (privileged) map vertex for the last VO update
LgTransform t_leaf_trunk_extrapolated     # The pose measurement extrapolated using STEAM
uint64 extrapolated_time_stamp          # The time extrapolated to using STEAM
uint64 trunk_id                         # Vertex ID of the trunk (privileged) map vertex
float64[] velocity                      # Current velocity estimate

# Control inputs
float64 omega_cmd                       # Angular velocity command
float64 v_cmd                           # Angular velocity command
float64 cmd_time_stamp                  # Time the control will be applied

# Prediction uncertainty info
float32 lateral_uncertainty             # Maximum 3-sigma lateral uncertainty over prediction horizon
float32 heading_uncertainty              # Maximum 3-sigma heading uncertainty over prediction horizon