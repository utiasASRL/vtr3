# @brief Collection of time-based matching metrics between a vertex and other runs
# TODO (old) replace fixed64 with PersistentId when the PR comes in
uint64 at_vid                       # The closest vertex id
uint64 to_vid                       # The vertex id we're headed towards
float64 path_curvature               # The curvature of the path (turn radius)
float64[] command                    # The command sent to the vehicle (v,omega)
uint64 transform_time               # The time-stamp from the transform
uint64 store_time                   # The time-stamp for when the message was stored in

