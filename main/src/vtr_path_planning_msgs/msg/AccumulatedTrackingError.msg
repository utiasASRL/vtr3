# Running total of path-tracking-error correction accumulated across
# repeats via integral-style accumulation:
#   total[N] = total[N-1] + residual[N]

# repeat run id (VertexId major_id) this accumulated total is valid for,
uint32 run_id

# nanoseconds since epoch, when this record was written
int64 timestamp

# accumulated correction, in the teach-path/trunk frame
float64 dx
float64 dy
float64 dyaw
