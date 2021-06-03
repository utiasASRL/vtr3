# @brief Path tracker info used by the safety monitor

string state                            # e.g. "FOLLOWING", "PAUSED", etc.
uint64 current_vertex                   # Vertex ID
uint64 next_vertex                      # Vertex ID
float64 lateral_error
float64 heading_error
float64 predicted_worst_lateral_error
float64 predicted_worst_heading_error
float64 commanded_linear_speed
float64 commanded_angular_speed
float64 localization_computation_time