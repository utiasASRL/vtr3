# The type of descriptor, and its size
string name             # Descriptor type (SURF, ORB etc.)
int32 dims              # Number of descriptor dimensions
int32 bytes_per_desc    # Data type (bytes per single descriptor)
bool upright            # whether the descriptor is upright or rotation invariant (rotated)
