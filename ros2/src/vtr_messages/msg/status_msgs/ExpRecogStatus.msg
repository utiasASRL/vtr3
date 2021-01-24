# Stores result / status information for experience recognition at a vertex

# The timestamp of the keyframe
vtr_messages/TimeStamp keyframe_time

# The vertex id of the live query vertex
uint64 query_id

# Constants for the algorithm that was used
uint8 ALGORITHM_UNKNOWN = 0
uint8 ALGORITHM_BAGOFWORDS = 1
uint8 ALGORITHM_TIME = 2
uint8 ALGORITHM_COLLAB = 3
uint8 ALGORITHM_RANDOM = 4
uint8 ALGORITHM_WORKSMART = 5
uint8 algorithm

# Whether the module was used to filter the experiences for localization
bool in_the_loop

# A map from experience run id to Bag-of-Words cosine distance,
# sorted from lowest to highest by run id
vtr_messages/RunToCosineDistance[] cosine_distances

# Recommended run ids, sorted from lowest to highest (lowest n distance)
uint32[] recommended_ids

# Computation time (main algorithm time)
float64 computation_time_ms

# Load time (time spent loading data from disk)
float64 load_time_ms
