# Collection of matching metrics between a vertex and other vertices/runs
uint32 num_features             # Number of features/landmarks in the vertex
VertexMatch[] matches           # List of all possible matches to vertices
VertexMatch[] full_matches      # List of matches to all vertices that observed feature (denser)
RunMatch[] run_matches          # Summary matches to each run; guaranted to not double count