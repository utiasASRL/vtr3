# Combined 3D position and description of landmark.
# This supports the compact representation used for VTR3, where only the first 
# descriptor from a VO track is kept.
string name                   # name of the channel
HVec3[] points                # 3D homogeneous point
float32[] covariance          # 3x3(x #pts) covariance
FeatureInfo[] lm_info         # feature appearance info
uint8[] descriptors           # sequentially stored feat. descriptors
DescriptorType desc_type      # information about the descriptor
Match[] vo_observations       # reverse indices to vo observations
Match[] matches               # indices to matching landmarks in previous experiences.
uint32[] num_vo_observations  # The number of times the landmark was observed in VO.
bool[] valid                  # The validity of the landmark
