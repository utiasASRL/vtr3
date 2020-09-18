# Features (keypoints, descriptors) for a specific camera and channel
string name                     # the camera name (e.g., left, right)
Keypoint[] keypoints            # the detected keypoints
uint8[] descriptors              # sequentially stored feature descriptors
DescriptorType desc_type        # information about the type of descriptor
FeatureInfo[] keypoint_info    # information about the detection of the keypoint
