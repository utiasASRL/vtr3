# Observations (keypoints, landmark matches) for specific camera and channel
string name             # the camera name (e.g. left, right)
Keypoint[] keypoints    # the observing keypoint
Match[] landmarks       # matches to landmarks
float32[] precisions      # the precisions of each keypoint observation
float32[] covariances