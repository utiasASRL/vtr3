#ifndef ASRL_SURF_GPU_STEREOMATCH_HPP
#define ASRL_SURF_GPU_STEREOMATCH_HPP

namespace asrl {

  /** 
   * A function to find stereo matches between a rectified stereo pair of images that have had thier 
   * descriptors correlated.
   * 
   * @param d_keypointsLeft Device pointer to the left keypoints.
   * @param n_keypointsLeft The number of left keypoints.
   * @param d_keypointsRight Device pointer to the right keypoints
   * @param n_keypointsRight The number of right keypoints
   * @param d_descriptorCorrelationMatrix The correlation matrix such that C[i,j] is the correlation value of the ith left descriptor with the ith right descriptor.
   * @param d_leftRightMatches An array to hold the resulting left-right matches.
   * @param y_tolerance The tolerance on the difference in y coordinates (number of standard deviations the right y coordinate can be from the left y coordinate)
   * @param correlation_tolerance The minimum correlation value acceptable for a match.
   * @param min_disparity The minimum allowable disparity.
   * @param max_disparity The maximum allowable disparity.
   * @param scale_tolerance The minimum allowable scale ratio between two matched features.
   */
  void find_stereo_matches(Keypoint * d_keypointsLeft, int n_keypointsLeft, float * d_descriptorLeft,
			   Keypoint * d_keypointsRight, int n_keypointsRight, float * d_descriptorRight,
			   /*float * d_descriptorCorrelationMatrix,*/ int * d_leftRightMatches, 
			   float y_tolerance, float correlation_tolerance, float min_disparity,
			   float max_disparity, float scale_tolerance);
};

#endif // ASRL_SURF_GPU_STEREOMATCH_HPP
