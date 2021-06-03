/**
 * @file   GpuSurfStereoDetector.hpp
 * @authors Paul Furgale and Chi Hay Tong
 * @date   May 14, 2010
 * $Rev$
 *
 * @brief  A class that wraps the GPU SURF algorithm
 *         sparse stereo pipeline
 *         The private implementation idiom is used
 *         so that consumers of the class do not need
 *         all of the CUDA headers.
 * 
 */
 
/*
Copyright (c) 2010, Paul Furgale and Chi Hay Tong
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are 
met:

* Redistributions of source code must retain the above copyright notice, 
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
  notice, this list of conditions and the following disclaimer in the 
  documentation and/or other materials provided with the distribution.
* The names of its contributors may not be used to endorse or promote 
  products derived from this software without specific prior written 
  permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef ASRL_GPU_SURF_STEREO_DETECTOR
#define ASRL_GPU_SURF_STEREO_DETECTOR

#include "GpuSurfDetector.hpp"
#include <iostream>

namespace asrl {

  // Forward declaration of the private implementation.
  class GpuSurfStereoDetectorInternal;

  /** 
   * @struct GpuSurfStereoConfiguration
   * @brief A structure representing the GPU surf stereo configuration.
   *
   * A structure representing the GPU surf stereo configuration. Default values
   * are filled in for all elements.
   */
  struct GpuSurfStereoConfiguration : public GpuSurfConfiguration {

    /** 
     * A constructor that fills in all the default values.
     *  
     */
    GpuSurfStereoConfiguration() :
      GpuSurfConfiguration(),
      stereoDisparityMinimum(0.f),
      stereoDisparityMaximum(120.f),
      stereoCorrelationThreshold(0.9f),
      stereoYTolerance(1.f),
      stereoScaleTolerance(0.8f)
    {

    }

    /// The minimum disparity value to consider.
    float stereoDisparityMinimum;
    /// The maximum disparity value to consider.
    float stereoDisparityMaximum;
    /// The maximum SSD score to consider.
    float stereoCorrelationThreshold;
    /// The tolerance on the difference in y coordinates (number of standard deviations the right y coordinate can be from the left y coordinate)
    float stereoYTolerance;
    /// The minimum allowable scale ratio between two matched features.
    float stereoScaleTolerance;
  };

  /** 
   * @class GpuSurfStereoDetector 
   * @brief The exported class representing the GPU surf detector sparse stereo pipeline.
   *
   * This class is the main entry point for high-level use of the gpusurf library. The class has been designed in an
   * RAII style and uses the "Private Implementation" idiom to shield end users from having to find cuda headers
   * TODO: Create an example of how to use this class
   */
  class GpuSurfStereoDetector
  {
  public:

    /** 
     * A constructor that takes a configuration object
     * 
     * @param config The configuration object
     * 
     */
    GPUSURF_API GpuSurfStereoDetector(GpuSurfStereoConfiguration config = GpuSurfStereoConfiguration());

    /** 
     * The destructor
     * 
     */	
    GPUSURF_API virtual ~GpuSurfStereoDetector();

    /** 
     * The first step in the surf pipeline. This builds an integral image from the left image argument
     * Only densly packed images images of type CV_8UC1 are supported.
     * 
     * @param leftImage  The left image used to create the integral image.
     * @param rightImage The right image that will be used for dense stereo.
     */

    GPUSURF_API void setImages(const cv::Mat & leftImage, const cv::Mat & rightImage);

    /** 
     * The second step in the SURF pipeline. This detects keypoints using the fast hessian algorithm
     * 
     */
    GPUSURF_API void detectKeypoints();

    /** 
     * This computes an orientation for all keypoints currently on the GPU
     * 
     */
    GPUSURF_API void findOrientation();

    /** 
     * This computes an orientation for all keypoints currently on the GPU
     * This computation is slightly faster than the one described in the SURF paper
     * 
     */
    GPUSURF_API void findOrientationFast();
  
    /** 
     * This computes a descriptor for each keypoint on the GPU
     * 
     * @param weighted Whether or not to weigh the descriptor components with a Gaussian (slightly better recall performance without)
     */
    GPUSURF_API void computeDescriptors(bool weighted);

    /** 
     * This computes an upright (not rotation invariant) descriptor for each keypoint on the GPU
     * 
     */
    GPUSURF_API void computeUprightDescriptors();

    /** 
     * Matches the keypoints between the left and right images.
     */
    GPUSURF_API void matchKeypoints();

    /**
     * This downloads the keypoints from the GPU and packs them into the vector
     *
     * @param outLeftKeypoints The vector of left keypoints
     * @param outRightKeypoints The vector of right keypoints
     * @param outMatches The vector of left-to-right matches.
     */
    void getKeypoints(std::vector<asrl::Keypoint> & outLeftKeypoints, std::vector<asrl::Keypoint> & outRightKeypoints, std::vector<int> & outMatches);

    /** 
     * This downloads the keypoint descriptors from the GPU and packs them into the vector
     * 
     * @param outKeypoints The vector of keypoint descriptors found. This buffer is N x descriptorSize() where
                           N is the number of keypoints returned by getKeypoints().
     */ 
    GPUSURF_API void getDescriptors(std::vector<float> & outDescriptors);

    /** 
     * The descriptor size
     * 
     * @return The number of elements in each keypoint descriptor
     */ 
    GPUSURF_API int descriptorSize();


    /** 
     * This pushes keypoints on to the GPU.
     * 
     * @param inKeypoints The keypoints to be pushed on to the GPU
     */ 
    GPUSURF_API void setRightKeypoints(std::vector<cv::KeyPoint> const & inKeypoints);

    /** 
     * This pushes keypoints on to the GPU.
     * 
     * @param inKeypoints The keypoints to be pushed on to the GPU
     */ 
    GPUSURF_API void setRightKeypoints(std::vector<asrl::Keypoint> const & inKeypoints);

    /** 
     * This pushes descriptors on to the GPU.
     * 
     * @param inKeypoints The descriptors to be pushed on to the GPU
     */ 
    GPUSURF_API void setRightDescriptors(std::vector<float> const & descriptors);

    /** 
     * This pushes keypoints on to the GPU.
     * 
     * @param inKeypoints The keypoints to be pushed on to the GPU
     */ 
    GPUSURF_API void setLeftKeypoints(std::vector<cv::KeyPoint> const & inKeypoints);

    /** 
     * This pushes keypoints on to the GPU.
     * 
     * @param inKeypoints The keypoints to be pushed on to the GPU
     */ 
    GPUSURF_API void setLeftKeypoints(std::vector<asrl::Keypoint> const & inKeypoints);

    /** 
     * This pushes descriptors on to the GPU.
     * 
     * @param inKeypoints The descriptors to be pushed on to the GPU
     */ 
    GPUSURF_API void setLeftDescriptors(std::vector<float> const & descriptors);


  private:
    /** 
     * @brief The detector is non-copyable. Declare a private copy constructor.
     */
    GpuSurfStereoDetector(const GpuSurfStereoDetector & rhs);
    /** 
     * The detector is non-copyable. Declare a private operator=()
     */
    GpuSurfStereoDetector & operator=(const GpuSurfStereoDetector & rhs);
    /// The private implementation idiom hides the cuda headers from the class consumer.
    GpuSurfStereoDetectorInternal * m_implementation;
  };

} // namespace asrl

inline std::ostream & operator<<(std::ostream & out, asrl::GpuSurfStereoConfiguration const & cfg)
{
  out << "GpuSurfStereoConfiguration:\n";
    out << "threshold: " << cfg.threshold << std::endl;
    out << "nOctaves: " << cfg.nOctaves << std::endl;
    out << "nIntervals: " << cfg.nIntervals << std::endl;
    out << "initialScale: " << cfg.initialScale << std::endl;
    out << "l1: " << cfg.l1 << std::endl;
    out << "l2: " << cfg.l2 << std::endl;
    out << "l3: " << cfg.l3 << std::endl;
    out << "l4: " << cfg.l4 << std::endl;
    out << "edgeScale: " << cfg.edgeScale << std::endl;
    out << "initialStep: " << cfg.initialStep << std::endl;
    out << "targetFeatures: " << cfg.targetFeatures << std::endl;
    out << "detector_threads_x: " << cfg.detector_threads_x << std::endl;
    out << "detector_threads_y: " << cfg.detector_threads_y << std::endl;
    out << "nonmax_threads_x: " << cfg.nonmax_threads_x << std::endl;
    out << "nonmax_threads_y: " << cfg.nonmax_threads_y << std::endl;
    out << "regions_horizontal: " << cfg.regions_horizontal << std::endl;
    out << "regions_vertical: " << cfg.regions_vertical << std::endl;
    out << "regions_target: " << cfg.regions_target << std::endl;

    out << "stereoDisparityMinimum: " << cfg.stereoDisparityMinimum << std::endl;
    out << "stereoDisparityMaximum: " << cfg.stereoDisparityMaximum << std::endl;
    out << "stereoCorrelationThreshold: " << cfg.stereoCorrelationThreshold << std::endl;
    out << "stereoYTolerance: " << cfg.stereoYTolerance << std::endl;
    out << "stereoScaleTolerance: " << cfg.stereoScaleTolerance << std::endl;


    return out;
}


#endif //ASRL_GPU_SURF_DETECTOR
