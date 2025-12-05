/**
 * @file   GpuSurfStereoDetectorInternal.hpp
 * @authors Paul Furgale and Chi Hay Tong
 * @date   May 14, 2010
 * $Rev$
 *
 * @brief  The private implementation of the GPU SURF detector.
 *
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

#ifndef ASRL_GPU_SURF_DETECTOR_INTERNAL_HPP
#define ASRL_GPU_SURF_DETECTOR_INTERNAL_HPP

#include <memory>
#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/types_c.h>
#elif CV_MAJOR_VERSION >= 3
  #include <opencv2/core/types.hpp>
#endif
#include "CudaSynchronizedMemory.hpp"
#include <vector>
#include "GpuIntegralImageProcessor.hpp"
#include "GpuIntegralImage.hpp"
#include <asrl/vision/gpusurf/GpuSurfStereoDetector.hpp>
#include "GpuSurfFeatures.hpp"
#include "GpuSurfOctave.hpp"

namespace asrl {


  /**
   * @class GpuSurfStereoDetectorInternal
   * @brief The private implementation of the GPU surf detector.
   */

  class GpuSurfStereoDetectorInternal
  {
  public:

    /**
     * Constructs a detector based on configuration parameters
     *
     * @param config The set of configuration parameters.
     */
    GpuSurfStereoDetectorInternal(GpuSurfStereoConfiguration config);

    /**
     * The destructor.
     *
     */
    virtual ~GpuSurfStereoDetectorInternal();

    /**
     * The first step in the surf pipeline. This builds an integral image from the left image argument
     * Only densly packed images images of type CV_8UC1 are supported.
     *
     * @param leftImage  The left image used to create the integral image.
     * @param rightImage The right image that will be used for dense stereo.
     */

    void setImages(const cv::Mat & leftImage, const cv::Mat & rightImage);

    /**
     * The second step in the SURF pipeline. This detects keypoints using the fast hessian algorithm
     *
     */
    void detectKeypoints();

    void detectLeftKeypoints();
    void detectRightKeypoints();
    /**
     * This computes an orientation for all keypoints currently on the GPU
     *
     */
    void findOrientation();

    /**
     * This computes an orientation for all keypoints currently on the GPU
     * This computation is slightly faster than the one described in the SURF paper
     *
     */
    void findOrientationFast();

    /**
     * This computes a descriptor for each keypoint on the GPU
     *
     * @param weighted Whether or not to weigh the descriptor components with a Gaussian (slightly better recall performance without)
     */
    void computeDescriptors(bool weighted);

    /**
     * This computes an upright (not rotation invariant) descriptor for each keypoint on the GPU
     *
     */
    void computeUprightDescriptors();

    /**
     * Matches the keypoints between the left and right images.
     */
    void matchKeypoints();

    /**
     * This downloads the keypoints from the GPU and packs them into the vector
     *
     * @param outLeftKeypoints The vector of left keypoints
     * @param outRightKeypoints The vector of right keypoints
     * @param outMatches The vector of left-to-right matches.
     */
    void getKeypoints(std::vector<asrl::Keypoint> & outLeftKeypoints, std::vector<asrl::Keypoint> & outRightKeypoints, std::vector<int> & outMatches);



    /**
     * This function initializes the detector for an image of a certian hight and width. Internal buffers are allocated
     * and constants are pushed to the GPU.
     *
     * @param width The width of the image to process
     * @param height The height of the image to process
     */
    void initDetector(int width, int height);

    /**
     * This downloads the keypoint descriptors from the GPU and packs them into the vector
     *
     * @param outKeypoints The vector of keypoint descriptors found.
     */
    void getDescriptors(std::vector<float> & outDescriptors);

    /**
     * The descriptor size
     *
     * @return
     */
    int descriptorSize();

    /**
     * This pushes keypoints on to the GPU.
     *
     * @param inKeypoints The keypoints to be pushed on to the GPU
     */
    void setRightKeypoints(std::vector<cv::KeyPoint> const & inKeypoints);

    /**
     * This pushes keypoints on to the GPU.
     *
     * @param inKeypoints The keypoints to be pushed on to the GPU
     */
    void setRightKeypoints(std::vector<asrl::Keypoint> const & inKeypoints);

    /**
     * This pushes descriptors on to the GPU.
     *
     * @param inKeypoints The descriptors to be pushed on to the GPU
     */
    void setRightDescriptors(std::vector<float> const & descriptors);

    /**
     * This pushes keypoints on to the GPU.
     *
     * @param inKeypoints The keypoints to be pushed on to the GPU
     */
    void setLeftKeypoints(std::vector<cv::KeyPoint> const & inKeypoints);

    /**
     * This pushes keypoints on to the GPU.
     *
     * @param inKeypoints The keypoints to be pushed on to the GPU
     */
    void setLeftKeypoints(std::vector<asrl::Keypoint> const & inKeypoints);

    /**
     * This pushes descriptors on to the GPU.
     *
     * @param inKeypoints The descriptors to be pushed on to the GPU
     */
    void setLeftDescriptors(std::vector<float> const & descriptors);


    /// Has the detector been initialized?
    bool m_initialized;

    /// A class that processes integral images.
    std::shared_ptr<GpuIntegralImageProcessor> m_intProcessor;
    /// The integral image corresponding to the left image
    std::shared_ptr<GpuIntegralImage> m_leftIntegralImage;
    /// The integral image corresponding to the right image
    std::shared_ptr<GpuIntegralImage> m_rightIntegralImage;

    /// A list of octaves to be processed
    GpuSurfOctave m_octaves[ ASRL_SURF_MAX_INTERVALS ];
    /// A value derived from the mask parameters used to speed up calculation of the masks.
    float m_dxx_width;
    /// A value derived from the mask parameters used to speed up calculation of the masks.
    float m_dxx_height;

    /// A class to hold the features on the GPU
    GpuSurfFeatures m_leftFeatures;

    /// A class to hold the features on the GPU
    GpuSurfFeatures m_rightFeatures;

    /// The configuration values used to initialize this detector.
    GpuSurfStereoConfiguration m_config;

    /// The properties of the device.
    cudaDeviceProp m_deviceProp;

    /// A temporary buffer for the max/min points
    CudaSynchronizedMemory<int4> m_maxmin;

    /// a buffer to store the results of the interest operator calculation
    CudaSynchronizedMemory<float> m_interest;

    /// Memory for storing histograms of interest operator scores in regions of the image.
    CudaSynchronizedMemory<unsigned int> m_histograms;

    /// Memory for storing adaptive thresholds calculated from the interest operator histograms
    CudaSynchronizedMemory<float> m_thresholds;

    /// The left image of the stereo pair.
    cv::Mat leftImage;
    /// The right image of the stereo pair.
    cv::Mat rightImage;

    /// Memory for storing the stereo left-right matches.
    CudaSynchronizedMemory<int> m_leftRightMatches;

    /// Memory for the descriptor matching
    // CudaSynchronizedMemory<float> m_correlation;
  };

} // namespace asrl



#endif // ASRL_GPU_SURF_DETECTOR_INTERNAL_HPP
