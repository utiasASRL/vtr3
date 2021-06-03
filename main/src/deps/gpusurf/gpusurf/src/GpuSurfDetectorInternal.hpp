/**
 * @file   GpuSurfDetectorInternal.hpp
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Mon Feb 22 12:15:49 2010
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
#include <asrl/vision/gpusurf/GpuSurfDetector.hpp>
#include "GpuSurfFeatures.hpp"
#include "GpuSurfOctave.hpp"

namespace asrl {


  /**
   * @class GpuSurfDetectorInternal
   * @brief The private implementation of the GPU surf detector.
   */

  class GpuSurfDetectorInternal
  {
  public:

    /**
     * Constructs a detector based on configuration parameters
     *
     * @param config The set of configuration parameters.
     */
    GpuSurfDetectorInternal(GpuSurfConfiguration config);

    /**
     * The destructor.
     *
     */
    virtual ~GpuSurfDetectorInternal();

    /**
     * The first step in the surf pipeline. This builds an integral image from the image argument
     * Only densly packed images images of type CV_8UC1 are supported.
     *
     * @param image The image used to create the integral image.
     */
    void buildIntegralImage(const cv::Mat & image);

    /**
     * The second step in the SURF pipeline. This detects keypoints using the fast hessian algorithm
     *
     */
    void detectKeypoints();

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
     * This downloads the keypoints from the GPU and packs them into the vector
     *
     * @param outKeypoints The vector of keypoints found.
     */
    void getKeypoints(std::vector<cv::KeyPoint> & outKeypoints);

    /**
     * This downloads the keypoints from the GPU and packs them into a vector
     *
     * @param outKeypoints The vector of Keypoints found
     */
    void getKeypoints(std::vector<asrl::Keypoint> & outKeypoints);

    /**
     * This pushes keypoints on to the GPU.
     *
     * @param inKeypoints The keypoints to be pushed on to the GPU
     */
    void setKeypoints(std::vector<cv::KeyPoint> const & inKeypoints);

    /**
     * This pushes keypoints on to the GPU.
     *
     * @param inKeypoints The keypoints to be pushed on to the GPU
     */
    void setKeypoints(std::vector<asrl::Keypoint> const & inKeypoints);

    /**
     * This function initializes the detector for an image of a certian hight and width. Internal buffers are allocated
     * and constants are pushed to the GPU.
     *
     * @param width The width of the image to process
     * @param height The height of the image to process
     */
    void initDetector(int width, int height);

   /**
     * This saves the hessian buffers to disk
     *
     * @param basename The base fileame. The buffers will be saved as basename-octave-N-interval-M.bin
     */
    void saveHessianBuffers(std::string const & /*basename*/);

    /**
     * This saves the integral image to disk
     *
     * @param basename The base fileame. The integral image will be saved as basename-iimg.bin
     */
    void saveIntegralImage(std::string const & basename);

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


    /// Has the detector been initialized?
    bool m_initialized;

    /// A class that processes integral images.
    std::shared_ptr<GpuIntegralImageProcessor> m_intProcessor;
    /// An integral image
    std::shared_ptr<GpuIntegralImage> m_intImg;

    /// A list of octaves to be processed
    GpuSurfOctave m_octaves[ ASRL_SURF_MAX_INTERVALS ];
    /// A value derived from the mask parameters used to speed up calculation of the masks.
    float m_dxx_width;
    /// A value derived from the mask parameters used to speed up calculation of the masks.
    float m_dxx_height;

    /// A class to hold the features on the GPU
    GpuSurfFeatures m_features;

    /// The configuration values used to initialize this detector.
    GpuSurfConfiguration m_config;

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

  };

} // namespace asrl



#endif // ASRL_GPU_SURF_DETECTOR_INTERNAL_HPP
