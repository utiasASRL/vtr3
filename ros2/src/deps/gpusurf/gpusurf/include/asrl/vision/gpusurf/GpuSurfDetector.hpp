/**
 * @file   GpuSurfDetector.hpp
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Fri Feb 19 14:54:04 2010
 * 
 * @brief  A class that wraps the GPU SURF algorithm
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


#ifndef ASRL_GPU_SURF_DETECTOR
#define ASRL_GPU_SURF_DETECTOR

#include <vector>
#include <opencv2/core/core.hpp>
#include "Keypoint.hpp"
#include <opencv2/features2d/features2d.hpp>

#ifdef WIN32
#    ifdef gpusurf_EXPORTS
#        define GPUSURF_API __declspec(dllexport)
#    else
#        define GPUSURF_API __declspec(dllimport)
#    endif
#else
#    define GPUSURF_API
#endif

namespace asrl {

  // Forward declaration of the private implementation.
  class GpuSurfDetectorInternal;

  /** 
   * @struct GpuSurfConfiguration
   * @brief A structure representing the GPU surf configuration.
   *
   * A structure representing the GPU surf configuration. Default values
   * are filled in for all elements.
   */
  struct GpuSurfConfiguration {

    /** 
     * A constructor that fills in all the default values.
     *  
     */
    GpuSurfConfiguration() :
      threshold(0.1f),
      nOctaves(4),
      nIntervals(4),
      initialScale(2.f),
      l1(3.f/1.5f),
      l2(5.f/1.5f),
      l3(3.f/1.5f),
      l4(1.f/1.5f),
      edgeScale(0.81f),
      initialStep(1),
      targetFeatures(1000),
      detector_threads_x(16),
      detector_threads_y(4),
      nonmax_threads_x(16),
      nonmax_threads_y(16),
      regions_horizontal(1),
      regions_vertical(1),
      regions_target(8192)
    {

    }

    /// The interest operator threshold
    float threshold;
    /// The number of octaves to process
    int nOctaves;
    /// The number of intervals in each octave
    int nIntervals;
    /// The scale associated with the first interval of the first octave
    float initialScale;
	
    /// mask parameter l_1
    float l1;
    /// mask parameter l_2 
    float l2;
    /// mask parameter l_3
    float l3;
    /// mask parameter l_4
    float l4;
    /// The amount to scale the edge rejection mask
    float edgeScale;
    /// The initial sampling step in pixels.
    int initialStep;
    /// The target number of features to return, sorted by strength.
    /// If this is zero, an adaptive threshold is not used.
    int targetFeatures;
    /// The number of threads used in the detector kernel (dimension 1)
    int detector_threads_x;
    /// The number of threads used in the detector kernel (dimension 2)
    int detector_threads_y;
    /// The number of threads used in the non-max suppression kernel (dimension 1)
    int nonmax_threads_x;
    /// The number of threads used in the non-max suppression kernel (dimension 2)
    int nonmax_threads_y;

    /// The number of horizontal regions in the image to use for adaptive thresholding
    int regions_horizontal;
    /// The number of horizontal regions in the image to use for adaptive thresholding
    int regions_vertical;
    /// The target number of features in each region of the first octave.
    int regions_target;

    bool upright_flag;
  };

  /** 
   * @class GpuSurfDetector 
   * @brief The exported class representing the GPU surf detector.
   *
   * This class is the main entry point for high-level use of the gpusurf library. The class has been designed in an
   * RAII style and uses the "Private Implementation" idiom to shield end users from having to find cuda headers
   * A complete example of how to use this class is given in the file gpusurf_engine.cpp, but
   * a simple example will be given below:
   * \code
   * 
   * #include <gpusurf/GpuSurfDetector.hpp>
   * 
   * using namespace asrl;
   *
   * // Use the OpenCV library to load an 8-bit, grayscale image.
   * cv::Mat image = cv::imread(imageFilename,CV_LOAD_IMAGE_GRAYSCALE);
   * 
   * // Create the configuration object with all default values
   * GpuSurfConfiguration config;
   *
   * // Create a detector initialized with the configuration
   * GpuSurfDetector detector(config);
   *
   * // Run each step of the SURF algorithm.
   * detector.buildIntegralImage(image);
   * detector.detectKeypoints();
   *
   * // At this point, it is possible to grab the keypoints if descriptors or orientation are not needed.
   *
   * detector.findOrientation();
   * detector.computeDescriptors();
   *
   * // Retrieve the keypoints from the GPU.
   * std::vector<cv::KeyPoint> keypoints;
   * detector.getKeypoints(keypoints)
   *
   * // Retrieve the descriptors from the GPU
   * std::vector<float> descriptors
   * detector.getDescriptors(descriptors);
   * 
   * \endcode
   * 
   * The library can also be used to create descriptors for existing keypoints:
   * \code 
   * #include <gpusurf/GpuSurfDetector.hpp>
   * using namespace asrl;
   *
   * // Create a vector of keypoints
   * std::vector<cv::KeyPoint> keypoints;
   * 
   * // Fill the vector somehow...
   *
   * // Push the keypoints on to the device
   * detector.setKeypoints(keypoints);
   *
   * // Now we can run the description parts of the SURF algorithm
   * detector.findOrientation();
   * detector.computeDescriptors();
   *
   * // Retrieve the keypoints from the GPU.
   * detector.getKeypoints(keypoints)
   *
   * // Retrieve the descriptors from the GPU
   * std::vector<float> descriptors
   * detector.getDescriptors(descriptors);
   * 
   * \endcode
   */
  class GpuSurfDetector
  {
  public:

    /** 
     * A constructor that takes a configuration object
     * 
     * @param config The configuration object
     * 
     */
    GPUSURF_API GpuSurfDetector(GpuSurfConfiguration config = GpuSurfConfiguration());

    /** 
     * The destructor
     * 
     */	
    GPUSURF_API virtual ~GpuSurfDetector();

    /** 
     * The first step in the surf pipeline. This builds an integral image from the image argument
     * Only densly packed images images of type CV_8UC1 are supported.
     * 
     * @param image The image used to create the integral image.
     */

    GPUSURF_API void buildIntegralImage(const cv::Mat & image);

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
     * This downloads the keypoints from the GPU and packs them into the vector
     * 
     * @param outKeypoints The vector of keypoints found.
     */ 
    GPUSURF_API void getKeypoints(std::vector<cv::KeyPoint> & outKeypoints);

    /**
     * This downloads the keypoints from the GPU and packs them into a vector
     *
     * @param outKeypoints The vector of Keypoints found
     */
    GPUSURF_API void getKeypoints(std::vector<asrl::Keypoint> & outKeypoints);

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
     * @return
     */ 
    GPUSURF_API int descriptorSize();


    /** 
     * This pushes keypoints on to the GPU.
     * 
     * @param inKeypoints The keypoints to be pushed on to the GPU
     */ 
    GPUSURF_API void setKeypoints(std::vector<cv::KeyPoint> const & inKeypoints);

    /** 
     * This pushes keypoints on to the GPU.
     * 
     * @param inKeypoints The keypoints to be pushed on to the GPU
     */ 
    GPUSURF_API void setKeypoints(std::vector<asrl::Keypoint> const & inKeypoints);

    /**
     * This saves the hessian buffers to disk
     *
     * @param basename The base filename. The buffers will be saved as basename-octave-N-interval-M.bin
     */
    GPUSURF_API void saveHessianBuffers(std::string const & basename);

    /**
     * This saves the integral image to disk
     *
     * @param basename The base fileame. The integral image will be saved as basename-iimg.bin
     */
    GPUSURF_API void saveIntegralImage(std::string const & basename);
  private:
    /** 
     * @brief The detector is non-copyable. Declare a private copy constructor.
     */
    GpuSurfDetector(const GpuSurfDetector & rhs);
    /** 
     * The detector is non-copyable. Declare a private operator=()
     */
    GpuSurfDetector & operator=(const GpuSurfDetector & rhs);
    /// The private implementation idiom hides the cuda headers from the class consumer.
    GpuSurfDetectorInternal * m_implementation;
  };

} // namespace asrl


#endif // ASRL_GPU_SURF_DETECTOR
