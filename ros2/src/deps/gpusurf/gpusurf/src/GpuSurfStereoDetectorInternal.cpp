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

#include "GpuSurfStereoDetectorInternal.hpp"
#include "gpu_utils.h"
#include "assert_macros.hpp"
#include "fasthessian.h"
#include "gpu_area.h"
#include "keypoint_interpolation.h"
#include "non_max_suppression.h"
#include "orientation.h"
#include "descriptors.h"
#include <fstream>
#include "timing.h"
#include "detector.h"
#include "gpu_stereomatch.h"
#include <cublas.h>
#include <thread>

namespace asrl {

  GpuSurfStereoDetectorInternal::GpuSurfStereoDetectorInternal(GpuSurfStereoConfiguration config) : 
    m_initialized(false),
    m_config(config)
  {
    //GlobalTimer.start("initialization");
    int deviceCount;
    int device;
    cudaError_t err;
    cudaGetDeviceCount(&deviceCount);
    ASRL_ASSERT_GT(deviceCount,0,"There are no CUDA capable devices present");
    
	
    err = cudaGetDevice(&device);
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to get the CUDA device: " << cudaGetErrorString(err));		
    //    std::cout << "Found device " << device << std::endl;
    err = cudaGetDeviceProperties(&m_deviceProp,device);
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to get the CUDA device properties: " << cudaGetErrorString(err));		

    // Some more checking...
    ASRL_ASSERT_GE(m_deviceProp.major,1,"Minimum compute capability 1.1 is necessary");
    if(m_deviceProp.major == 1)
      {
	ASRL_ASSERT_GE(m_deviceProp.minor,1,"Minimum compute capability 1.1 is necessary");
      }


    m_maxmin.init(ASRL_SURF_MAX_CANDIDATES,false);
    m_maxmin.memset(0);


    // cublasStatus cbs = cublasInit();
    // ASRL_ASSERT_EQ(cbs, CUBLAS_STATUS_SUCCESS, "Unable to initialize cudablas");
    //m_correlation.init(ASRL_SURF_MAX_FEATURES * ASRL_SURF_MAX_FEATURES);
    //m_correlation.memset(0);

    

  }

  GpuSurfStereoDetectorInternal::~GpuSurfStereoDetectorInternal()
  {

  }

  void GpuSurfStereoDetectorInternal::setImages(const cv::Mat & leftImage, const cv::Mat & rightImage)
  {
    ASRL_ASSERT_GT(leftImage.rows,0,"The image size doesn't make sense");
    ASRL_ASSERT_GT(leftImage.cols,0,"The image size doesn't make sense");
    ASRL_ASSERT_EQ(rightImage.cols,leftImage.cols,"The image sizes don't make sense");
    ASRL_ASSERT_EQ(rightImage.rows,leftImage.rows,"The image sizes don't make sense");
    if(!m_initialized || m_leftIntegralImage->width() != leftImage.cols || m_leftIntegralImage->height() != leftImage.rows)
      {
	//GlobalTimer.start("initialize detector");
	initDetector(leftImage.cols,leftImage.rows);
	//GlobalTimer.stop("initialize detector");
      }
    
    //GlobalTimer.start("build integral image");
    m_intProcessor->process(leftImage, *m_leftIntegralImage);
    m_intProcessor->process(rightImage, *m_rightIntegralImage);
    //GlobalTimer.stop("build integral image");
    init_globals(m_leftIntegralImage->width(), m_leftIntegralImage->height(), m_octaves, m_config.nOctaves, m_config.regions_horizontal, m_config.regions_vertical);
  }


  void GpuSurfStereoDetectorInternal::detectKeypoints()
  {
    detectLeftKeypoints();
    detectRightKeypoints();
    cudaStreamSynchronize(0);
  }

  void GpuSurfStereoDetectorInternal::detectLeftKeypoints() {
    //////////////////////////////////////////////
    // Process the left image.
    texturize_integral_image(m_leftIntegralImage->d_get());
    m_leftFeatures.featureCounterMem().memset(0);
    for(int o = 0; o < m_config.nOctaves; o++) {
      if(m_octaves[o].valid()) {
        run_surf_detector(m_interest.d_get(), m_octaves[o], o, m_leftFeatures, 
	            m_config.threshold, m_config.detector_threads_x,
	            m_config.detector_threads_y, m_config.nonmax_threads_x,
	            m_config.nonmax_threads_y, m_histograms, m_thresholds,
	            m_config.regions_horizontal, m_config.regions_vertical);
        ASRL_CHECK_CUDA_ERROR_DBG("detector: left")
      }
    }
  }

  void GpuSurfStereoDetectorInternal::detectRightKeypoints() {
    /////////////////////////////////////////////
    // Process the right image.
    texturize_integral_image(m_rightIntegralImage->d_get());
    m_rightFeatures.featureCounterMem().memset(0);
    for(int o = 0; o < m_config.nOctaves; o++) {
      if(m_octaves[o].valid()) {
        run_surf_detector(m_interest.d_get(), m_octaves[o], o, m_rightFeatures, 
                          m_config.threshold, m_config.detector_threads_x,
                          m_config.detector_threads_y, m_config.nonmax_threads_x,
                          m_config.nonmax_threads_y, m_histograms, m_thresholds,
                          m_config.regions_horizontal, m_config.regions_vertical);
        ASRL_CHECK_CUDA_ERROR_DBG("detector: right")
      }
    }
  }

  void GpuSurfStereoDetectorInternal::findOrientation()
  {
    int nFeatures = m_leftFeatures.ftCount();
    //GlobalTimer.start("find orientation", nFeatures);
    if(nFeatures > 0)
      {
	texturize_integral_image(m_leftIntegralImage->d_get());
	find_orientation(m_leftFeatures.deviceFeatures(), nFeatures);
	ASRL_CHECK_CUDA_ERROR_DBG("Find orientation");
      }

    nFeatures = m_rightFeatures.ftCount();
    if(nFeatures > 0)
      {
	texturize_integral_image(m_rightIntegralImage->d_get());
	find_orientation(m_rightFeatures.deviceFeatures(), nFeatures);
	ASRL_CHECK_CUDA_ERROR_DBG("Find orientation");
      }
    //GlobalTimer.stop("find orientation");
  }

  void GpuSurfStereoDetectorInternal::findOrientationFast()
  {

    int nFeatures = m_leftFeatures.ftCount();
    //GlobalTimer.start("find orientation fast", nFeatures);

    if(nFeatures > 0)
      {
	texturize_integral_image(m_leftIntegralImage->d_get());
	find_orientation_fast(m_leftFeatures.deviceFeatures(), m_leftFeatures.ftCount());
	ASRL_CHECK_CUDA_ERROR_DBG("Find orientation fast");
      }

    nFeatures = m_rightFeatures.ftCount();
    if(nFeatures > 0)
      {
	texturize_integral_image(m_rightIntegralImage->d_get());
	find_orientation_fast(m_rightFeatures.deviceFeatures(), m_rightFeatures.ftCount());
	ASRL_CHECK_CUDA_ERROR_DBG("Find orientation fast");
      }
    //GlobalTimer.stop("find orientation fast");
  }

  void GpuSurfStereoDetectorInternal::computeDescriptors(bool weighted)
  {
    //GlobalTimer.start("compute descriptors");
    int nFeatures = m_leftFeatures.ftCount();
    if(nFeatures > 0)
      {
	texturize_integral_image(m_leftIntegralImage->d_get());
	compute_descriptors(m_leftFeatures.deviceDescriptors(), m_leftFeatures.deviceFeatures(), nFeatures, weighted);
	ASRL_CHECK_CUDA_ERROR_DBG("compute descriptors");
      }


    nFeatures = m_rightFeatures.ftCount();
    if(nFeatures > 0)
      {
	texturize_integral_image(m_rightIntegralImage->d_get());
	compute_descriptors(m_rightFeatures.deviceDescriptors(), m_rightFeatures.deviceFeatures(), nFeatures, weighted);
	ASRL_CHECK_CUDA_ERROR_DBG("compute descriptors");
      }

    //GlobalTimer.stop("compute descriptors");
  }

  void GpuSurfStereoDetectorInternal::computeUprightDescriptors()
  {
    ASRL_THROW("Not implemented");
  }

  void GpuSurfStereoDetectorInternal::matchKeypoints()
  {

	int n_lt = m_leftFeatures.ftCount();
	int n_rt = m_rightFeatures.ftCount();
	// cublasSgemm('t','n',n_lt,n_rt,ASRL_SURF_DESCRIPTOR_DIM,1.f,
	// 	    m_leftFeatures.deviceDescriptors(),ASRL_SURF_DESCRIPTOR_DIM,
	// 	    m_rightFeatures.deviceDescriptors(),ASRL_SURF_DESCRIPTOR_DIM,
	// 	    0.f,m_correlation.d_get(),ASRL_SURF_MAX_FEATURES);
	// int err = cublasGetError();
	// ASRL_ASSERT_EQ(err, CUBLAS_STATUS_SUCCESS, "Error performing the feature correlation");
     
	find_stereo_matches(m_leftFeatures.deviceFeatures(), 
			    n_lt, 
			    m_leftFeatures.deviceDescriptors(),
			    m_rightFeatures.deviceFeatures(), 
			    n_rt,
			    m_rightFeatures.deviceDescriptors(),
			    /*m_correlation.d_get(),*/
			    m_leftRightMatches.d_get(), 
			    m_config.stereoYTolerance,   // Y tolerance
			    m_config.stereoCorrelationThreshold,  // Correlation tolerance,
			    m_config.stereoDisparityMinimum,   // minimum disparity
			    m_config.stereoDisparityMaximum, // maximum disparity
			    m_config.stereoScaleTolerance);  // scale tolerance
    cudaDeviceSynchronize();
    ASRL_CHECK_CUDA_ERROR_DBG("stereomatch");
  }


  void GpuSurfStereoDetectorInternal::getKeypoints(std::vector<asrl::Keypoint> & leftKeypoints, std::vector<asrl::Keypoint> & rightKeypoints, std::vector<int> & matches)
  {
    m_leftRightMatches.pullFromDevice();
    m_rightFeatures.getAsrlKeypoints(rightKeypoints);
    m_leftFeatures.getAsrlKeypoints(leftKeypoints);
    matches.resize(m_leftFeatures.ftCount());
    memcpy(&matches[0],m_leftRightMatches.h_get(),sizeof(int)*m_leftFeatures.ftCount());
  }


  void GpuSurfStereoDetectorInternal::getDescriptors(std::vector<float> & outDescriptors)
  {
   

    int ftcount = m_leftFeatures.ftCount();
    m_leftFeatures.descriptorsMem().pullFromDevice();
    cudaDeviceSynchronize();


    // Resize the destination buffer.
    outDescriptors.resize(descriptorSize() * ftcount);
    // Copy the descriptors into the buffer. AFAIK, all known std::vector implementations use
    // contiguous memory.
    memcpy(&outDescriptors[0],m_leftFeatures.hostDescriptors(), descriptorSize() * ftcount * sizeof(float));
    
  }

  int GpuSurfStereoDetectorInternal::descriptorSize()
  {
    return ASRL_SURF_DESCRIPTOR_DIM;
  }


  void GpuSurfStereoDetectorInternal::initDetector(int width, int height) 
  {  
    m_intProcessor.reset(new GpuIntegralImageProcessor(width, height)); 
    m_leftIntegralImage.reset(new GpuIntegralImage(width, height));
    m_rightIntegralImage.reset(new GpuIntegralImage(width, height));
    
    // initialize the fast hessian parameters.
    m_dxx_width = 1 + m_config.l1;
    m_dxx_height = m_config.l2;
   
    // Initialize the octaves
    for(int o = 0; o < m_config.nOctaves; o++)
      {

	int targetFeaturesPerRegion = ASRL_SURF_MAX_CANDIDATES;
	if(m_config.regions_target > 0)
	  targetFeaturesPerRegion = std::max(m_config.regions_target/((1<<(2*o)) * m_config.regions_horizontal * m_config.regions_vertical),1);
	//std::cout << "[" << o << "] target/region:" << targetFeaturesPerRegion << std::endl;
	m_octaves[o].init(width,height,m_config.l1,m_config.l2,m_config.l3, m_config.l4, m_config.edgeScale, m_config.initialScale, o, m_config.initialStep, m_config.nIntervals, targetFeaturesPerRegion);
      }
    
    // Initialize a buffer for the interest operator that is big enough for the largest octave.
    m_interest.init(m_octaves[0].stride() * m_octaves[0].height() * m_octaves[0].intervals());
    m_initialized = true;

    // Initialize variables used for the adaptive threshold
    m_histograms.init(ASRL_SURF_HISTOGRAM_BUCKETS*ASRL_SURF_MAX_REGIONS*ASRL_SURF_MAX_REGIONS);
    m_thresholds.init(ASRL_SURF_MAX_REGIONS*ASRL_SURF_MAX_REGIONS);
    m_thresholds.memset(0);

    // Initialize the feature matches
    m_leftRightMatches.init(ASRL_SURF_MAX_FEATURES);
    
  }

  void GpuSurfStereoDetectorInternal::setRightKeypoints(std::vector<cv::KeyPoint> const & inKeypoints)
  {
    m_rightFeatures.setKeypoints(inKeypoints);
  }
  void GpuSurfStereoDetectorInternal::setRightKeypoints(std::vector<asrl::Keypoint> const & inKeypoints)
  {
    m_rightFeatures.setAsrlKeypoints(inKeypoints);
  }
  void GpuSurfStereoDetectorInternal::setLeftKeypoints(std::vector<cv::KeyPoint> const & inKeypoints)
  {
    m_leftFeatures.setKeypoints(inKeypoints);
  }
  void GpuSurfStereoDetectorInternal::setLeftKeypoints(std::vector<asrl::Keypoint> const & inKeypoints)
  {
    m_leftFeatures.setAsrlKeypoints(inKeypoints);
  }

  void GpuSurfStereoDetectorInternal::setRightDescriptors(std::vector<float> const & descriptors)
  {
    ASRL_ASSERT_EQ(descriptors.size(), m_rightFeatures.ftCount() * 64, "Unexpected descriptor vector size. Expected 64 times the number of keypoints (" << m_rightFeatures.ftCount());
    CudaSynchronizedMemory<float> & dmem = m_rightFeatures.descriptorsMem();

    ASRL_ASSERT_LE(descriptors.size(), dmem.size(), "There are too many descriptor entries for the fixed size buffer");
    
    for(unsigned i = 0; i < descriptors.size(); i++)
      {
	dmem[i] = descriptors[i];
      }

    dmem.pushToDevice();    
  }

  void GpuSurfStereoDetectorInternal::setLeftDescriptors(std::vector<float> const & descriptors)
  {
    ASRL_ASSERT_EQ(descriptors.size(), m_leftFeatures.ftCount() * 64, "Unexpected descriptor vector size. Expected 64 times the number of keypoints (" << m_leftFeatures.ftCount());
    CudaSynchronizedMemory<float> & dmem = m_leftFeatures.descriptorsMem();

    ASRL_ASSERT_LE(descriptors.size(), dmem.size(), "There are too many descriptor entries for the fixed size buffer");
    
    for(unsigned i = 0; i < descriptors.size(); i++)
      {
	dmem[i] = descriptors[i];
      }

    dmem.pushToDevice();

  }



} // namespace asrl
