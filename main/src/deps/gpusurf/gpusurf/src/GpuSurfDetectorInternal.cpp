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

#include "GpuSurfDetectorInternal.hpp"
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


namespace asrl {

  GpuSurfDetectorInternal::GpuSurfDetectorInternal(GpuSurfConfiguration config) : 
    m_initialized(false),
    m_config(config)
  {
    GlobalTimer.start("initialization");
    int deviceCount;
    int device;
    cudaError_t err;
    cudaGetDeviceCount(&deviceCount);
    ASRL_ASSERT_GT(deviceCount,0,"There are no CUDA capable devices present");
    
	
    err = cudaGetDevice(&device);
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to get the CUDA device: " << cudaGetErrorString(err));		
    //std::cout << "Found device " << device << std::endl;
    err = cudaGetDeviceProperties(&m_deviceProp,device);
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to get the CUDA device properties: " << cudaGetErrorString(err));		

    // Some more checking...
    // Some more checking...
    ASRL_ASSERT_GE(m_deviceProp.major,1,"Minimum compute capability 1.1 is necessary");
    if(m_deviceProp.major == 1)
      {
	ASRL_ASSERT_GE(m_deviceProp.minor,1,"Minimum compute capability 1.1 is necessary");
      }


    m_maxmin.init(ASRL_SURF_MAX_CANDIDATES,false);
    m_maxmin.memset(0);

    GlobalTimer.stop("initialization");

  }

  GpuSurfDetectorInternal::~GpuSurfDetectorInternal()
  {

  }

  void GpuSurfDetectorInternal::buildIntegralImage(const cv::Mat & image)
  {
    ASRL_ASSERT_GT(image.rows,0,"The image size doesn't make sense");
    ASRL_ASSERT_GT(image.cols,0,"The image size doesn't make sense");
    if(!m_initialized || m_intImg->width() != image.cols || m_intImg->height() != image.rows)
      {
	GlobalTimer.start("initialize detector");
	initDetector(image.cols,image.rows);
	GlobalTimer.stop("initialize detector");
      }
    
    GlobalTimer.start("build integral image");
    m_intProcessor->process(image, *m_intImg);
    GlobalTimer.stop("build integral image");
    texturize_integral_image(m_intImg->d_get());
    init_globals(m_intImg->width(), m_intImg->height(), m_octaves, m_config.nOctaves, m_config.regions_horizontal, m_config.regions_vertical);
  }

  void GpuSurfDetectorInternal::saveHessianBuffers(std::string const & /*basename*/)
  {

//     for(int o = 0; o < m_config.nOctaves; o++)
//       {
// 	m_octaves[o].pullHessianFromDevice();
// 	float * hptr = m_octaves[o].h_hessian();
// 	int intervalSize = m_octaves[o].width() * m_octaves[o].height();
// 	for(int i = 0; i < m_octaves[o].intervals(); i++, hptr += intervalSize)
// 	  {
// 	    // A stringstream used to build the file names.
// 	    std::stringstream sout;
// 	    sout << basename << "-octave-" << o << "-interval-" << i << ".bin";
// 	    std::ofstream fout(sout.str().c_str(),std::ios::binary);
// 	    ASRL_ASSERT(fout.good(),"Unable to open file \"" << sout.str() << "\" for writing");
// 	    int size[2];
// 	    size[0] = m_octaves[o].width();
// 	    size[1] = m_octaves[o].height();
// 	    fout.write(reinterpret_cast<const char *>(&size[0]),2*sizeof(int));
// 	    fout.write(reinterpret_cast<const char *>(hptr),intervalSize*sizeof(float));
// 	  }
//       }
  }

  void GpuSurfDetectorInternal::saveIntegralImage(std::string const & basename)
  {
    float * iimg = m_intImg->h_get();
    std::stringstream sout;
    sout << basename << "-iimg.bin";
    std::ofstream fout(sout.str().c_str(),std::ios::binary);
    ASRL_ASSERT(fout.good(),"Unable to open file \"" << sout.str() << "\" for writing");
    int size[2];
    size[0] = m_intImg->width();
    size[1] = m_intImg->height();
    fout.write(reinterpret_cast<const char *>(&size[0]),2*sizeof(int));
    fout.write(reinterpret_cast<const char *>(iimg),m_intImg->width()*m_intImg->height() * sizeof(float));

  }


  void GpuSurfDetectorInternal::detectKeypoints()
  {

	m_features.featureCounterMem().memset(0);
	

	for(int o = 0; o < m_config.nOctaves; o++)
	  {
	    if(m_octaves[o].valid())
	      {
		run_surf_detector(m_interest.d_get(), m_octaves[o], o, m_features, 
				  m_config.threshold, m_config.detector_threads_x,
				  m_config.detector_threads_y, m_config.nonmax_threads_x,
				  m_config.nonmax_threads_y, m_histograms, m_thresholds,
				  m_config.regions_horizontal, m_config.regions_vertical);
	      }
	  }
  }

  void GpuSurfDetectorInternal::findOrientation()
  {
    int nFeatures = m_features.ftCount();
    GlobalTimer.start("find orientation", nFeatures);
    if(nFeatures > 0)
      {
	find_orientation(m_features.deviceFeatures(), m_features.ftCount());
	ASRL_CHECK_CUDA_ERROR_DBG("Find orientation");
      }
    GlobalTimer.stop("find orientation");
  }

  void GpuSurfDetectorInternal::findOrientationFast()
  {
    int nFeatures = m_features.ftCount();
    GlobalTimer.start("find orientation fast", nFeatures);
    if(nFeatures > 0)
      {
	find_orientation_fast(m_features.deviceFeatures(), m_features.ftCount());
	ASRL_CHECK_CUDA_ERROR_DBG("Find orientation fast");
      }
    GlobalTimer.stop("find orientation fast");
  }

  void GpuSurfDetectorInternal::computeDescriptors(bool weighted)
  {
    int nFeatures = m_features.ftCount();
    GlobalTimer.start("compute descriptors");
    if(nFeatures > 0)
      {
	compute_descriptors(m_features.deviceDescriptors(), m_features.deviceFeatures(), m_features.ftCount(), weighted);
	ASRL_CHECK_CUDA_ERROR_DBG("compute descriptors");
      }
    GlobalTimer.stop("compute descriptors");
  }

  void GpuSurfDetectorInternal::computeUprightDescriptors()
  {
    ASRL_THROW("Not implemented");
  }

  void GpuSurfDetectorInternal::getKeypoints(std::vector<cv::KeyPoint> & outKeypoints)
  {
    int ftcount = m_features.ftCount();
    GlobalTimer.start("download keypoints",ftcount);
    m_features.getKeypoints(outKeypoints);
    GlobalTimer.stop("download keypoints");

  }

  void GpuSurfDetectorInternal::getKeypoints(std::vector<asrl::Keypoint> & outKeypoints)
  {
    m_features.getAsrlKeypoints(outKeypoints);
  }


  void GpuSurfDetectorInternal::setKeypoints(std::vector<cv::KeyPoint> const & inKeypoints)
  {
    m_features.setKeypoints(inKeypoints);
  }

  void GpuSurfDetectorInternal::setKeypoints(std::vector<asrl::Keypoint> const & inKeypoints)
  {
    m_features.setAsrlKeypoints(inKeypoints);
  }

  void GpuSurfDetectorInternal::getDescriptors(std::vector<float> & outDescriptors)
  {
    int ftcount = m_features.ftCount();
    GlobalTimer.start("download descriptors", ftcount);
    m_features.descriptorsMem().pullFromDevice();
    //cudaDeviceSynchronize();
    ASRL_CHECK_CUDA_ERROR("Pull descriptors");
    //print_len(m_features.deviceDescriptors(), ftcount);

    // Resize the destination buffer.
    outDescriptors.resize(descriptorSize() * ftcount);
    // Copy the descriptors into the buffer. AFAIK, all known std::vector implementations use
    // contiguous memory.
    memcpy(&outDescriptors[0],m_features.hostDescriptors(), descriptorSize() * ftcount * sizeof(float));
    GlobalTimer.stop("download descriptors");
  }

  int GpuSurfDetectorInternal::descriptorSize()
  {
    return ASRL_SURF_DESCRIPTOR_DIM;
  }


  void GpuSurfDetectorInternal::initDetector(int width, int height) 
  {  
    m_intProcessor.reset(new GpuIntegralImageProcessor(width, height)); 
    m_intImg.reset(new GpuIntegralImage(width, height));
    
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
    
  }

} // namespace asrl
