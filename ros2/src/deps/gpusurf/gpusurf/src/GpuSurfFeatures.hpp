/**
 * @file   GpuSurfFeatures.hpp
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Tue Apr 20 15:41:37 2010
 * 
 * @brief  The memory required for SURF features on the GPU.
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

#ifndef ASRL_GPU_SURF_FEATURES
#define ASRL_GPU_SURF_FEATURES

//#include <builtin_types.h>
#include "CudaSynchronizedMemory.hpp"
#include "gpu_globals.h"


namespace asrl {

  /**
   * @class GpuSurfFeatures
   * @brief Memory required for SURF features on the GPU
   */
  class GpuSurfFeatures
  {
  public:
    
    /** 
     * Constructor This allocates memory for the features found by the asrl::GpuSurfDetector. 
     *
     * As there is no concept of dynamic memory
     * on the GPU, this allocates memory for the maximum number of possible features as
     * defined by ASRL_SURF_MAX_FEATURES and ASRL_SURF_MAX_CANDIDATES
     * 
     */
    GpuSurfFeatures();

    /** 
     * Destructor, cleans up the memory associated with the features.
     * 
     */
    ~GpuSurfFeatures();

    /** 
     * 
     * gets a pointer to the device (GPU) memory for features.
     * 
     * @return a pointer to the start of the device (GPU) feature memory
     */
    Keypoint * deviceFeatures(){return m_features.d_get();}

    /** 
     * 
     * gets the host (CPU) memory for features.
     * 
     * @return a pointer to the start of the host (CPU) feature memory
     */
    Keypoint * hostFeatures(){return m_features.h_get();}

    /** 
     * 
     * gets a pointer to the device (GPU) memory for descriptors
     * 
     * @return a pointer to the start of the device (GPU) memory for descriptors
     */
    float * deviceDescriptors(){return m_descriptors.d_get();}

    /** 
     * 
     * gets a pointer to the host (CPU) memory for descriptors
     * 
     * @return a pointer to the start of the host (CPU) memory for descriptors
     */
    float * hostDescriptors(){ return m_descriptors.h_get();}

    /** 
     * 
     * gets a pointer to the device (GPU) memory for the feature counters
     * 
     * @return a pointer to the device (GPU) memory for the feature counters
     */
    unsigned int * deviceFtCount(){return m_feature_counter.d_get(); }
    
    /** 
     * 
     * gets a pointer to the host (CPU) memory for the feature counters
     * 
     * @return a pointer to the host (GPU) memory for the feature counters
     */
    unsigned int * hostFtCount(){return m_feature_counter.h_get(); }

    /** 
     * Pulls features from the device (GPU) to the host (CPU).
     * 
     * @return a pointer to the start of the host memory for features.
     */
    Keypoint * downloadFeatures();

    /** 
     * Gets the number of features found by the detector. This is the value stored at
     * \code
     * this->featureCounterMem()[0]
     * \endcode 
     *
     * @return The number of features found by the detector.
     */
    unsigned int ftCount();

    /** 
     * Gets the number of features found by the detector before subpixel interpolation. This is the value stored at
     * \code
     * this->featureCounterMem()[1]
     * \endcode 
     * 
     * @return The number of features found by the detector before subpixel interpolation.
     */
    unsigned int rawFeatureCount();

    /** 
     * Clears the feature counts on the host and the device.
     * 
     */
    void clearFeatureCounts();

    /** 
     * 
     * Downloads the descriptors from the device (GPU) to the device (CPU).
     * 
     * @return a pointer to the start of the memory used for the descriptors.
     */
    float * downloadDescriptors();

    /** 
     * Sends a signal to the object that it must download feature counts, features, and descriptors if requested.
     * Calling this doesn't cause any data to be downloaded from the GPU.
     * 
     */
    void setDirty();

    /** 
     * @return The memory object used for features
     */
    CudaSynchronizedMemory<Keypoint> & featuresMem(){return m_features;}

    /** 
     * @return The memory object used for descriptors
     */
    CudaSynchronizedMemory<float> & descriptorsMem(){ return m_descriptors;}
    
    /** 
     * @return The memory object used for feature counters
     */
    CudaSynchronizedMemory<unsigned int> & featureCounterMem(){ return m_feature_counter;}
    /** 
     * @return The memory object used for features before subpixel interpolation
     */
    CudaSynchronizedMemory<int4> & rawFeatureMem(){ return m_rawFeatures; }

    /** 
     * Downloads the keypoints from the GPU and packs them into a vector
     * of asrl keypoints. This function is templated to avoid having C++ headers
     * in this file
     * 
     * @param outKeypoints The vector of keypoints to pack. 
     */
    template<typename ASRL_KEYPOINT_VECTOR>
    void getAsrlKeypoints(ASRL_KEYPOINT_VECTOR & outKeypoints);

    /** 
     * Gets keypoints and packs them into a vector of type T that looks like a 
     * cv::KeyPoint. 
     * The function is templated to avoid having any OpenCV headers in this file.
     * 
     * @param outKeypoints The vector of keypoints to pack.
     */
    template<typename CV_KEYPOINT_VECTOR>
    void getKeypoints(CV_KEYPOINT_VECTOR & outKeypoints);

    /** 
     * Gets the raw (uninterpolated) keypoints from the device.
     * The function is templated to avoid having any OpenCV headers in this file.
     *
     * @param outKeypoints A vector slated to hold the keypoints from the device
     */
    template<typename CV_KEYPOINT_VECTOR>
    void getRawKeypoints(CV_KEYPOINT_VECTOR & outKeypoints);

    /** 
     * Pushes a vector of keypoints to the device
     *
     * The function is templated to avoid having any OpenCV headers in this file.
     * 
     * @param inKeypoints The vector of keypoints to push to the device.
     */
    template<typename CV_KEYPOINT_VECTOR>
    void setKeypoints(CV_KEYPOINT_VECTOR const & inKeypoints);

    /** 
     * Pushes a vector of keypoints to the device
     *
     * The function is templated to avoid having any C++ headers in this file.
     * 
     * @param inKeypoints The vector of keypoints to push to the device.
     */
    template<typename ASRL_KEYPOINT_VECTOR>
    void setAsrlKeypoints(ASRL_KEYPOINT_VECTOR const & inKeypoints);
  private:

    /// Synchronized memory for features 
    CudaSynchronizedMemory<Keypoint> m_features;

    /// Synchronized memory for descriptors
    CudaSynchronizedMemory<float> m_descriptors;

    /// Synchronized memory for the feature counters.
    CudaSynchronizedMemory<unsigned int> m_feature_counter;

    /// Synchronized memory for the raw (uninterpolated) features.
    CudaSynchronizedMemory<int4> m_rawFeatures;

    /// Do the descriptors need to be pulled from the device?
    bool m_descriptorsDirty;

    /// Do the features need to be pulled from the device?
    bool m_featuresDirty;

    /// Do the feature counters need to be pulled from the device?
    bool m_countDirty;
  };




  // This is overly clever. Using a template here avoids including
  // the STL headers.
  template<typename ASRL_KEYPOINT_VECTOR>
  void GpuSurfFeatures::getAsrlKeypoints(ASRL_KEYPOINT_VECTOR & outKeypoints)
  {
    // This has an implicit "Thread synchronize"
    int ftcount = ftCount();

    featuresMem().pullFromDevice();

    outKeypoints.resize(ftcount);
    memcpy(&outKeypoints[0],featuresMem().h_get(),ftcount * sizeof(asrl::Keypoint));

  }

  // This is overly clever. Using a template here avoids including
  // the OpenCV header which CUDA has terrible problems with.
  template<typename CV_KEYPOINT_VECTOR>
  void GpuSurfFeatures::getKeypoints(CV_KEYPOINT_VECTOR & outKeypoints)
  {
    // This has an implicit "Thread synchronize"
    int ftcount = ftCount();

    featuresMem().pullFromDevice();
    //m_features->descriptorsMem().pullFromDevice();

    outKeypoints.resize(ftcount);
    if(ftcount > 0)
      {

	for(int i = 0; i < ftcount; i++)
	  {
	    typename CV_KEYPOINT_VECTOR::value_type & k = outKeypoints[i];
	    Keypoint * ft = hostFeatures() + i;

	    k.pt.x = ft->x;
	    k.pt.y = ft->y;

	    k.size = ft->size;

	    k.octave= ft->octave;
	    k.angle = ft->angle;
	    // This is a hack for development
	    k.class_id = ft->angle_response * 1000000;
	    k.response = ft->response;


	  }
      }

  }

  template<typename CV_KEYPOINT_VECTOR>
  void GpuSurfFeatures::getRawKeypoints(CV_KEYPOINT_VECTOR & outKeypoints)
  {
    // This has an implicit "Thread synchronize"
    m_feature_counter.pullFromDevice();
    m_rawFeatures.pullFromDevice();
    int ftcount = m_feature_counter[1];


    outKeypoints.resize(ftcount);


    for(int i = 0; i < ftcount; i++) {
      //  {hidx_x, hidx_y, threadIdx.z, octave};
      typename CV_KEYPOINT_VECTOR::value_type & k = outKeypoints[i];

      k.pt.x = (float)m_rawFeatures[i].x;
      k.pt.y = (float)m_rawFeatures[i].y;

      k.response = (float)m_rawFeatures[i].z;

      k.octave= m_rawFeatures[i].w;
      k.angle = 0;

    }


  }

  // This is overly clever. Using a template here avoids including
  // the STL headers.
  template<typename ASRL_KEYPOINT_VECTOR>
  void GpuSurfFeatures::setAsrlKeypoints(ASRL_KEYPOINT_VECTOR const & inKeypoints)
  {
    ASRL_ASSERT_LT(inKeypoints.size(), ASRL_SURF_MAX_FEATURES, "The device is unable to hold " << inKeypoints.size() << " features. Max: " << ASRL_SURF_MAX_FEATURES);
    
    m_feature_counter[0] = inKeypoints.size();
    m_feature_counter.pushToDevice();



    memcpy(featuresMem().h_get(),&inKeypoints[0],inKeypoints.size() * sizeof(asrl::Keypoint));

  }

  template<typename CV_KEYPOINT_VECTOR>
  void GpuSurfFeatures::setKeypoints(CV_KEYPOINT_VECTOR const & inKeypoints){

    ASRL_ASSERT_LT(inKeypoints.size(), ASRL_SURF_MAX_FEATURES, "The device is unable to hold " << inKeypoints.size() << " features. Max: " << ASRL_SURF_MAX_FEATURES);
    
    m_feature_counter[0] = inKeypoints.size();
    m_feature_counter.pushToDevice();

    for(unsigned i = 0; i < inKeypoints.size(); i++)
      {
	typename CV_KEYPOINT_VECTOR::value_type const & k = inKeypoints[i];
	Keypoint & ft = m_features[i];
	ft.x = k.pt.x;
	ft.y = k.pt.y;
	ft.size = k.size;
	ft.angle = k.angle;
	ft.response = k.response;
	ft.octave = k.octave;
	ft.sigma_xx = (1 << k.octave)*(1 << k.octave);
	ft.sigma_xy = 0.0;
	ft.sigma_yy = ft.sigma_yy;
      }

    m_features.pushToDevice(inKeypoints.size());
  }
} // namespace asrl

#endif
