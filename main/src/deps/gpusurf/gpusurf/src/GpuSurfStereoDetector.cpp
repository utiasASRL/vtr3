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

#include <asrl/vision/gpusurf/GpuSurfStereoDetector.hpp>
#include "GpuSurfStereoDetectorInternal.hpp"
//#include "utils.h"

namespace asrl {

GpuSurfStereoDetector::GpuSurfStereoDetector(GpuSurfStereoConfiguration config)
  : m_implementation(0)
{
  m_implementation = new GpuSurfStereoDetectorInternal(config);
}

  GpuSurfStereoDetector::~GpuSurfStereoDetector()
  {
    if(m_implementation)
      delete m_implementation;
  }

  void GpuSurfStereoDetector::setImages(const cv::Mat & leftImage, const cv::Mat & rightImage)
  {
    m_implementation->setImages(leftImage,rightImage);
  }

  void GpuSurfStereoDetector::detectKeypoints()
  {
    m_implementation->detectKeypoints();
  }

  void GpuSurfStereoDetector::findOrientation()
  {
    m_implementation->findOrientation();
  }

  void GpuSurfStereoDetector::findOrientationFast()
  {
    m_implementation->findOrientationFast();
  }

  void GpuSurfStereoDetector::computeDescriptors(bool weighted)
  {
    m_implementation->computeDescriptors(weighted);
  }

  void GpuSurfStereoDetector::computeUprightDescriptors()
  {
    m_implementation->computeUprightDescriptors();
  }

  void GpuSurfStereoDetector::matchKeypoints()
  {
    m_implementation->matchKeypoints();
  }

    /**
     * This downloads the keypoints from the GPU and packs them into the vector
     *
     * @param outLeftKeypoints The vector of left keypoints
     * @param outRightKeypoints The vector of right keypoints
     * @param outMatches The vector of left-to-right matches.
     */
  void GpuSurfStereoDetector::getKeypoints(std::vector<asrl::Keypoint> & outLeftKeypoints, std::vector<asrl::Keypoint> & outRightKeypoints, std::vector<int> & outMatches)
  {
    m_implementation->getKeypoints(outLeftKeypoints,outRightKeypoints, outMatches);
  }

  void GpuSurfStereoDetector::getDescriptors(std::vector<float> & outDescriptors)
  {
    m_implementation->getDescriptors(outDescriptors);
  }

  int GpuSurfStereoDetector::descriptorSize()
  {
    return m_implementation->descriptorSize();
  }

  void GpuSurfStereoDetector::setRightKeypoints(std::vector<cv::KeyPoint> const & inKeypoints)
  {
    m_implementation->setRightKeypoints(inKeypoints);
  }
  void GpuSurfStereoDetector::setRightKeypoints(std::vector<asrl::Keypoint> const & inKeypoints)
  {
    m_implementation->setRightKeypoints(inKeypoints);
  }
  void GpuSurfStereoDetector::setLeftKeypoints(std::vector<cv::KeyPoint> const & inKeypoints)
  {
    m_implementation->setLeftKeypoints(inKeypoints);
  }
  void GpuSurfStereoDetector::setLeftKeypoints(std::vector<asrl::Keypoint> const & inKeypoints)
  {
    m_implementation->setLeftKeypoints(inKeypoints);
  }

  void GpuSurfStereoDetector::setRightDescriptors(std::vector<float> const & descriptors)
  {
    m_implementation->setRightDescriptors(descriptors);
  }

  void GpuSurfStereoDetector::setLeftDescriptors(std::vector<float> const & descriptors)
  {
    m_implementation->setLeftDescriptors(descriptors);
  }



} // namespace asrl
