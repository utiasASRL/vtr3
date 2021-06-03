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

#include <opencv2/features2d/features2d.hpp>
#include <asrl/vision/gpusurf/GpuSurfDetector.hpp>
#include "GpuSurfDetectorInternal.hpp"
//#include "utils.h"

namespace asrl {

GpuSurfDetector::GpuSurfDetector(GpuSurfConfiguration config)
  : m_implementation(0)
{
  m_implementation = new GpuSurfDetectorInternal(config);
}

  GpuSurfDetector::~GpuSurfDetector()
  {
    if(m_implementation)
      delete m_implementation;
  }

  void GpuSurfDetector::buildIntegralImage(const cv::Mat & image)
  {
    m_implementation->buildIntegralImage(image);
  }

  void GpuSurfDetector::detectKeypoints()
  {
    m_implementation->detectKeypoints();
  }

  void GpuSurfDetector::findOrientation()
  {
    m_implementation->findOrientation();
  }

  void GpuSurfDetector::findOrientationFast()
  {
    m_implementation->findOrientationFast();
  }

  void GpuSurfDetector::computeDescriptors(bool weighted)
  {
    m_implementation->computeDescriptors(weighted);
  }

  void GpuSurfDetector::computeUprightDescriptors()
  {
    m_implementation->computeUprightDescriptors();
  }

  void GpuSurfDetector::getKeypoints(std::vector<cv::KeyPoint> & outKeypoints)
  {
    m_implementation->getKeypoints(outKeypoints);
  }

  void GpuSurfDetector::getKeypoints(std::vector<asrl::Keypoint> & outKeypoints)
  {
    m_implementation->getKeypoints(outKeypoints);
  }

  void GpuSurfDetector::setKeypoints(std::vector<cv::KeyPoint> const & inKeypoints)
  {
    m_implementation->setKeypoints(inKeypoints);
  }

  void GpuSurfDetector::setKeypoints(std::vector<asrl::Keypoint> const & inKeypoints)
  {
    m_implementation->setKeypoints(inKeypoints);
  }

  void GpuSurfDetector::saveHessianBuffers(std::string const & basename)
  {
    m_implementation->saveHessianBuffers(basename);
  }

  void GpuSurfDetector::saveIntegralImage(std::string const & basename)
  {
    m_implementation->saveIntegralImage(basename);
  }

  void GpuSurfDetector::getDescriptors(std::vector<float> & outDescriptors)
  {
    m_implementation->getDescriptors(outDescriptors);
  }

  int GpuSurfDetector::descriptorSize()
  {
    return m_implementation->descriptorSize();
  }


} // namespace asrl
