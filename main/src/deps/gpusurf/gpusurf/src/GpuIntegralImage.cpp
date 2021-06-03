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

#include "GpuIntegralImage.hpp"
#include <cuda.h>
#include <builtin_types.h>
#include <channel_descriptor.h>
#include "assert_macros.hpp"

namespace asrl {

  GpuIntegralImage::GpuIntegralImage(int width, int height) : m_width(width), m_height(height)
  {
    // allocate gpu cudaArray
    cudaChannelFormatDesc desc = cudaCreateChannelDesc<float>();
    cudaArray * carr;
    cudaError_t err = cudaMallocArray(&carr, &desc, width, height);
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to allocate CUDA array integral image. output: " << cudaGetErrorString(err));
    m_cudaArray.reset(carr,&cudaFreeArray);
  }

  GpuIntegralImage::~GpuIntegralImage()
  {
    // The smart pointer should clean up after itself.
  }

  cudaArray * GpuIntegralImage::d_get()
  {
    return m_cudaArray.get();
  }

  int GpuIntegralImage::width()
  {
    return m_width;
  }

  int GpuIntegralImage::height()
  {
    return m_height;
  }

  float * GpuIntegralImage::h_get()
  {
    // Allocate the buffer if necessary.
    if(m_buffer.size() == 0)
      {
	m_buffer.init(width() * height());
      }
      
      // Download the integral image.
      cudaError_t err = cudaMemcpyFromArray (m_buffer.d_get(), m_cudaArray.get(), 0, 0, width()*height()*sizeof(float),  cudaMemcpyDeviceToDevice);
      ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to retrieve the integral image from the device. output: " << cudaGetErrorString(err));      
      m_buffer.pullFromDevice();
      return m_buffer.h_get();
      
  }

} // namespace asrl
