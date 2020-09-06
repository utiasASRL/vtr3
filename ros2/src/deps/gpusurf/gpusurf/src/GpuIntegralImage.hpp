/**
 * @file   GpuIntegralImage.hpp
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Thu Feb 18 23:03:30 2010
 *
 * @brief  A simple class that holds an integral image.
 *         The underlying storage is a cudaArray which
 *         may be converted to a texture. The underlying
 *         storage is float.
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


#ifndef ASRL_GPU_INTEGRAL_IMAGE
#define ASRL_GPU_INTEGRAL_IMAGE

#include <memory>
#include "CudaSynchronizedMemory.hpp"
// Forward declarations.
struct cudaArray;

namespace asrl {

  /**
   * @class GpuIntegralImage
   * @brief The integral image on the device
   *
   *         A simple class that holds an integral image.
   *         The underlying storage is a cudaArray which
   *         may be converted to a texture. The underlying
   *         storage is float.
   *
   */
  class GpuIntegralImage {
  public:

    /**
     * A constructor that will initialize the integral image
     *
     * @param width the width of the image (in pixels)
     * @param height the height of the image (in pixels)
     */
    GpuIntegralImage(int width, int height);

    /**
     * The destructor cleans up the device memory.
     *
     */
    ~GpuIntegralImage();

    /**
     *
     * @return the underlying device pointer
     */
    cudaArray * d_get();

    /**
     *
     * @return the underlying host pointer
     */
    float * h_get();

    /**
     *
     * @return the width of the integral image
     */
    int width();

    /**
     *
     * @return the height of the integral image
     */
    int height();
  private:
    /// The width of the integral image
    int m_width;

    /// The height of the integral image
    int m_height;

    /// The underlying device pointer
    std::shared_ptr<cudaArray> m_cudaArray;

    /// The underlying host pointer
    CudaSynchronizedMemory<float> m_buffer;
  };

} // namespace asrl

#endif // ASRL_GPU_INTEGRAL_IMAGE
