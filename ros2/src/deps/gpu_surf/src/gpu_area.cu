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

#include "gpu_area.h"


/// The global texture pointer.
texture<float, 2, cudaReadModeElementType> d_integralTex;	// global variable

namespace asrl {
  // Bind the cudaArray integral image to a texture
  cudaError_t texturize_integral_image_c(cudaArray* intImg)
  {
    // set texture parameters (reference: sample code - simpleTexture)
    d_integralTex.addressMode[0] = cudaAddressModeClamp;	// clamped to valid region
    d_integralTex.addressMode[1] = cudaAddressModeClamp;	// (for both u and v dimensions)
    d_integralTex.filterMode = cudaFilterModeLinear;//cudaFilterModePoint;//		// linear interpolation
    d_integralTex.normalized = false;						// access with regular texture coordinates

    // reference: pg 46 of CUDA Programming Guide 2.1
    cudaError_t err = cudaBindTextureToArray(d_integralTex, intImg);	

    return err;
  }

  // unbind a texture from an array
  cudaError_t fh_untexturizeIntegral_c()
  {
    cudaError_t err = cudaUnbindTexture(d_integralTex);
    
    return err;
  }


  /** 
   * A CUDA device function for looking up the sum of pixels in an area using
   * an integral image. This is accomplished with 4 lookups. Each pixel in an 
   * integral image contains the sum of all pixels above and to the left of 
   * it. To calculate the sum of pixels within any area, we look up pixels at 
   * the corners:
   *
   * \verbatim
            A *-----------* B
              |           |
              |           |
              |           |
            C *-----------* D
    \endverbatim
   *
   * Area = A - B - C + D
   *
   * Cuda requires that texture variables are global within file scope so this function uses
   * the d_integralTex variable
   * 
   * @param cx         The horizontal pixel coordinates of the center of the area to look up
   * @param cy         The vertical pixel coordinates of the center of the area to look up
   * @param halfWidth  Half of the width of the area to look up
   * @param halfHeight Half of the height of the area to look up
   * 
   * @return The area within the region.
   */
  __device__ float iiAreaLookupCDHalfWH(float cx, float cy, float halfWidth, float halfHeight)
  {
    float result = 0.f;

    // A
    result += tex2D(d_integralTex, cx - halfWidth, cy - halfHeight);
    // B
    result -= tex2D(d_integralTex, cx + halfWidth, cy - halfHeight);
    // C
    result -= tex2D(d_integralTex, cx - halfWidth, cy + halfHeight);
    // D
    result += tex2D(d_integralTex, cx + halfWidth, cy + halfHeight);

    return result;
  }

  /** 
   * A CUDA device function for looking up the sum of pixels in an area using
   * an integral image. This is accomplished with 4 lookups. Each pixel in an 
   * integral image contains the sum of all pixels above and to the left of 
   * it. To calculate the sum of pixels within any area, we look up pixels at 
   * the corners:
   *
   * \verbatim
            A *-----------* B
              |           |
              |           |
              |           |
            C *-----------* D
    \endverbatim
   *
   * Area = A - B - C + D
   *
   * Cuda requires that texture variables are global within file scope so this function uses
   * the d_integralTex variable
   * 
   * @param cx     The horizontal pixel coordinates of the center of the area to look up
   * @param cy     The vertical pixel coordinates of the center of the area to look up
   * @param width  The width of the area to look up
   * @param height The height of the area to look up
   * 
   * @return The area within the region.
   */
  __device__ float iiAreaLookupCD(float cx, float cy, float width, float height)
  {
    return iiAreaLookupCDHalfWH(cx,cy,width*0.5f,height*0.5f);
  }

  /** 
   * A simple kernel that looks up one area on the device
   * 
   * @param d_result The device pointer to a single float used to store the result.
   * @param cx       The horizontal pixel coordinates of the center of the area to look up
   * @param cy       The vertical pixel coordinates of the center of the area to look up
   * @param width    The width of the area to look up
   * @param height   The height of the area to look up
   */
  __global__ void iiAreaLookupCDKernel(float * d_result, float cx, float cy, float width, float height)
  {
    d_result[0] = iiAreaLookupCD(cx,cy,width,height);
  }


  void run_iiAreaLookupCDKernel(dim3 grid, dim3 threads, float * d_result, float cx, float cy, float width, float height)
  {
    iiAreaLookupCDKernel<<< grid, threads >>>(d_result, cx, cy, width, height);
  }
}

