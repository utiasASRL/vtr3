/**
 * @file   gpu_area.h
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Tue Apr 20 20:09:57 2010
 * 
 * @brief  Functions to calculate areas in GPU box filters
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

#ifndef ASRL_GPU_AREA_HPP
#define ASRL_GPU_AREA_HPP

#include <cuda.h>
#include <cuda_runtime_api.h>

namespace asrl {
  


  /** 
   * Store the integral image as a texture on the GPU.
   * 
   * @param intImg The integral image to use as a texture.
   */
  void texturize_integral_image(cudaArray* intImg);

  /** 
   * Store the integral image as a texture on the GPU. This version does not throw an exception on error
   * 
   * @param intImg The integral image to use as a texture.
   */
  cudaError_t texturize_integral_image_c(cudaArray* intImg);


  /** 
   * Remove the integral image texture association.
   * 
   */
  void fh_untexturizeIntegral();

  /** 
   * Remove the integral image texture association. This version throws an exception on error
   * 
   */
  cudaError_t fh_untexturizeIntegral_c();

  /** 
   * A debugging function that calculates an area using box filters on the GPU. This is mostly used for debugging.
   * 
   * @param image The integral image that the region area will be calculated on
   * @param cx    The center of the region in horizontal pixels
   * @param cy    The center of the region in vertical pixels
   * @param width The width of the region.
   * @param height The height of the region.
   * 
   * @return 
   */
  float iiAreaLookupC(cudaArray * image, float cx, float cy, float width, float height);

  /** 
   * A kernel to calculate the area of a region on the GPU. The integral image must have been previously passed
   * to texturize_integral_image()
   * 
   * @param grid    The grid size for the kernel
   * @param threads The thread block size for the kernel
   * @param result  A device pointer where the result will be stored.
   * @param cx      The center of the region in horizontal pixels
   * @param cy      The center of the region in vertical pixels
   * @param width   The width of the region.
   * @param height  The height of the region.
   * 
   * @return 
   */  
  void run_iiAreaLookupCDKernel(dim3 grid, dim3 threads, float * d_result, float cx, float cy, float width, float height);


} // namespace asrl

#endif // ASRL_GPU_AREA_HPP


