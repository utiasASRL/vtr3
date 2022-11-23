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

/**
 * @file   gpu_area.cpp
 * @author Paul Furgale and Chi Hay Tong
 * @date   Tue Apr 27 16:56:38 2010
 * 
 * @brief  C++ functions wrapping gpu area calls and integral image functionality
 * 
 * 
 */


#include "gpu_area.h"
#include "CudaSynchronizedMemory.hpp"



namespace asrl {


  // unbind a texture from an array
  void fh_untexturizeIntegral()
  {
    cudaError_t err = fh_untexturizeIntegral_c();
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to unbind the integral image texture. " << cudaGetErrorString(err));		
    return;
  }

  // Bind the cudaArray integral image to a texture
  void texturize_integral_image(cudaArray* intImg)
  { 
    cudaError_t err = texturize_integral_image_c(intImg);
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to bind the integral image texture. " << cudaGetErrorString(err));
    return;
  }


  float iiAreaLookupC(cudaArray * image, float cx, float cy, float width, float height)
  {
    texturize_integral_image(image);

    dim3 threads; // 512 threads
   
    threads.x = 1;//FH_X_THREADS;
    threads.y = 1;//FH_Y_THREADS;
    threads.z = 1;
    dim3 grid;
    grid.x = 1;
    grid.y = 1;
    grid.z = 1;
    
    CudaSynchronizedMemory<float> result;
    result.init(1);

    run_iiAreaLookupCDKernel(grid, threads, result.d_get(), cx, cy, width,height);
    ASRL_CHECK_CUDA_ERROR_DBG("iiAreaLookupCDKernel");

    result.pullFromDevice();

    fh_untexturizeIntegral();

    return result[0];
  }  

}
