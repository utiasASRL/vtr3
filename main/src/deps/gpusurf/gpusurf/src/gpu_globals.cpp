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
 * @file   gpu_globals.cpp
 * @author Paul Furgale and Chi Hay Tong
 * @date   Tue Apr 27 15:48:14 2010
 * 
 * @brief  Functions for dealing with the constant memory used by gpusurf.
 * 
 * 
 */


#include "gpu_globals.h"
#include "GpuSurfOctave.hpp"
#include "assert_macros.hpp"
#include <cuda.h>

namespace asrl {

  void init_globals(int imWidth, int imHeight, GpuSurfOctave * octaves, int nOctaves, int regions_horizontal, int regions_vertical)
  {
    if(imWidth == get_s_initWidth() && imHeight == get_s_initHeight())
      return;
  
  
    float hessian_scale[ASRL_SURF_MAX_OCTAVES*ASRL_SURF_MAX_INTERVALS];
    SurfOctaveParameters params[ASRL_SURF_MAX_OCTAVES];
    for(int r = 0; r < nOctaves; r++)
      {
	params[r] = (SurfOctaveParameters)octaves[r];
	for(int c = 0; c < octaves[r].intervals(); c++)
	  {
	    hessian_scale[r*ASRL_SURF_MAX_INTERVALS + c] = octaves[r].scales()[c];
	  }
      }

    cudaError_t err = cudaMemcpyToSymbol( (const char *)get_d_hessian_scale(), hessian_scale, ASRL_SURF_MAX_OCTAVES*ASRL_SURF_MAX_INTERVALS*sizeof(float));
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to copy cuda hessian scale parameter array: " << cudaGetErrorString(err));  

    err = cudaMemcpyToSymbol((const char *)get_d_octave_params(), params, ASRL_SURF_MAX_OCTAVES*sizeof(SurfOctaveParameters));
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to copy cuda hessian parameter array: " << cudaGetErrorString(err));  

    int stride = octaves[0].stride();
    err = cudaMemcpyToSymbol((const char *)get_d_hessian_stride(), &stride, sizeof(int));
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to copy cuda hessian stride array: " << cudaGetErrorString(err));  
    
    int regions[2];
    regions[0] = regions_horizontal;
    regions[1] = regions_vertical;
    err = cudaMemcpyToSymbol((const char *)get_d_regions(), regions, sizeof(int)*2);
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to copy regions constants array: " << cudaGetErrorString(err));  
    

    get_s_initWidth() = imWidth;
    get_s_initHeight() = imHeight;
    get_s_initialized() = true;


  }

} // namespace asrl
