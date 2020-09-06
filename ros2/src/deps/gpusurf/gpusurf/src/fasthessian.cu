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

#include "fasthessian.h"
#include "gpu_utils.h"
#include "gpu_globals.h"

namespace asrl{


  __device__ float evalDyyByArea(float x, float y, float t, float mask_width, float mask_height, float fscale)
  {
	float Dyy = 0.f;
	
	Dyy +=     iiAreaLookupCDHalfWH( x , y, mask_width, mask_height);
	Dyy -= t * iiAreaLookupCDHalfWH( x , y, mask_width, fscale);

	Dyy *=  1/(fscale*fscale);
	return Dyy;
  }

  __device__ float evalDxxByArea(float x, float y, float t, float mask_width, float mask_height, float fscale)
  {
	float Dxx = 0.f;
	
	Dxx +=     iiAreaLookupCDHalfWH( x , y, mask_height, mask_width);
	Dxx -= t * iiAreaLookupCDHalfWH( x , y, fscale     , mask_width);

	Dxx *=  1/(fscale*fscale);
	return Dxx;
  }


  __device__ float evalDxy(float x, float y, float fscale, int octave)
  {
	float center_offset =  d_octave_params[octave].dxy_center_offset  * fscale;
	float half_width    =  d_octave_params[octave].dxy_half_width  * fscale;
	float Dxy = 0.f;
	Dxy += iiAreaLookupCDHalfWH(x - center_offset, y - center_offset, half_width, half_width);
	Dxy -= iiAreaLookupCDHalfWH(x - center_offset, y + center_offset, half_width, half_width);
	Dxy += iiAreaLookupCDHalfWH(x + center_offset, y + center_offset, half_width, half_width);
	Dxy -= iiAreaLookupCDHalfWH(x + center_offset, y - center_offset, half_width, half_width);
	
	Dxy *= 1/(fscale*fscale);
	return Dxy;
  }

  __global__ void fasthessian_kernel(float * d_hessian, int octave)
  {
    // Determine the indices in the Hessian buffer
    int hidx_x = threadIdx.x + __mul24(blockIdx.x,blockDim.x);
    int hidx_y = threadIdx.y + __mul24(blockIdx.y,blockDim.y);
    int hidx_z = threadIdx.z;
    
    int hidx_lin = hidx_x + d_hessian_stride[0] * hidx_y + d_hessian_stride[0] * d_octave_params[octave].y_size * hidx_z;

    // Compute the scale
    float fscale = d_hessian_scale[ASRL_SURF_MAX_INTERVALS*octave + hidx_z];

    // Compute the lookup location of the mask center
    float xpos = (hidx_x * d_octave_params[octave].step + d_octave_params[octave].border);
    float ypos = (hidx_y * d_octave_params[octave].step + d_octave_params[octave].border);

    // Scale the mask dimensions according to the scale
    if(hidx_x < d_octave_params[octave].x_size && hidx_y < d_octave_params[octave].y_size && hidx_z < d_octave_params[octave].nIntervals)
      {
	float local_mask_width =  d_octave_params[octave].mask_width  * fscale;
	float local_mask_height = d_octave_params[octave].mask_height * fscale;

	// Compute the filter responses
	float Dyy = evalDyyByArea(xpos, ypos, d_octave_params[octave].mask_height, local_mask_width, local_mask_height, fscale);
	float Dxx = evalDxxByArea(xpos, ypos, d_octave_params[octave].mask_height, local_mask_width, local_mask_height, fscale);
	float Dxy = evalDxy(xpos, ypos, fscale, octave);
	
	// Combine the responses and store the Laplacian sign
	float result = ( Dxx * Dyy ) - d_octave_params[octave].dxy_scale*(Dxy*Dxy);
	if(Dxx+Dyy > 0.f)
	  setLastBit(result);
	else
	  clearLastBit(result);

	d_hessian[hidx_lin] = result;
      }
  }


  __global__ void eval_component_kernel(float * d_hessian, int octave, int component)
  {
    int hidx_x = threadIdx.x + blockIdx.x * blockDim.x;
    int hidx_y = threadIdx.y + blockIdx.y * blockDim.y;
    int hidx_z = threadIdx.z;
    int hidx_lin = hidx_x + d_hessian_stride[0] * hidx_y + d_hessian_stride[0] * d_octave_params[octave].y_size * hidx_z;

    __shared__ float t;
    if(threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0)
      {
	t = d_octave_params[octave].mask_height;
      }

    __syncthreads();

    float fscale = d_hessian_scale[ASRL_SURF_MAX_INTERVALS*octave + hidx_z];

    float x = (hidx_x * d_octave_params[octave].step + d_octave_params[octave].border);
    float y = (hidx_y * d_octave_params[octave].step + d_octave_params[octave].border);

    if(hidx_x < d_octave_params[octave].x_size && hidx_y < d_octave_params[octave].y_size && hidx_z < d_octave_params[octave].nIntervals)
      {
	float mask_width =  d_octave_params[octave].mask_width  * fscale;
	float mask_height = d_octave_params[octave].mask_height * fscale;

	float Dyy = evalDyyByArea(x, y, t, mask_width, mask_height, fscale);
	float Dxx = evalDxxByArea(x, y, t, mask_width, mask_height, fscale);
	float Dxy = evalDxy(x, y, fscale, octave);
	float censure = 0.f;

	float result = ( Dxx * Dyy ) - d_octave_params[octave].dxy_scale*(Dxy*Dxy);

	switch(component)
	  {
	  case FH_DXX:
	    d_hessian[hidx_lin] = Dxx;
	    break;
	  case FH_DYY:
	    d_hessian[hidx_lin] = Dyy;
	    break;
	  case FH_DXX_DYY:
	    d_hessian[hidx_lin] = Dxx * Dyy;
	    break;
	  case FH_DXY:
	    d_hessian[hidx_lin] = Dxy;
	    break;
	  case FH_LINEAR_IDX:
	    d_hessian[hidx_lin] = hidx_lin;
	    break;
	  case FH_HIDX_X_IDX:
	    d_hessian[hidx_lin] = hidx_x;
	    break;
	  case FH_HIDX_Y_IDX:
	    d_hessian[hidx_lin] = hidx_y;
	    break;
	  case FH_FSCALE:
	    d_hessian[hidx_lin] = fscale;
	    break;
	  case FH_X:
	    d_hessian[hidx_lin] = x;
	    break;
	  case FH_Y:
	    d_hessian[hidx_lin] = y;
	    break;
	  case FH_RESULT:
	    d_hessian[hidx_lin] = result;
	    break;
	  case FH_CENSURE:
	    d_hessian[hidx_lin] = censure;
	  case FH_RESULT_BIT_SET:
	    if(Dxx+Dyy > 0.f)
	      setLastBit(result);
	    else
	      clearLastBit(result);
	    d_hessian[hidx_lin] = result;
	    break;
	  default:
	    d_hessian[hidx_lin] = result;
	    break;
	  }
      }
  }
  

  void run_fasthessian_kernel(dim3 grid, dim3 threads, float * d_hessian, int octave)
  {
  	fasthessian_kernel<<< grid, threads >>>(d_hessian, octave);
	}
	
	void run_eval_component_kernel(dim3 grid, dim3 threads, float * d_hessian, int octave, fh_component comp)
	{
		eval_component_kernel<<< grid, threads >>>(d_hessian, octave, (int)comp);
	}

} // namespace asrl
