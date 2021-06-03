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

#include "GpuIntegralImage_kernel.h"

namespace asrl {
  // convert unsigned chars into floats (scaled by 255.0f)
  // - second function does nothing - overloaded, but __device__ functions are inline, so no overhead
  __device__ void convert_dev(float & out, unsigned char in){ out = (float) in / 255.0f; }
  __device__ void convert_dev(float & out, float in){ out = in; }


  // matrix transpose operation (on the GPU)
  template <typename T>
  __global__ void transpose_kernel(float *odata, size_t o_pitch, T *idata, size_t i_pitch, size_t width, size_t height)
  {
    __shared__ float block[ASRL_TRANSPOSE_BLOCK_DIM][ASRL_TRANSPOSE_BLOCK_DIM+1];

    // read the matrix tile into shared memory
    unsigned int xBlock = __mul24(blockDim.x, blockIdx.x);
    unsigned int yBlock = __mul24(blockDim.y, blockIdx.y);
    unsigned int xIndex = xBlock + threadIdx.x;
    unsigned int yIndex = yBlock + threadIdx.y;

    if ((xIndex < width) && (yIndex < height))
      {
	// load block into shared memory
	unsigned int index_in = __mul24(i_pitch, yIndex) + xIndex;	// where from in data
	convert_dev(block[threadIdx.y][threadIdx.x], idata[index_in]);	// convert to float (if not already)
      }

    __syncthreads();

    // write it back to global memory
    xIndex = yBlock + threadIdx.x;
    yIndex = xBlock + threadIdx.y;
    if ((xIndex < height) && (yIndex < width))
      {
	unsigned int index_out = __mul24(o_pitch, yIndex) + xIndex;
	odata[index_out] = block[threadIdx.x][threadIdx.y];
      }
  }
  
  void run_transpose_kernel_uchar(dim3 grid, dim3 block, float *odata, size_t o_pitch, unsigned char *idata, size_t i_pitch, size_t width, size_t height)
  {
    transpose_kernel<unsigned char> <<< grid, block, 0>>> (odata, o_pitch, idata, i_pitch, width, height);
  }
  
  void run_transpose_kernel_float(dim3 grid, dim3 block, float *odata, size_t o_pitch, float *idata, size_t i_pitch, size_t width, size_t height)
  {
    transpose_kernel<float> <<< grid, block, 0>>> (odata, o_pitch, idata, i_pitch, width, height);
  }

} // namespace asrl


