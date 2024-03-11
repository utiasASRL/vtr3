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

#include "assert_macros.hpp"
#include "GpuIntegralImage_kernel.h"

namespace asrl {
  void call_integral_kernel(size_t width, size_t height,
			    unsigned char * char_data,		// initial storage on the gpu of the unsigned char data
			    float *norm_data, 
			    float *trans_data,	// storage of floats on the gpu
			    cudaArray *output_int_img,
			    size_t norm_pitch, 
			    size_t trans_pitch, 
			    size_t char_pitch,
			    CUDPPHandle & rowPlan,
			    CUDPPHandle & colPlan,
					cudaStream_t /*stream*/)
  {
    // transpose and convert the data into floats
    dim3 block(ASRL_TRANSPOSE_BLOCK_DIM, ASRL_TRANSPOSE_BLOCK_DIM, 1);
    dim3 grid( (int) ((block.x + width - 1) / block.x), (int) ((block.y + height - 1) / block.y), 1);

    //TIME_START("iimg: transpose 1");
    run_transpose_kernel_uchar(grid, block, trans_data, trans_pitch/sizeof(float), 
                                        char_data, char_pitch/sizeof(unsigned char), width, height);
    ASRL_CHECK_CUDA_ERROR_DBG("transpose 1");    

    // compute row scan on transposed image (columns)
    cudppMultiScan(colPlan, trans_data, trans_data, height, width);
    ASRL_CHECK_CUDA_ERROR_DBG("column scan");


    // transpose (again, to return to original orientation)
    grid.x = (int) ((block.x + height - 1) / block.x);
    grid.y = (int) ((block.y + width - 1) / block.y);
    
    run_transpose_kernel_float(grid, block, norm_data, norm_pitch/sizeof(float), 
                                trans_data, trans_pitch/sizeof(float), height, width);
    ASRL_CHECK_CUDA_ERROR_DBG("transpose 2");

    // compute row scan on re-transposed image (rows)
    cudppMultiScan(rowPlan, norm_data, norm_data, width, height);
    ASRL_CHECK_CUDA_ERROR_DBG("row scan");

    // transfer data to cudaArray
    cudaError_t err = cudaMemcpy2DToArray(output_int_img, 0, 0, norm_data, norm_pitch, 
                                          width*sizeof(float), height, cudaMemcpyDeviceToDevice);
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to copy the integral image to the texture buffer: (" << err << "): " << cudaGetErrorString(err));		
  }
}
