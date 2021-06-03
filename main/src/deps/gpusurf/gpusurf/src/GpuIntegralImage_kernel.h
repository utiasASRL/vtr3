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

#ifndef ASRL_GPU_INTEGRAL_IMAGE_KERNEL
#define ASRL_GPU_INTEGRAL_IMAGE_KERNEL

#include <cuda_runtime_api.h>
#include <cudpp.h>

// Matrix transpose block dimension
#define ASRL_TRANSPOSE_BLOCK_DIM 16

namespace asrl {

  /**
   * The C interface to the CUDA kernel that builds the integral image
   *
   * @param width the width of the input image
   * @param height the height of the input image
   * @param char_data a device pointer to the input image
   * @param norm_data a device pointer for intermediate storage
   * @param trans_data a device pointer for intermediate storage
   * @param output_int_img a device pointer to the output buffer for the integral image
   * @param norm_pitch the pitch of the rows of the image array in the standard orientation
   * @param trans_pitch the pitch of the rows of the transposed image array
   * @param char_pitch the pitch of the rows of the character array
   * @param rowPlan a handle to the CUDPP plan structure governing the row scan operation
   * @param colPlan a handle to the CUDPP plan structure governing the column scan operation
   * @param stream An optional cuda stream.
   */
  void call_integral_kernel(size_t width, size_t height,
			    unsigned char * char_data,
			    float *norm_data,
			    float *trans_data,
			    cudaArray *output_int_img,
			    size_t norm_pitch,
			    size_t trans_pitch,
			    size_t char_pitch,
			    CUDPPHandle & rowPlan,
			    CUDPPHandle & colPlan,
					cudaStream_t /*stream*/);
	
	/**
	 * The interface to the unsigned char-to-float matrix transpose operation.
	 *
	 * @param grid    The kernel grid configuration
	 * @param block   The kernel block configuration
	 * @param odata   The output data (device)
	 * @param o_pitch The output data pitch
	 * @param idata   The input data (device)
	 * @param i_pitch The input data pitch
	 * @param width   The input data width
	 * @param height  The input data height
	 */
  void run_transpose_kernel_uchar(dim3 grid, dim3 block, float *odata, size_t o_pitch, unsigned char *idata, size_t i_pitch, size_t width, size_t height);
  
	/**
	 * The interface to the float-to-float matrix transpose operation.
	 *
	 * @param grid    The kernel grid configuration
	 * @param block   The kernel block configuration
	 * @param odata   The output data (device)
	 * @param o_pitch The output data pitch
	 * @param idata   The input data (device)
	 * @param i_pitch The input data pitch
	 * @param width   The input data width
	 * @param height  The input data height
	 */
  void run_transpose_kernel_float(dim3 grid, dim3 block, float *odata, size_t o_pitch, float *idata, size_t i_pitch, size_t width, size_t height);

} // namespace asrl

#endif // ASRL_GPU_INTEGRAL_IMAGE_KERNEL
