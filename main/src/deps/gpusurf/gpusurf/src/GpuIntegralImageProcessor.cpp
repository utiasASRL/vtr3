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

#include "GpuIntegralImageProcessor.hpp"
#include <cudpp.h>
#include <cuda.h>
#include <builtin_types.h>
#include <channel_descriptor.h>
#include <iostream>
#include "GpuIntegralImage_kernel.h"
#include "GpuIntegralImage.hpp"
#include "assert_macros.hpp"


namespace asrl {

  GpuIntegralImageProcessor::GpuIntegralImageProcessor(int width, int height)
  {
    cudaError_t err;
    m_width = width;
    m_height = height;

    // allocate GPU memory (char, normal, and transposed data)
    unsigned char * char_ptr;
    err = cudaMallocPitch( (void**) &char_ptr, &char_pitch, width*sizeof(unsigned char), height);
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to allocate CUDA char* input image.");
    char_data.reset(char_ptr,&cudaFree);

    float * norm_ptr;
    err = cudaMallocPitch( (void**) &norm_ptr, &norm_pitch, width*sizeof(float), height);
    ASRL_ASSERT_EQ(err, cudaSuccess, "Unable to allocate CUDA normally oriented float integral image.");
    norm_data.reset(norm_ptr,&cudaFree);

    float * trans_ptr;
    err = cudaMallocPitch( (void**) &trans_ptr, &trans_pitch, height*sizeof(float), width);
    ASRL_ASSERT_EQ(err, cudaSuccess, "Unable to allocate CUDA transpose oriented integral image.");
    trans_data.reset(trans_ptr,&cudaFree);

    CUDPPConfiguration config = { CUDPP_SCAN, CUDPP_ADD, CUDPP_FLOAT, CUDPP_OPTION_FORWARD | CUDPP_OPTION_INCLUSIVE };
    CUDPPResult result = cudppPlan(&colPlan, config, width*height, width, trans_pitch/sizeof(float));
    ASRL_ASSERT_EQ(result, CUDPP_SUCCESS, "Error creating scanPlan (column scan):" << result);

    result = cudppPlan(&rowPlan, config, width*height, height, norm_pitch/sizeof(float));
    ASRL_ASSERT_EQ(result,CUDPP_SUCCESS, "Error creating scanPlan (row scan): " << result);
  }

  GpuIntegralImageProcessor::~GpuIntegralImageProcessor()
  {
    try{
      CUDPPResult result = cudppDestroyPlan(colPlan);
      if(result != CUDPP_SUCCESS)
	std::cerr << "Unable to destroy column plan. Err code: " << result;
    } catch(std::exception const & e) {
      std::cout << e.what() << std::endl;
    }
    try{
      CUDPPResult result = cudppDestroyPlan(rowPlan);
      if(result != CUDPP_SUCCESS)
	std::cerr << "Unable to destroy column plan. Err code: " << result;
    } catch(std::exception const & e) {
      std::cout << e.what() << std::endl;
    }
  }


  void GpuIntegralImageProcessor::upload(const cv::Mat & image)
  {
    ASRL_ASSERT_EQ(image.type(),CV_8UC1,"The image must be single channel, 8 bit");
    ASRL_ASSERT(image.isContinuous(),"The image must be tightly packed. width: " << image.cols << ", step: " << image.step);
    // This is synchronous. Asynchronous calls have to go through page-locked memory. I'm not sure it would be faster to
    // copy the buffer to PLM and then transfer asynchronously...
    cudaError_t err = cudaMemcpy2D( (void*) char_data.get(), char_pitch, (void*) image.ptr(), image.cols*sizeof(unsigned char),
				    image.cols*sizeof(unsigned char), image.rows, cudaMemcpyHostToDevice );
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to copy image to GPU: (" << err << "): " << cudaGetErrorString(err));		
  }

  void GpuIntegralImageProcessor::process(const cv::Mat & image, GpuIntegralImage & outImage, cudaStream_t stream){
    upload(image);
    process(outImage, stream);
  }

  void GpuIntegralImageProcessor::process(GpuIntegralImage & outImage, cudaStream_t stream)
  {
    call_integral_kernel((size_t) width(), (size_t)height(),
			 char_data.get(),		// initial storage on the gpu of the unsigned char data
			 norm_data.get(), 
			 trans_data.get(),	// storage of floats on the gpu
			 outImage.d_get(),
			 norm_pitch, 
			 trans_pitch, 
			 char_pitch,
			 rowPlan,
			 colPlan,
			 stream);
  }

} // namespace asrl
