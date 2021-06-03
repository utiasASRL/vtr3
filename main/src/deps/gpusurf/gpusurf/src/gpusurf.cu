/**
* @file   gpusurf.cu
* @authors Paul Furgale and Chi Hay Tong
* @date   Mon Feb 22 15:32:33 2010
* $Rev$ 
* @brief  The collected cu file containing all cuda code for GPU SURF
* 
* Our GPU SURF code uses constant memory to speed up execution.
* Constant memory is localized at the file scope. Hence, we must
* compile all blocks under a single file. To ease readability 
* we use separate files for logical blocks and then collect those
* files in a single compile here.
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

#include "gpu_globals.h"
#include "gpu_area.h"
#include "fasthessian.h"
#include "non_max_suppression.h"
#include "keypoint_interpolation.h"
#include "orientation.h"
#include "descriptors.h"
#include "gpu_stereomatch.h"

#include "gpu_globals.cu"
#include "GpuIntegralImage_kernel.cu"
#include "gpu_area.cu"
#include "fasthessian.cu"
#include "non_max_suppression.cu"
#include "keypoint_interpolation.cu"
#include "orientation.cu"
#include "descriptors.cu"
#include "adaptive_threshold.cu"
#include "gpu_stereomatch.cu"

