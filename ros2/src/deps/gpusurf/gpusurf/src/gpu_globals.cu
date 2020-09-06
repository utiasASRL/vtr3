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

namespace asrl {

  bool s_initialized = false;
  int s_initWidth = -1;
  int s_initHeight = -1;	// texturing the integral image
  __constant__ SurfOctaveParameters d_octave_params[ASRL_SURF_MAX_OCTAVES];
  __constant__ float d_hessian_scale[ASRL_SURF_MAX_INTERVALS*ASRL_SURF_MAX_OCTAVES];
  __constant__ int d_hessian_stride[1];
  __constant__ int d_regions[2];

  bool & get_s_initialized(){ return s_initialized;}
  int & get_s_initWidth()   { return s_initWidth;  }
  int & get_s_initHeight()  { return s_initHeight; }

  SurfOctaveParameters * get_d_octave_params() { return d_octave_params; }
  float * get_d_hessian_scale() { return d_hessian_scale;  }
  int * get_d_hessian_stride()  { return d_hessian_stride; }
  int * get_d_regions(){ return d_regions; }


} // namespace asrl

