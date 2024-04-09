/**
 * @file   non_max_suppression.h
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Wed Apr 21 06:48:36 2010
 * 
 * @brief  Nonmax suppression functions.
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

#ifndef ASRL_NON_MAX_SUPPRESSION
#define ASRL_NON_MAX_SUPPRESSION

namespace asrl {
			    
  /** 
   * Find maxima within 3D the interest operator buffer
   * 
   * @param grid			        The grid configuration
   * @param threads		        The thread configuration
   * @param sharedBytes       The amount of shared memory used by the kernel
   * @param d_hessian	        The device buffer to store the fast hessian
   * @param octave            The octave number to be working on
   * @param d_maxmin          Device buffer for raw features
   * @param d_maxmin_counter  Device counter for number of raw features
   * @param threshold         The interest operator threshold
   * @param d_histograms      The device pointer to the strength histogram memory.
   */
  void run_surf_nonmaxonly_kernel(dim3 grid, dim3 threads, size_t sharedBytes, float * d_hessian, int octave, int4 * d_maxmin, unsigned int * d_maxmin_counter, float threshold,
				  unsigned int * d_histograms);

} // namespace asrl

#endif // ASRL_NON_MAX_SUPPRESSION

