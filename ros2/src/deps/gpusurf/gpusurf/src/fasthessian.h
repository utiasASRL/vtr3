/**
 * @file   fasthessian.h
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Tue Apr 20 19:51:59 2010
 * 
 * @brief  Functions to calculate the fasthessian interest point operator on the GPU
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


#ifndef ASRL_FASTHESSIAN_H
#define ASRL_FASTHESSIAN_H


namespace asrl{

  /**
   * Runs the SURF fast hessian kernel
   *
   * @param grid			The grid configuration
   * @param threads		The thread configuration
   * @param d_hessian	The device buffer to store the fast hessian
   * @param octave		The octave number to be working on
   */
  void run_fasthessian_kernel(dim3 grid, dim3 threads, float * d_hessian, int octave);

  /** 
   * An enum specifying the components of the fasthessian algorithm
   * used in debugging.
   * 
   */
  enum fh_component {
    FH_DXX,
    FH_DYY,
    FH_DXX_DYY,
    FH_DXY,
    FH_CENSURE,
    FH_RESULT,
    FH_RESULT_BIT_SET,
    FH_LINEAR_IDX,
    FH_HIDX_X_IDX,
    FH_HIDX_Y_IDX,
    FH_FSCALE,
    FH_X,
    FH_Y
  };

 /**
   * Evaluates a component of the SURF fasthessian algorithm at every point
   * in the buffer. Used for debugging.
   *
   * @param grid			The grid configuration
   * @param threads		The thread configuration
   * @param d_hessian	The device buffer to store the fast hessian
   * @param octave		The octave number to be working on
   * @param comp 			The component to return in the octave buffers
   */
  void run_eval_component_kernel(dim3 grid, dim3 threads, float * d_hessian, int octave, fh_component comp);

} // namespace asrl

#endif // ASRL_FASTHESSIAN_H


