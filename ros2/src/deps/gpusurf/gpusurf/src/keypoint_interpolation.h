/**
 * @file   keypoint_interpolation.h
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Wed Apr 21 06:46:02 2010
 * $Rev$
 * @brief  Keypoint interpolation functions.
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

#ifndef ASRL_KEYPOINT_INTERPOLATION_HPP
#define ASRL_KEYPOINT_INTERPOLATION_HPP


namespace asrl {
  class Keypoint;
  /** 
   * Interpolate a set of keypoints on the GPU
   * 
   * @param d_hessian         The interest operator buffer
   * @param d_features        Device buffer for interpolated features
   * @param d_maxmin          Device buffer for raw features
   * @param d_feature_counter Device counter for number of interpolated features
   * @param d_max_min_counter Device counter for number of raw features
   * @param d_threshold       Device pointers to the threshold
   */
  void run_fh_interp_extremum(float * d_hessian, Keypoint * d_features, int4 * d_maxmin, unsigned int * d_feature_counter, unsigned int * d_max_min_counter, float * d_threshold);
} // namespace asrl


#endif // ASRL_KEYPOINT_INTERPOLATION_HPP
