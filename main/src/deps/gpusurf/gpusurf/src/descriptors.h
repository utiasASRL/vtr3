/**
 * @file   descriptors.h
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Tue Apr 20 19:47:40 2010
 * 
 * @brief  Functions to calculate the descriptors.
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

#ifndef ASRL_DESCRIPTORS_FTK_H
#define ASRL_DESCRIPTORS_FTK_H

namespace asrl {
  
  /** 
   * A function to calculate SURF descriptors from keypoints. The keypoint location, orientation and scale are
   * used in this function.
   * 
   * @param d_descriptors   The device pointer to descriptor memory.
   * @param d_features      The device pointer to feature memory.
   * @param nFeaturesFound  The number of features to be described. Features should be arranged linearly on the GPU
   * @param weighted        Whether or not to weigh the descriptor components with a Gaussian (slightly better recall performance without)
   */
  void compute_descriptors(float * d_descriptors, Keypoint * d_features, int nFeaturesFound, bool weighted);


} // namespace asrl

#endif 

