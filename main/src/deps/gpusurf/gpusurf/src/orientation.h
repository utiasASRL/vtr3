/**
 * @file   orientation.h
 * @authors Paul Furgale and Chi Hay Tong
 * 
 * @brief  Functions for calculating the orientation of a keypoint
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


#ifndef ORIENTATION_FTK_H
#define ORIENTATION_FTK_H

namespace asrl {
  
  /** 
   * Computes the orientation for a list of keypoints. This is the orientation calculation from the original SURF algorithm
   * 
   * @param d_features  A device pointer to the keypoint list.
   * @param nFeatures   The number of features available
   */
  void find_orientation(Keypoint * d_features, int nFeatures);
  
  /** 
   * Computes the orientation for a list of keypoints. This is an orientation calculation that is 10x faster than the original
   * 
   * @param d_features  A device pointer to the keypoint list.
   * @param nFeatures   The number of features available
   */
  void find_orientation_fast(Keypoint * d_features, int nFeatures);
}

#endif 

