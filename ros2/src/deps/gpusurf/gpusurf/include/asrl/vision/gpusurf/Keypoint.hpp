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

/**
 * @file   Keypoint.hpp
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Tue Apr 20 20:16:39 2010
 * $Rev$
 * @brief A keypoint class that has a few more fields than are available in OpenCV
 * 
 * 
 */

#ifndef ASRL_SURF_KEYPOINT
#define ASRL_SURF_KEYPOINT

namespace asrl {

  /**
   * \class Keypoint
   * \brief A keypoint class used on the GPU.
   *
   *  It is very difficult to use complicated header files with CUDA.
   *  For example, both boost and opencv cause problems. Hence, we must
   *  create our own keypoint type and use that in the CUDA functions.
   */
  
  class Keypoint
  {
    public:
    Keypoint():
    x(0.f), y(0.f),size(0.f),response(0.f),angle(0.f),angle_response(0.f),octave(0){}
    float x;
    float y;
    float size;
    float response;
    float angle;
    float angle_response;
    float sigma_xx;
    float sigma_xy;
    float sigma_yy;
    float octave;
    float threshold_region;
    /// Try to keep the keypoint at 64 bytes
    float _reserved[5];
  };

  /**
   * The layout of a keypoint so the elements may be grabbed as an array. 
   * 
   */
  enum KeypointLayout {
    SF_X,
    SF_Y,
    SF_SIZE,
    SF_RESPONSE,
    SF_ANGLE,
    SF_ANGLE_RESPONSE,
    SF_SIGMA_XX,
    SF_SIGMA_XY,
    SF_SIGMA_YY,
    SF_OCTAVE,
    SF_FEATURE_STRIDE = sizeof(Keypoint) / sizeof(float)
  };

} // namespace asrl

#endif // ASRL_SURF_KEYPOINT
