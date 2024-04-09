/**
 * @file   GpuSurfOctave.hpp
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Mon Apr  5 11:27:20 2010
 *
 * @brief  A class representing a single octave used in the GPU SURF algorithm
 *
 * The class includes all memory allocation and parameters used to compute the octave.
 * Memory for the detector is mirrored on the host mostly for debugging.
 * Descriptions of the box filter parameters are provided in the asrl::GpuSurfOctave class description.
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


#ifndef ASRL_GPU_SURF_OCTAVE_HPP
#define ASRL_GPU_SURF_OCTAVE_HPP

#include "CudaSynchronizedMemory.hpp"
#include "gpu_globals.h"

namespace asrl {

  /**
   * @class GpuSurfOctave
   * @brief A class that reserves memory on the GPU for processing the interest point operator.
   *
   * This class sets up the memory and computes the parameters for each octave for the Fast Hessian computation. Custom box filter dimensions can be specified using the \a l1-l4 mask parameters, which are utilized as illustrated in the following diagram.
   \image html mask_parameters.png "Dxx/Dyy and Dxy filter mask parameters"
   */
  class GpuSurfOctave {
  public:
    /**
     * The default constructor. This is included so the octave can be stored in a standard library container.
     */
    GpuSurfOctave();
    
    
    /**
     * Initialization of the components of each GpuSurfOctave object.
     */
    void init(int img_width, int img_height, float l1, float l2, float l3, float l4, float edge_scale, float base_scale, int octave, int baseStep, int nIntervals, int regions_target0);
    
    /**
     * @return The parameters for the octave.
     */
    operator SurfOctaveParameters();
    
    /**
     * @return The half-width of the Dxx/Dyy mask.
     */
    float mask_width() {return m_mask_width;}
    
    /**
     * @return The half-height of the Dxx/Dyy mask.
     */
    float mask_height(){return m_mask_height;}
    
    /**
     * @return A flag specifying whether or not we have a valid size for the Hessian buffer.
     */
    bool valid(){ return m_valid;}
    
    /**
     * @return The size of the border (in pixels) such that no evaluated filter exceeds the bounds of the image.
     */
    int border(){ return m_border;}
    
    /**
     * @return The height of the Hessian buffer.
     */
    int height(){ return m_height;}
    
    /**
     * @return The width of the Hessian buffer.
     */
    int width(){ return m_width;}
    
    /**
     * @return The number of intervals in the octave.
     */
    int intervals(){ return m_intervals;}
    
    /**
     * @return The step size (in pixels) for the octave.
     */
    int step() { return m_step; }    
    
    /**
     * @return The scales in the octave.
     */
    const float * scales(){ return m_scales; }
    
    /**
     * @return The 128-byte-divisible stride for a row of the octave
     */
    int stride() {

      // 128 bytes is 32 floats.
      int n32s = width() / 32;
      int n32s_remainder = width() % 32;
      if(n32s_remainder != 0)
	n32s++;


      return n32s * 32;
    }

  private:
  	/// Dxx/Dyy mask half-width
    float m_mask_width;
    /// Dxx/Dyy mask half-height
    float m_mask_height;
    /// Filter mask parameter l1
    float m_l1;
    /// Filter mask parameter l2
    float m_l2;
    /// Filter mask parameter l3
    float m_l3;
    /// Filter mask parameter l4
    float m_l4;
    /// Relative filter response weighting used as: response = Dxx*Dyy - edge_scale*Dxy*Dxy
    float m_edge_scale;
    /// Flag for whether or not a valid Hessian buffer can be created
    bool m_valid;
    /// Size of the border to ensure filter lookups do not exceed the image boundaries
    int m_border;
    /// Hessian buffer width
    int m_width;
    /// Hessian buffer height
    int m_height;
    /// Number of intervals in the octave
    int m_intervals;
    /// Step size (in pixels)
    int m_step;

    /// The scales in the octave
    float m_scales[ASRL_SURF_MAX_INTERVALS];

    /// Target number of features in each region of the image.
    int m_regionTarget;
  };

} // namespace asrl

#endif // ASRL_SURF_OCTAVE_HPP
