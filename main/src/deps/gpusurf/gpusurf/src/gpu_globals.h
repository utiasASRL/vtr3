/**
 * @file   gpu_globals.h
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Tue Apr 20 20:16:39 2010
 * $Rev$
 * @brief  Global configuration variables and structure definitions.
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


#ifndef ASRL_GPU_GLOBALS_FTK_H
#define ASRL_GPU_GLOBALS_FTK_H

#include <asrl/vision/gpusurf/Keypoint.hpp>

/**
 * \def ASRL_SURF_MAX_INTERVALS
 * \brief The maximum number of intervals that device memory is reserved for.
 *
 */
#define ASRL_SURF_MAX_INTERVALS 6

/**
 * \def ASRL_SURF_MAX_OCTAVES
 * \brief The maximum number of octaves that device memory is reserved for.
 *
 */
#define ASRL_SURF_MAX_OCTAVES 6

/**
 * \def ASRL_SURF_MAX_FEATURES
 * \brief The maximum number of features that memory is reserved for.
 *
 */
#define ASRL_SURF_MAX_FEATURES 4096

/**
 * \def ASRL_SURF_MAX_CANDIDATES
 * \brief The maximum number of features (before subpixel interpolation) that memory is reserved for.
 *
 */
#define ASRL_SURF_MAX_CANDIDATES 6120

/**
 * \def ASRL_SURF_DESCRIPTOR_DIM
 * \brief The dimension of the SURF descriptor
 *
 */
#define ASRL_SURF_DESCRIPTOR_DIM 64

/**
 * \def ASRL_SURF_MAX_REGIONS
 * \brief The maximum number of regions to process an adaptive threshold in
 */
#define ASRL_SURF_MAX_REGIONS 16

/**
 * ASRL_SURF_HISTOGRAM_BUCKETS
 * \brief The number of buckets in an adaptive threshold histogram
 */
#define ASRL_SURF_HISTOGRAM_BUCKETS 128

/**
 * ASRL_SURF_HISTOGRAM_SATURATION 
 * \brief The saturation point of a histogram. Any strength values above this will land in the last bucket.
 */
#define ASRL_SURF_HISTOGRAM_SATURATION 30.f

namespace asrl
{
  // Forward declaration
  class GpuSurfOctave;

  /** 
   * \class SurfOctaveParameters
   * \brief A structure which holds the constant parameters that describe an octave layout.
   *
   */
  struct SurfOctaveParameters
  {
    /// The width of the octave buffer.
    int x_size;
    /// The height of the octave buffer.
    int y_size;
    /// The number of intervals in the octave.
    int nIntervals;
    /// The size of the octave border in pixels.
    int border;
    /// The step size used in this octave in pixels.
    int step;

    /// Mask sizes derived from the mask parameters
    float mask_width;
    /// Mask sizes derived from the mask parameters
    float mask_height;
    /// Mask sizes derived from the mask parameters
    float dxy_center_offset;
    /// Mask sizes derived from the mask parameters
    float dxy_half_width;
    /// Mask sizes derived from the mask parameters
    float dxy_scale;
    /// The target number of features for each region in this octave
    int region_target;
  };






  /** 
   * Initialize global variables used by the SURF detector
   * 
   * @param imWidth   The width of the integral image
   * @param imHeight  The height of the integral image
   * @param octaves   The octave parameters
   * @param regions_horizontal The number of regions horizontally used to compute adaptive thresholding
   * @param regions_vertical The number of regions vertically used to compute adaptive thresholding
   */
  void init_globals(int imWidth, int imHeight, GpuSurfOctave * octaves, int nOctaves, int regions_horizontal, int regions_vertical);

  /** 
   * @return A variable at gpusurf.cu file scope that says if the constant memory has been initialized.
   */
  bool & get_s_initialized();

  /** 
   * 
   * @return A variable at gpusurf.cu file scope that holds the initialized image width
   */
  int & get_s_initWidth();

  /** 
   * 
   * @return A variable at gpusurf.cu file scope that holds the initialized image height
   */
  int & get_s_initHeight();

  /** 
   * 
   * @return A __constant__ variable at gpusurf.cu file scope that holds octave parameters
   */
  SurfOctaveParameters * get_d_octave_params();

  /** 
   * 
   * @return A __constant__ variable at gpusurf.cu file scope that holds the octave scale constants
   */
  
  float * get_d_hessian_scale();

  /** 
   * 
   * @return A __constant__ variable at gpusurf.cu file scope that holds the hessian buffer row stride.
   */
  int * get_d_hessian_stride();  

  /** 
   * 
   * @return A __constant__ variable at gpusurf.cu file scope that holds the number of regions in x and y
   */
  int * get_d_regions();


} // namespace asrl





#endif // ASRL_GPU_GLOBALS_FTK_H
