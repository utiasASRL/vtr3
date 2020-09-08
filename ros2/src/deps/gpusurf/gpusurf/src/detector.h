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
 * @file   detector.h
 * @author Paul Furgale and Chi Hay Tong
 * @date   Sun Apr 25 14:56:54 2010
 * 
 * @brief  A collected function that runs the GPUSURF detector
 * 
 * 
 */


#ifndef ASRL_GPUSURF_DETECTOR_H
#define ASRL_GPUSURF_DETECTOR_H

#include "CudaSynchronizedMemory.hpp"

namespace asrl {
  class GpuSurfOctave;
  class GpuSurfFeatures;
  
  /** 
   * A Function that runs the gpusurf detector. This includes
   * <ol>
   * <li> Computing the interest operator </li>
   * <li> Finding maxima in the 3d buffer </li>
   * <li> Computing subpixel interpolation of the keypoints </li>
   * </ol>
   * 
   * @param d_hessianBuffer    A device pointer to the buffer where the interest operator results are stored.
   * @param octave             The parameters of the octave being processed
   * @param octaveIdx          The index of the octave being processed
   * @param features           Memory to hold the resultant features.
   * @param basic_threshold    The initial threshold used to reject uninteresting blobs
   * @param fh_x_threads       The number of threads per block used to compute the interest operator (dimension 1)
   * @param fh_y_threads       The number of threads per block used to compute the interest operator (dimension 2)
   * @param nonmax_x_threads   The number of threads per block used to find maxima (dimension 1) 
   * @param nonmax_y_threads   The number of threads per block used to find maxima (dimension 2)
   * @param histograms         Device memory for the histograms used to process the adaptive threshold
   * @param thresholds         Device memory used to store the adaptive threshold
   * @param regions_horizontal The number of horizontal regions to perform adaptive thresholding on.
   * @param regions_vertical   The numbero of vertical regions to perform adaptive thresholding on.
   */
  void run_surf_detector(float * d_hessianBuffer, GpuSurfOctave & octave, int octaveIdx, GpuSurfFeatures & features, 
			 float basic_threshold, int fh_x_threads, int fh_y_threads,
			 int nonmax_x_threads, int nonmax_y_threads, CudaSynchronizedMemory<unsigned int> & histograms,
			 CudaSynchronizedMemory<float> & thresholds, int regions_horizontal, int regions_vertical);

} // namespace asrl

#endif // ASRL_GPUSURF_DETECTOR_H
