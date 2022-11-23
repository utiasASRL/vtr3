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

#include "detector.h"
#include "timing.h"
#include "fasthessian.h"
#include "non_max_suppression.h"
#include "keypoint_interpolation.h"
#include "GpuSurfFeatures.hpp"
#include "GpuSurfOctave.hpp"
#include <fstream>
#include "adaptive_threshold.h"

namespace asrl {
  void run_surf_detector(float * d_hessianBuffer, GpuSurfOctave & octave, int octaveIdx, GpuSurfFeatures & features, 
			 float basic_threshold, int fh_x_threads, int fh_y_threads,
			 int nonmax_x_threads, int nonmax_y_threads, CudaSynchronizedMemory<unsigned int> & histograms,
			 CudaSynchronizedMemory<float> & thresholds, int regions_horizontal, int regions_vertical)
  {
    /////////////////
    // FASTHESSIAN //
    /////////////////
    // GlobalTimer.start("compute interest operator");
    dim3 threads; 

    threads.x = fh_x_threads;
    threads.y = fh_y_threads;
    threads.z = octave.intervals();

    dim3 grid;
    grid.x = ( (octave.width()  + threads.x - 1) / threads.x);
    grid.y = ( (octave.height() + threads.y - 1) / threads.y);
    grid.z = 1;
	
    if(octave.valid()) {
      run_fasthessian_kernel(grid, threads, d_hessianBuffer, octaveIdx);
      ASRL_CHECK_CUDA_ERROR("Finding fasthessian");
    }

    // GlobalTimer.stop("compute interest operator");

    ////////////
    // NONMAX //
    ////////////
    // GlobalTimer.start("nonmax suppression");
    // Reset the candidate count.
    cudaMemset(features.featureCounterMem().d_get() + 1,0,sizeof(int));
    // clear the adaptive threshold histograms.
    histograms.memsetDevice(0);


    threads.x = nonmax_x_threads;
    threads.y = nonmax_y_threads;
    threads.z = 1;//octave.intervals();
	
    grid.x = ( (octave.width()  + (threads.x) - 1) / (threads.x));
    grid.y = ( (octave.height() + (threads.y-2) - 1) / (threads.y-2));
    grid.z = 1;


    size_t sharedBytes = (threads.x + 2) * threads.y * octave.intervals() * sizeof(float);
    run_surf_nonmaxonly_kernel(grid, threads, sharedBytes, d_hessianBuffer,
			       octaveIdx, features.rawFeatureMem().d_get(), features.featureCounterMem().d_get() + 1,
			       basic_threshold, histograms.d_get());

    ASRL_CHECK_CUDA_ERROR("Running Nonmax, octave " << octaveIdx);

    // GlobalTimer.stop("nonmax suppression");


    ////////////////////////
    // ADAPTIVE THRESHOLD //
    ////////////////////////
    // GlobalTimer.start("find thresholds");
    run_find_adaptive_thresholds_kernel(histograms.d_get(), thresholds.d_get(), regions_horizontal, regions_vertical, octaveIdx, basic_threshold);
    ASRL_CHECK_CUDA_ERROR("Running adaptive threhsold kernel, octave " << octaveIdx);

    // GlobalTimer.stop("find thresholds");
    /*
#if 0
    histograms.pullFromDevice();
    std::stringstream fname;
    fname << "hist-" << octaveIdx << ".txt";
    std::ofstream fout(fname.str().c_str());
    for(int r = 0; r < ASRL_SURF_MAX_REGIONS*ASRL_SURF_MAX_REGIONS; r++)
      {
	for(int i = 0; i < ASRL_SURF_HISTOGRAM_BUCKETS; i++)
	  {
	    fout << histograms[r*ASRL_SURF_HISTOGRAM_BUCKETS + i] << '\t';
	  }
	fout << std::endl;
      }
    
    thresholds.pullFromDevice();


    std::stringstream fnameb;
    fnameb << "threshold-" << octaveIdx << ".txt";
    std::ofstream foutb(fnameb.str().c_str());
    for(int r = 0; r < ASRL_SURF_MAX_REGIONS*ASRL_SURF_MAX_REGIONS; r++)
      {
	foutb << thresholds[r] << '\t';
      }
#endif
    */
    ///////////////////
    // INTERPOLATION //
    ///////////////////
    // GlobalTimer.start("keypoint interpolation");
    
    run_fh_interp_extremum(d_hessianBuffer,
			   features.deviceFeatures(), 
			   features.rawFeatureMem().d_get(), 
			   features.featureCounterMem().d_get(),
			   features.featureCounterMem().d_get() + 1, 
			   thresholds.d_get());
    ASRL_CHECK_CUDA_ERROR("Running subpixel interpolation");

    features.featureCounterMem().pullFromDevice();
    features.setDirty();
    // GlobalTimer.stop("keypoint interpolation");

  } // run_surf_detector()

} // namespace asrl
