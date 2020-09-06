#include "adaptive_threshold.h"

namespace asrl {

  __global__ void find_adaptive_thresholds_kernel(unsigned int * d_histograms, float * d_thresholds, int octave, float basic_threshold)
  {
    // Each block handles one histogram.
    __shared__ unsigned int histogram[ASRL_SURF_HISTOGRAM_BUCKETS];
    int ridx = blockIdx.x + blockIdx.y * d_regions[0];
    
    if(threadIdx.x < ASRL_SURF_HISTOGRAM_BUCKETS)
      histogram[threadIdx.x] = d_histograms[ridx * ASRL_SURF_HISTOGRAM_BUCKETS + threadIdx.x];

    __syncthreads();
    
    // Okay, this is the dead, simplest version of this code. It's not very GPU, but it may be fast, and 
    // it is a good first start.
    if(threadIdx.x == 0)
      {
	// sum the keypoint count startign with the strongest bucket and 
	// continuing until we have exceeded the target amount
	__shared__ int sum;
	sum = 0;
	__shared__ int i;
	for(i = ASRL_SURF_HISTOGRAM_BUCKETS - 1; i >= 0; i--)
	  {
	    sum += histogram[i];
	    if(sum > d_octave_params[octave].region_target)
	      break;
	  }

	// Now i is the index of the bucket
	// Compute the inverse of the functions that mapped strengths to buckets.
	// it is a crazy function.
	//                            exp(bucket * (  log((SATURATION                      - threshold       + 1e-3)*1000)) /        NBUCKETS          )*1e-3 + threshold - 1e-3;
	float adaptive_threshold = __expf((i)      * (__logf((ASRL_SURF_HISTOGRAM_SATURATION - basic_threshold + 1e-3f)*1000.f))/ASRL_SURF_HISTOGRAM_BUCKETS)*1e-3f + basic_threshold - 1e-3f;
	d_thresholds[ridx] = adaptive_threshold;

      }
  }

  
  void run_find_adaptive_thresholds_kernel(unsigned int * d_histograms, float * d_thresholds, int regions_horizontal, int regions_vertical, int octave, float basic_threshold)
  {
    dim3 grid,threads;
    
    threads.x = ASRL_SURF_HISTOGRAM_BUCKETS;
    threads.y = 1;
    threads.z = 1;
    
    grid.x = regions_horizontal;
    grid.y = regions_vertical;
    grid.z = 1;

    find_adaptive_thresholds_kernel <<< grid, threads >>> (d_histograms,d_thresholds, octave, basic_threshold);
  }

} // namespace asrl
