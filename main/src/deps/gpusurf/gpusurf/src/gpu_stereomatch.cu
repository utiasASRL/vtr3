#include "gpu_stereomatch.h"
#include <cublas.h>


namespace asrl {

#define STM_THREADS 256

  __global__ void find_stereo_matches_kernel(Keypoint * d_lt, int n_lt, float * d_ltd,
					     Keypoint * d_rt, int n_rt, float * d_rtd,
					     /*float * d_C,*/ int * matches,  
					     float ytol, float ctol, 
					     float mind, float maxd,
					     float stol)
  {
    // tid is the unique index for the left feature this thread is processing.
    int tid = __mul24(blockIdx.x,STM_THREADS) + threadIdx.x;

    // Get data from the left feature.
    float y = d_lt[tid].y;
    float x = d_lt[tid].x;

    float strength = d_lt[tid].response;
    float scale = d_lt[tid].size;
    float octave_scale = 1 << (int)d_lt[tid].octave;

    float maxc = -FLT_MAX;
    int maxIdx = -1;

    __shared__ float ry[STM_THREADS];
    __shared__ float rx[STM_THREADS];
    __shared__ float rs[STM_THREADS];
    __shared__ float rc[STM_THREADS];

    for(int c = 0; c < n_rt; c+= STM_THREADS) {
      // Load the y coordinate of the comparison feature.
      ry[threadIdx.x] = d_rt[c+threadIdx.x].y;
      rx[threadIdx.x] = d_rt[c+threadIdx.x].x;
      rs[threadIdx.x] = d_rt[c+threadIdx.x].response;
      rc[threadIdx.x] = d_rt[c+threadIdx.x].size;
      __syncthreads();

      for(int i = 0; i < STM_THREADS; i++) {
	float d = x - rx[i];
	// Now each feature compares against the correlation for this pair of features.
	if(
	   c+i < n_rt 				                 // This element is in-bounds
	   && isLastBitSet(rs[i]) == isLastBitSet(strength) 	 // The blobs match (light/dark) 
	   && (fabs(y - ry[i])) < ytol * octave_scale            // the row is similar
	   && d >= mind                                          // the disparity is above the minimum threshold
	   && d <= maxd				                 // The disparity is less than the threshold.
	   && min(rc[i],scale)/max(rc[i],scale) > stol           // The scale differences are not too great.
	  )
	  {
	    // Load the descriptor correlation value.
	    //float cr = d_C[__mul24(c+i,ASRL_SURF_MAX_FEATURES) + tid];
	    float cr = 0.f;
	    for(int j = 0; j < ASRL_SURF_DESCRIPTOR_DIM; j++)
	      {
		cr += d_ltd[tid*ASRL_SURF_DESCRIPTOR_DIM + j] * d_rtd[ (c+i)*ASRL_SURF_DESCRIPTOR_DIM + j];
	      }
	    if(cr > maxc) {
	      maxc = cr;
	      maxIdx = c+i;
	    }
	  }
      }
      __syncthreads();
    }

    // Threshold the descriptor correlation.
    if(maxc < ctol) 
      maxIdx = -1;

    if( tid < n_lt ) {
      matches[tid] = maxIdx;
    }
  }

  void find_stereo_matches(Keypoint * d_lt, int n_lt, float * d_ltd,
			   Keypoint * d_rt, int n_rt, float * d_rtd,
			   /*float * d_C,*/ int * matches, 
			   float ytol, float ctol, float mind, float maxd,
			   float stol){

    //    printf("Finding stereo matches between %d and %d keypoints\n", n_lt, n_rt);
    //    printf("Tolerances: ytol: %f, ctol: %f, mind: %f, maxd: %f, stol: %f\n", ytol, ctol, mind, maxd, stol);
    dim3 threads(STM_THREADS,1,1);
    dim3 grid(1,1,1);
    grid.x = ( (n_lt + threads.x - 1) / threads.x);

    if ((n_lt > 0) && (n_rt > 0))
      find_stereo_matches_kernel<<< grid, threads >>>(d_lt, n_lt, d_ltd, d_rt, n_rt, d_rtd, /*d_C,*/ matches, ytol, ctol, mind, maxd, stol);
  } 

} // namespace asrl
