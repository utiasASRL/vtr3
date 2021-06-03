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

#include "keypoint_interpolation.h"

namespace asrl {

  // Notes:
  // 1) perhaps fermi doesn't like float[2][2];
  // 2) perhaps we overflow the atomic counter.

#define MID_IDX 1
  __global__ void fh_interp_extremum_passthrough(float * d_hessian, Keypoint * d_features, int4 * d_maxmin, unsigned int * d_feature_counter, unsigned int * d_max_min_counter, float * d_threshold)
  { 
    if(blockIdx.x >= *d_max_min_counter)
      return;

    int octave = d_maxmin[blockIdx.x].w;  

    unsigned int i = ASRL_SURF_MAX_FEATURES;
    int hidx_x = d_maxmin[blockIdx.x].x-1 + threadIdx.x;
    int hidx_y = d_maxmin[blockIdx.x].y-1 + threadIdx.y;
    int hidx_z = d_maxmin[blockIdx.x].z-1 + threadIdx.z;

    int hidx_lin = hidx_x + d_hessian_stride[0] * hidx_y + d_hessian_stride[0] * d_octave_params[octave].y_size * hidx_z;

    __shared__ float fh_vals[3][3][3];
    __shared__ Keypoint p;

    fh_vals[threadIdx.z][threadIdx.y][threadIdx.x] = d_hessian[hidx_lin];
    __syncthreads();

    if(threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0){
      // Calculate the region index
      int ridx = hidx_x * d_regions[0] / d_octave_params[octave].x_size + (hidx_y * d_regions[1] / d_octave_params[octave].y_size) * d_regions[0];
      if(fh_vals[MID_IDX][MID_IDX][MID_IDX] < d_threshold[ridx])
	return;

      // if the step is near the interpolation region, perform it
	
      // Get a new feature index.
      // Using -1 here avoids wraparound
      i = atomicInc(d_feature_counter,(unsigned int) -1);
      
      if(i < ASRL_SURF_MAX_FEATURES) {
        p.x =  ((float)d_maxmin[blockIdx.x].x) *  (float)d_octave_params[octave].step + d_octave_params[octave].border;
        p.y =  ((float)d_maxmin[blockIdx.x].y) *  (float)d_octave_params[octave].step + d_octave_params[octave].border;
        
        p.size =  0.8f * d_hessian_scale[ASRL_SURF_MAX_INTERVALS*octave + d_maxmin[blockIdx.x].z];
        
        p.octave = octave;
	
        bool laplacian = isLastBitSet(fh_vals[MID_IDX][MID_IDX][MID_IDX]);
        p.response = fh_vals[MID_IDX][MID_IDX][MID_IDX];

        if(laplacian)
          setLastBit(p.response);
        else
          clearLastBit(p.response);
        
        // Save the covariance of p
        p.sigma_xx = 1.f;
        p.sigma_yy = 1.f;
        p.sigma_xy = 1.f;
        
        // Should we split up this transfer over many threads?
        d_features[i] = p;
      } else {
        *d_feature_counter = ASRL_SURF_MAX_FEATURES;
      } // If there is still room in memory for the feature
   
    } // If this is thread 0.

    //     __syncthreads();
    //     // Transfer the feature back.
    //     if(i < MAX_N_FEATURES)
    //       {
    // 	int tid = threadIdx.x + __mul24(threadIdx.y,3) + __mul24(threadIdx.z,9);
    // 	if(tid < 6)
    // 	  {
    // 	    ((float*)&d_features[i])[tid] = ((float*)&p)[tid];
    // 	  }
    //       }
  }



#define MID_IDX 1
  __global__ void fh_interp_extremum(float * d_hessian, Keypoint * d_features, int4 * d_maxmin, unsigned int * d_feature_counter, unsigned int * d_max_min_counter, float * d_threshold)
  { 
    if(blockIdx.x >= *d_max_min_counter)
      return;

    int octave = d_maxmin[blockIdx.x].w;  

    unsigned int i = ASRL_SURF_MAX_FEATURES;
    int hidx_x = d_maxmin[blockIdx.x].x-1 + threadIdx.x;
    int hidx_y = d_maxmin[blockIdx.x].y-1 + threadIdx.y;
    int hidx_z = d_maxmin[blockIdx.x].z-1 + threadIdx.z;

    int hidx_lin = hidx_x + d_hessian_stride[0] * hidx_y + d_hessian_stride[0] * d_octave_params[octave].y_size * hidx_z;

    __shared__ float fh_vals[3][3][3];
    __shared__ Keypoint p;

    fh_vals[threadIdx.z][threadIdx.y][threadIdx.x] = d_hessian[hidx_lin];
    __syncthreads();

    if(threadIdx.x == 0 && threadIdx.y == 0 && threadIdx.z == 0){
      // Calculate the region index
      int ridx = hidx_x * d_regions[0] / d_octave_params[octave].x_size + (hidx_y * d_regions[1] / d_octave_params[octave].y_size) * d_regions[0];
      if(fh_vals[MID_IDX][MID_IDX][MID_IDX] < d_threshold[ridx])
	return;

      __shared__ float H[3][3];

      //dyy
      H[0][0] =      fh_vals[MID_IDX    ][MID_IDX + 1][MID_IDX    ] 
	-       2.0f*fh_vals[MID_IDX    ][MID_IDX    ][MID_IDX    ]
	+            fh_vals[MID_IDX    ][MID_IDX - 1][MID_IDX    ];

      //dxx
      H[1][1] =      fh_vals[MID_IDX    ][MID_IDX    ][MID_IDX + 1] 
	-       2.0f*fh_vals[MID_IDX    ][MID_IDX    ][MID_IDX    ]
	+            fh_vals[MID_IDX    ][MID_IDX    ][MID_IDX - 1];
      //dss
      H[2][2] =      fh_vals[MID_IDX + 1][MID_IDX    ][MID_IDX    ] 
	-       2.0f*fh_vals[MID_IDX    ][MID_IDX    ][MID_IDX    ]
	+            fh_vals[MID_IDX - 1][MID_IDX    ][MID_IDX    ];

      //dxy
      H[0][1]= 0.25f*(fh_vals[MID_IDX    ][MID_IDX + 1][MID_IDX + 1] -
		      fh_vals[MID_IDX    ][MID_IDX - 1][MID_IDX + 1] -
		      fh_vals[MID_IDX    ][MID_IDX + 1][MID_IDX - 1] + 
		      fh_vals[MID_IDX    ][MID_IDX - 1][MID_IDX - 1]);
      //dys
      H[0][2]= 0.25f*(fh_vals[MID_IDX + 1][MID_IDX + 1][MID_IDX    ] -
		      fh_vals[MID_IDX + 1][MID_IDX - 1][MID_IDX    ] -
		      fh_vals[MID_IDX - 1][MID_IDX + 1][MID_IDX    ] + 
		      fh_vals[MID_IDX - 1][MID_IDX - 1][MID_IDX    ]);

      //dxs
      H[1][2]= 0.25f*(fh_vals[MID_IDX + 1][MID_IDX    ][MID_IDX + 1] -
		      fh_vals[MID_IDX + 1][MID_IDX    ][MID_IDX - 1] -
		      fh_vals[MID_IDX - 1][MID_IDX    ][MID_IDX + 1] + 
		      fh_vals[MID_IDX - 1][MID_IDX    ][MID_IDX - 1]);

      //dyx = dxy
      H[1][0] = H[0][1];

      //dsy = dys
      H[2][0] = H[0][2];

      //dsx = dxs
      H[2][1] = H[1][2];


      __shared__ float dD[3];

      //dy
      dD[0] = 0.5f*(fh_vals[MID_IDX    ][MID_IDX + 1][MID_IDX    ] -
	 	    fh_vals[MID_IDX    ][MID_IDX - 1][MID_IDX    ]);
      //dx
      dD[1] = 0.5f*(fh_vals[MID_IDX    ][MID_IDX    ][MID_IDX + 1] -
		    fh_vals[MID_IDX    ][MID_IDX    ][MID_IDX - 1]);
      //ds
      dD[2] = 0.5f*(fh_vals[MID_IDX + 1][MID_IDX    ][MID_IDX    ] -
		    fh_vals[MID_IDX - 1][MID_IDX    ][MID_IDX    ]);

      __shared__ float invdet;
      invdet = 1.f /
	(
	 H[0][0]*H[1][1]*H[2][2] 
	 +   H[0][1]*H[1][2]*H[2][0]
	 +   H[0][2]*H[1][0]*H[2][1]
	 -   H[0][0]*H[1][2]*H[2][1]
	 -   H[0][1]*H[1][0]*H[2][2]
	 -   H[0][2]*H[1][1]*H[2][0]
	 );


      //   // 1-based entries of a 3x3 inverse
      //   /*             [ |a22 a23|   |a12 a13|  |a12 a13|]     */
      //   /*             [ |a32 a33|  -|a32 a33|  |a22 a23|]     */
      //   /*             [                                 ]     */
      //   /*             [ |a21 a23|   |a11 a13|  |a11 a13|]     */
      //   /*    A^(-1) = [-|a31 a33|   |a31 a33| -|a21 a23|] / d */
      //   /*             [                                 ]     */
      //   /*             [ |a21 a22|   |a11 a12|  |a11 a12|]     */
      //   /*             [ |a31 a32|  -|a31 a32|  |a21 a22|]     */

      __shared__ float Hinv[3][3];
      Hinv[0][0] =  invdet*(H[1][1]*H[2][2]-H[1][2]*H[2][1]);
      Hinv[0][1] =  invdet*(H[0][2]*H[2][1]-H[0][1]*H[2][2]);
      Hinv[0][2] =  invdet*(H[0][1]*H[1][2]-H[0][2]*H[1][1]);

      Hinv[1][0] =  invdet*(H[1][2]*H[2][0]-H[1][0]*H[2][2]);
      Hinv[1][1] =  invdet*(H[0][0]*H[2][2]-H[0][2]*H[2][0]);
      Hinv[1][2] =  invdet*(H[0][2]*H[1][0]-H[0][0]*H[1][2]);

      Hinv[2][0] =  invdet*(H[1][0]*H[2][1]-H[1][1]*H[2][0]);
      Hinv[2][1] =  invdet*(H[0][1]*H[2][0]-H[0][0]*H[2][1]);
      Hinv[2][2] =  invdet*(H[0][0]*H[1][1]-H[0][1]*H[1][0]);

      __shared__ float x[3];

      x[0] = -(Hinv[0][0]*(dD[0]) + Hinv[0][1]*(dD[1]) + Hinv[0][2]*(dD[2]));
      x[1] = -(Hinv[1][0]*(dD[0]) + Hinv[1][1]*(dD[1]) + Hinv[1][2]*(dD[2]));
      x[2] = -(Hinv[2][0]*(dD[0]) + Hinv[2][1]*(dD[1]) + Hinv[2][2]*(dD[2]));

      if(fabs(x[0])< 1.5f && fabs(x[1])< 1.5f && fabs(x[2])< 1.5f) { // if the step is near the interpolation region, perform it
	
	// Get a new feature index.
	i = atomicInc(d_feature_counter,(unsigned int) -1);

 	if(i < ASRL_SURF_MAX_FEATURES) {
	  
 	  //d_features[MAX_N_FEATURES*SF_X + i] =      ((float)d_maxmin[blockIdx.x].x+x[1]) *  (float)d_octave_params[octave].step + d_octave_params[octave].border;
	  p.x =  ((float)d_maxmin[blockIdx.x].x+x[1]) *  (float)d_octave_params[octave].step + d_octave_params[octave].border;
 	  //d_features[MAX_N_FEATURES*SF_Y + i] =      ((float)d_maxmin[blockIdx.x].y+x[0]) *  (float)d_octave_params[octave].step + d_octave_params[octave].border;
	  p.y =  ((float)d_maxmin[blockIdx.x].y+x[0]) *  (float)d_octave_params[octave].step + d_octave_params[octave].border;

 	  if(x[2] > 0)
 	    {
 	      float a = d_hessian_scale[ASRL_SURF_MAX_INTERVALS*octave + d_maxmin[blockIdx.x].z];
 	      float b = d_hessian_scale[ASRL_SURF_MAX_INTERVALS*octave + d_maxmin[blockIdx.x].z + 1];
 	      //d_features[MAX_N_FEATURES*SF_SCALE+i] = (1.f - x[2]) * a + x[2] * b;
	      p.size =  0.8f * ((1.f - x[2]) * a + x[2] * b); // 2 * 1.2f/3.f = 0.8f (scaling to match SURF's range)
 	    } 
 	  else
 	    {
 	      float a = d_hessian_scale[ASRL_SURF_MAX_INTERVALS*octave + d_maxmin[blockIdx.x].z];
 	      float b = d_hessian_scale[ASRL_SURF_MAX_INTERVALS*octave + d_maxmin[blockIdx.x].z - 1];
	      //d_features[MAX_N_FEATURES*SF_SCALE+i] = (1.f + x[2]) * a - x[2] * b;
	      p.size = 0.8f * ((1.f + x[2]) * a - x[2] * b);
 	    }

	  //((char*)&d_features[MAX_N_FEATURES*SF_OCTAVE])[i] = octave;
	  p.octave = octave;
			
	  //d_features[MAX_N_FEATURES*SF_STRENGTH + i] =  fh_vals[MID_IDX][MID_IDX][MID_IDX];
	  bool laplacian = isLastBitSet(fh_vals[MID_IDX][MID_IDX][MID_IDX]);
	  p.response = fh_vals[MID_IDX][MID_IDX][MID_IDX] + 0.5f * (x[0]*dD[0] + x[1]*dD[1] + x[2]*dD[2]);
	  if(laplacian)
	    setLastBit(p.response);
	  else
	    clearLastBit(p.response);

	  // Save the covariance of p
	  p.sigma_xx = -(float)d_octave_params[octave].step * (float)d_octave_params[octave].step * Hinv[1][1];
	  p.sigma_yy = -(float)d_octave_params[octave].step * (float)d_octave_params[octave].step * Hinv[0][0];
	  p.sigma_xy = -(float)d_octave_params[octave].step * (float)d_octave_params[octave].step * Hinv[1][0];

	  // Should we split up this transfer over many threads?
	  d_features[i] = p;
	} else {
	  *d_feature_counter = ASRL_SURF_MAX_FEATURES;
	} // If there is still room in memory for the feature
      } // If the subpixel interpolation worked
    } // If this is thread 0.

    //     __syncthreads();
    //     // Transfer the feature back.
    //     if(i < MAX_N_FEATURES)
    //       {
    // 	int tid = threadIdx.x + __mul24(threadIdx.y,3) + __mul24(threadIdx.z,9);
    // 	if(tid < 6)
    // 	  {
    // 	    ((float*)&d_features[i])[tid] = ((float*)&p)[tid];
    // 	  }
    //       }
  }


  void run_fh_interp_extremum(float * d_hessian, Keypoint * d_features, int4 * d_maxmin, unsigned int * d_feature_counter, unsigned int * d_max_min_counter, float * d_threshold)
  {
    dim3 threads;
    threads.x = 3;
    threads.y = 3;
    threads.z = 3;
    
    dim3 grid;
    grid.x = ASRL_SURF_MAX_CANDIDATES;
    grid.y = 1; 
    grid.z = 1;
    
    fh_interp_extremum<<< grid, threads >>>(d_hessian,
					    d_features, 
					    d_maxmin, 
					    d_feature_counter,
					    d_max_min_counter, 
					    d_threshold);
  }

} // namespace asrl
