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

#include "orientation.h"
//#include <float.h> // FLT_MAX
#define FLT_MAX  3.4028235E38f
#include "gpu_globals.h"
#include "gpu_utils.h"


namespace asrl {
  // precomputed values for a Gaussian with a standard deviation of 2
  __constant__ float dc_gauss1D[13] = {0.002215924206f, 0.008764150247f, 0.026995483257f, 0.064758797833f, 0.120985362260f, 0.176032663382f, 0.199471140201f, 0.176032663382f, 0.120985362260f, 0.064758797833f, 0.026995483257f, 0.008764150247f, 0.002215924206f};

  __device__ inline void swap(float3 & a, float3 & b)
  {
    float3 tmp = a;
    a = b;
    b = tmp;
  }

  __device__ void setMaxXZ(float3 & dest, float3 & comp)
  {
    if(dest.x < comp.x) {
      dest.x = comp.x;
      dest.z = comp.z;
    }
  }

  __device__ void setSumXY(float2 & dest, float2 & src)
  {
    dest.x += src.x;
    dest.y += src.y;
  }

  __device__ void setMaxZ3(float3 & dest, float3 & comp)
  {
    if(dest.z < comp.z) {
      dest.x = comp.x;
      dest.y = comp.y;
      dest.z = comp.z;
    }
  }

  __global__ void find_orientation_fast_kernel(Keypoint * d_features)
  {
    int tid = __mul24(threadIdx.y, 17) + threadIdx.x;
    int tid2 = INT_MAX;
    if(threadIdx.x < 13 && threadIdx.y < 13) {
      tid2 = __mul24(threadIdx.y,13) + threadIdx.x;
    }


    __shared__ float texLookups[17][17];
    //__shared__ float2 dxDy[13*13];
    
    __shared__ float Edx[13*13];
    __shared__ float Edy[13*13];
    __shared__ float xys[3];
    // Read my x, y, scale.
    if(tid < 3)
      {
	xys[tid] = ((float*)(&d_features[blockIdx.x]))[tid];
      }

    __syncthreads();

    __shared__ float inv_scale;
    if(tid == 0)
      inv_scale = 1.f/(16.f * xys[2] * xys[2]);
    
    
    // Read all texture locations into memory
    // Maybe I should use __mul24 here?
    texLookups[threadIdx.x][threadIdx.y] = 
      tex2D(d_integralTex, xys[0] + ((int)threadIdx.x - 8)*xys[2], 
	    xys[1] + ((int)threadIdx.y - 8)*xys[2]);

    __syncthreads();

    float dx = 0.f;
    float dy = 0.f;
	 
    // Computes lookups for all points in a 13x13 lattice.
    // - SURF says to only use a circle, but the branching logic would slow it down
    // - Gaussian weighting should reduce the effects of the outer points anyway
    if(tid2 < 169)
      {

	dx -=     texLookups[threadIdx.x    ][threadIdx.y    ];
	dx += 2.f*texLookups[threadIdx.x + 2][threadIdx.y    ];
	dx -=     texLookups[threadIdx.x + 4][threadIdx.y    ];
	dx +=     texLookups[threadIdx.x    ][threadIdx.y + 4];
	dx -= 2.f*texLookups[threadIdx.x + 2][threadIdx.y + 4];
	dx +=     texLookups[threadIdx.x + 4][threadIdx.y + 4];
	dx *= inv_scale;

	dy -=     texLookups[threadIdx.x    ][threadIdx.y    ];
	dy += 2.f*texLookups[threadIdx.x    ][threadIdx.y + 2];
	dy -=     texLookups[threadIdx.x    ][threadIdx.y + 4];
	dy +=     texLookups[threadIdx.x + 4][threadIdx.y    ];
	dy -= 2.f*texLookups[threadIdx.x + 4][threadIdx.y + 2];
	dy +=     texLookups[threadIdx.x + 4][threadIdx.y + 4];
	dy *= inv_scale;

	float g = dc_gauss1D[threadIdx.x] * dc_gauss1D[threadIdx.y];

	Edx[tid2] = dx * g;
	Edy[tid2] = dy * g;

      }

    __syncthreads();

    // This is a scan to get the summed dx, dy values.
    // Gets 128-168
    if (tid < 41) {Edx[tid] += Edx[tid+128]; } __syncthreads(); 
    if (tid < 64) { Edx[tid] += Edx[tid + 64]; } __syncthreads(); 
    if (tid < 32) {
      Edx[tid]+=Edx[tid + 32];
      Edx[tid]+=Edx[tid + 16];
      Edx[tid]+=Edx[tid + 8];
      Edx[tid]+=Edx[tid + 4];
      Edx[tid]+=Edx[tid + 2];
      Edx[tid]+=Edx[tid + 1];
    }

    // Gets 128-168
    if (tid < 41) {Edy[tid] += Edy[tid+128]; } __syncthreads(); 
    if (tid < 64) {Edy[tid] += Edy[tid + 64]; } __syncthreads(); 
    if (tid < 32) {
      Edy[tid]+=Edy[tid + 32];
      Edy[tid]+=Edy[tid + 16];
      Edy[tid]+=Edy[tid + 8];
      Edy[tid]+=Edy[tid + 4];
      Edy[tid]+=Edy[tid + 2];
      Edy[tid]+=Edy[tid + 1];
    }

 
    // Thread 0 saves back the result.
    if (tid == 0)
      {
	d_features[blockIdx.x].angle = -atan2(Edy[0],Edx[0]);
	d_features[blockIdx.x].angle_response = Edy[0]*Edy[0] + Edx[0]*Edx[0];
      }

  }

  __global__ void find_orientation_kernel(Keypoint * d_features)
  {
    int tid = __mul24(threadIdx.y, 17) + threadIdx.x;
    int tid2 = INT_MAX;
    if(threadIdx.x < 13 && threadIdx.y < 13) {
      tid2 = __mul24(threadIdx.y,13) + threadIdx.x;
    }

    //int featureBase = __mul24( blockIdx.x , SF_FEATURE_STRIDE);


    __shared__ float texLookups[17][17];
    __shared__ float3 dxDyAngle[17*17];
    __shared__ float xys[3];
    // Read my x, y, scale.
    if(tid < 3)
      {
	//xys[tid] = d_features[featureBase + tid ];
	xys[tid] = ((float*)(&d_features[blockIdx.x]))[tid];
      }

    __syncthreads();

    // Read all texture locations into memory
    // Maybe I should use __mul24 here?
    texLookups[threadIdx.x][threadIdx.y] = 
      tex2D(d_integralTex, xys[0] + ((int)threadIdx.x - 8)*xys[2], 
	    xys[1] + ((int)threadIdx.y - 8)*xys[2]);

    __syncthreads();

    // DEBUG
    // The top left texture lookup for each thread.
    //d_debug[tid] = texLookups[threadIdx.x][threadIdx.y];


    float dx = 0.f;
    float dy = 0.f;
    float angle = 0.f;
    dxDyAngle[tid].z = FLT_MAX;
	 
    // Computes lookups for all points in a 13x13 lattice.
    // - SURF says to only use a circle, but the branching logic would slow it down
    // - Gaussian weighting should reduce the effects of the outer points anyway
    if(tid2 < 169)
      {
	dx -=     texLookups[threadIdx.x    ][threadIdx.y    ];
	dx += 2.f*texLookups[threadIdx.x + 2][threadIdx.y    ];
	dx -=     texLookups[threadIdx.x + 4][threadIdx.y    ];
	dx +=     texLookups[threadIdx.x    ][threadIdx.y + 4];
	dx -= 2.f*texLookups[threadIdx.x + 2][threadIdx.y + 4];
	dx +=     texLookups[threadIdx.x + 4][threadIdx.y + 4];

	dy -=     texLookups[threadIdx.x    ][threadIdx.y    ];
	dy += 2.f*texLookups[threadIdx.x    ][threadIdx.y + 2];
	dy -=     texLookups[threadIdx.x    ][threadIdx.y + 4];
	dy +=     texLookups[threadIdx.x + 4][threadIdx.y    ];
	dy -= 2.f*texLookups[threadIdx.x + 4][threadIdx.y + 2];
	dy +=     texLookups[threadIdx.x + 4][threadIdx.y + 4];

	//float g = d_gauss_table[tid2];
	float g = dc_gauss1D[threadIdx.x] * dc_gauss1D[threadIdx.y];

	dxDyAngle[tid2].x = dx * g;
	dxDyAngle[tid2].y = dy * g;
	angle = atan2(dy,dx);
	dxDyAngle[tid2].z = angle;

      }

    __syncthreads();

#if 0

    // Threads should be synced...
    // Now the array is sorted, we can go through and calculate the sliding window.
    // This takes about 10ms on my laptop
    if(tid < 169) {
      angle = dxDyAngle[tid].z;
      dx = dxDyAngle[tid].x;
      dy = dxDyAngle[tid].y;

      int idx = tid;
      //float wrapped = 0.f;
      for(int i = 1; i < 169; i++) {
	idx++;
	if(idx == 169){
	  idx = 0;
	}
	float a2 = dxDyAngle[idx].z;
	if(a2 < angle)
	  {
	    a2 += 6.2831853071796f;
	  }

	if( a2 < angle +  1.047197551197f) // pi / 3
	  {
	    dx += dxDyAngle[idx].x;
	    dy += dxDyAngle[idx].y;
	  }
      }

      angle = atan2f(dy,dx);
      dx = dx*dx + dy*dy;
      //d_debug[tid] = idx;
    }
    __syncthreads();

    if(tid < 169) {
      dxDyAngle[tid].x = dx;
      dxDyAngle[tid].z = angle;
    } else {
      dxDyAngle[tid].x = -FLT_MAX;
    }
#else
    // Now do we sort or just scan the whole array?  
    // sort...there is no clean way to compare angles that are not sorted.
    // Parallel bitonic sort.
    // This only seems to work for powers of two so we'll pad to 256. 
    // This sort works. I hope it is fast! It takes about 10 ms on my laptop

    for (unsigned int k = 2; k <= 256; k *= 2)
      {
	// Bitonic merge:
	for (unsigned int j = k / 2; j>0; j /= 2)
	  {
	    unsigned int ixj = tid ^ j;

	    if(tid < 256 && ixj > tid)
	      {
		if ((tid & k) == 0)
		  {
		    if (dxDyAngle[tid].z > dxDyAngle[ixj].z)
		      {
			swap(dxDyAngle[tid], dxDyAngle[ixj]);
		      }
		  }
		else
		  {
		    if (dxDyAngle[tid].z < dxDyAngle[ixj].z)
		      {
			swap(dxDyAngle[tid], dxDyAngle[ixj]);
		      }
		  }
	      }

	    __syncthreads();
	  }
      }

    // Threads should be synced...
    // Now the array is sorted, we can go through and calculate the sliding window.
    // This takes about 10ms on my laptop
    if(tid < 169) {
      angle = dxDyAngle[tid].z;
      dx = dxDyAngle[tid].x;
      dy = dxDyAngle[tid].y;

      int idx = tid;
      float wrapped = 0.f;
      for(int i = 1; i < 169; i++) {
	idx++;
	if(idx == 169){
	  idx = 0;
	  wrapped = 6.2831853071796f; // 2* Pi
	}

	if( (dxDyAngle[idx].z + wrapped) > angle +  1.047197551197f){ // pi / 3
	  break;
	} else {
	  dx += dxDyAngle[idx].x;
	  dy += dxDyAngle[idx].y;
	}
      }

      angle = atan2f(dy,dx);
      dx = dx*dx + dy*dy;
    }
    __syncthreads();

    if(tid < 169) {
      dxDyAngle[tid].x = dx;
      dxDyAngle[tid].z = angle;
    } else {
      dxDyAngle[tid].x = -FLT_MAX;
    }
#endif

    __syncthreads();

    // Now do a reduction to find the maximum orientation
    // This takes about 1ms	
    if (tid < 128) { setMaxXZ(dxDyAngle[tid],dxDyAngle[tid+128]); } __syncthreads(); 

    if (tid < 64) { setMaxXZ(dxDyAngle[tid], dxDyAngle[tid + 64]); } __syncthreads(); 

    if (tid < 32) {
      setMaxXZ(dxDyAngle[tid],dxDyAngle[tid + 32]);
      setMaxXZ(dxDyAngle[tid],dxDyAngle[tid + 16]);
      setMaxXZ(dxDyAngle[tid],dxDyAngle[tid + 8]);
      setMaxXZ(dxDyAngle[tid],dxDyAngle[tid + 4]);
      setMaxXZ(dxDyAngle[tid],dxDyAngle[tid + 2]);
      setMaxXZ(dxDyAngle[tid],dxDyAngle[tid + 1]);
    }

    // Thread 0 saves back the result.
    if (tid == 0)
      {
	//d_features[featureBase + SF_ORIENTATION ] = -dxDyAngle[0].z;
	d_features[blockIdx.x].angle = -dxDyAngle[0].z;
      }

  }



  void find_orientation( Keypoint * d_features, int nFeatures) 
  {

    dim3 threads;
    threads.x = 17;
    threads.y = 17;

    dim3 grid;
    grid.x = nFeatures;
    grid.y = 1;
    grid.z = 1;

    find_orientation_kernel <<< grid, threads >>>(d_features);

  }

  void find_orientation_fast( Keypoint * d_features, int nFeatures) 
  {

    dim3 threads;
    threads.x = 17;
    threads.y = 17;

    dim3 grid;
    grid.x = nFeatures;
    grid.y = 1;
    grid.z = 1;

    find_orientation_fast_kernel <<< grid, threads >>>(d_features);
  }


} // namespace asrl
