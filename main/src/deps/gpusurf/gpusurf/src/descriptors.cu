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

#include "descriptors.h"
#include "gpu_globals.h"
#include "gpu_utils.h"
#include <stdio.h>


namespace asrl {
  // precomputed values for a Gaussian with a standard deviation of 3.3
  // - it appears SURF uses a different value, but not sure what it is
  __constant__ float dc_3p3gauss1D[20] = {0.001917811039f, 0.004382549939f, 0.009136246641f, 0.017375153068f, 0.030144587513f,
					  0.047710056854f, 0.068885910797f, 0.090734146446f, 0.109026229640f, 0.119511889092f,
					  0.119511889092f, 0.109026229640f, 0.090734146446f, 0.068885910797f, 0.047710056854f,
					  0.030144587513f, 0.017375153068f, 0.009136246641f, 0.004382549939f, 0.001917811039f};
					  
					  
					  
  // Computes unnormalized 64 dimensional descriptor, puts it into d_descriptors in the correct location
  __global__ void compute_descriptors_kernel(float * d_descriptors, Keypoint * d_features)
  {
    // compute thread IDs (row-major)
    int tid = __mul24(threadIdx.y,blockDim.x) + threadIdx.x;

    // allocate shared memory
    __shared__ float smem[2*5*5];		// 2 floats (dx,dy) for each thread (5x5 sample points in each sub-region)

    // get the interest point parameters (x, y, scale, strength, theta)
    __shared__ float ipt[5];
    if (tid < 5)
      {
	//ipt[tid] = d_features[__mul24(blockIdx.x, SF_FEATURE_STRIDE) + tid];
	ipt[tid] = ((float*)&d_features[blockIdx.x])[tid];
      }
    __syncthreads();


    // compute sin(theta), cos(theta)
    // (there are faster, but less accurate trig functions: __sinf, __cosf)
    __shared__ float sin_theta;
    __shared__ float cos_theta;
    if (tid == 0)
      {
	sin_theta = sinf(ipt[SF_ANGLE]);
      }
    else if (tid == 24)	// another number in a different half-warp (to ensure no repeated branching logic)
      {
	cos_theta = cosf(ipt[SF_ANGLE]);
      }
    __syncthreads();


    // Compute sampling points
    // since grids are 2D, need to compute xBlock and yBlock indices
    int xBlock = (blockIdx.y & 3);	// blockIdx.y % 4
    int yBlock = (blockIdx.y >> 2);	// floor(blockIdx.y/4)
    int xIndex = __mul24(xBlock, blockDim.x) + threadIdx.x;
    int yIndex = __mul24(yBlock, blockDim.y) + threadIdx.y;

    // Compute rotated sampling points
    // (clockwise rotation since we are rotating the lattice)
    // (subtract 9.5f to start sampling at the top left of the lattice, 0.5f is to space points out properly - there is no center pixel)
    float sample_x = ipt[SF_X] + (  cos_theta * ((float) (xIndex-9.5f)) * ipt[SF_SIZE]
				    + sin_theta * ((float) (yIndex-9.5f)) * ipt[SF_SIZE]);
    float sample_y = ipt[SF_Y] + ( -sin_theta * ((float) (xIndex-9.5f)) * ipt[SF_SIZE]
				   + cos_theta * ((float) (yIndex-9.5f)) * ipt[SF_SIZE]);

    // gather integral image lookups for Haar wavelets at each point (some lookups are shared between dx and dy)
    //	a b c
    //	d   f
    //	g h i
    float a = tex2D(d_integralTex, sample_x - ipt[SF_SIZE], sample_y - ipt[SF_SIZE]);
    float b = tex2D(d_integralTex, sample_x, 					 sample_y - ipt[SF_SIZE]);
    float c = tex2D(d_integralTex, sample_x + ipt[SF_SIZE], sample_y - ipt[SF_SIZE]);
    float d = tex2D(d_integralTex, sample_x - ipt[SF_SIZE], sample_y);
    float f = tex2D(d_integralTex, sample_x + ipt[SF_SIZE], sample_y);
    float g = tex2D(d_integralTex, sample_x - ipt[SF_SIZE], sample_y + ipt[SF_SIZE]);
    float h = tex2D(d_integralTex, sample_x, 					 sample_y + ipt[SF_SIZE]);
    float i = tex2D(d_integralTex, sample_x + ipt[SF_SIZE], sample_y + ipt[SF_SIZE]);	

    // compute axis-aligned HaarX, HaarY
    // (could group the additions together into multiplications)
    float gauss = dc_3p3gauss1D[xIndex] * dc_3p3gauss1D[yIndex];	// separable because independent (circular)      
    float aa_dx = gauss * (-(a-b-g+h) + (b-c-h+i));		// unrotated dx
    float aa_dy = gauss * (-(a-c-d+f) + (d-f-g+i));		// unrotated dy

    // rotate responses (store all dxs then all dys)
    // - counterclockwise rotation to rotate back to zero orientation
    smem[tid] =	 aa_dx*cos_theta - aa_dy*sin_theta;		// rotated dx
    smem[25+tid] = aa_dx*sin_theta + aa_dy*cos_theta;		// rotated dy
    __syncthreads();


    // sum (reduce) 5x5 area response
    __shared__ float rmem[5*5];		// buffer for conducting reductions

    // copy all of the dx responses to a |dx| array
    rmem[tid] = fabs(smem[tid]);	// |dx| array
    __syncthreads();


    // sum (reduce) dx and |dx|
    // first step is to reduce from 25 to 16
    if (tid < 9)	// use 9 threads
      {
	smem[tid] = smem[tid] + smem[tid + 16];
	rmem[tid] = rmem[tid] + rmem[tid + 16];
      }
    __syncthreads();


    // sum (reduce) from 16 to 1 (unrolled - aligned to a half-warp)
    if (tid < 16)
      {
	smem[tid] = smem[tid] + smem[tid + 8];
	smem[tid] = smem[tid] + smem[tid + 4];
	smem[tid] = smem[tid] + smem[tid + 2];
	smem[tid] = smem[tid] + smem[tid + 1];

	rmem[tid] = rmem[tid] + rmem[tid + 8];
	rmem[tid] = rmem[tid] + rmem[tid + 4];
	rmem[tid] = rmem[tid] + rmem[tid + 2];
	rmem[tid] = rmem[tid] + rmem[tid + 1];
      }
    __syncthreads();


    // write dx and |dx| result out (order matches SURF)
    if (tid == 0)
      {
	int block_start = __mul24(blockIdx.x,ASRL_SURF_DESCRIPTOR_DIM) + __mul24(blockIdx.y,4);
	d_descriptors[block_start] = smem[0];
	d_descriptors[block_start+1] = rmem[0];
      }
    __syncthreads();


    // index shift for the dy values
    int dy_index = tid + 25;

    // copy all of the dy responses to a |dy| array
    rmem[tid] = fabs(smem[dy_index]);	// |dy| array
    __syncthreads();


    // sum (reduce) dy and |dy|
    // first step is to reduce from 25 to 16
    if (tid < 9)	// use 9 threads
      {
	smem[dy_index] = smem[dy_index] + smem[dy_index + 16];
	rmem[tid] = rmem[tid] + rmem[tid + 16];
      }
    __syncthreads();


    // sum (reduce) from 16 to 1 (unrolled - aligned to a half-warp)
    if (tid < 16)
      {
	smem[dy_index] = smem[dy_index] + smem[dy_index + 8];
	smem[dy_index] = smem[dy_index] + smem[dy_index + 4];
	smem[dy_index] = smem[dy_index] + smem[dy_index + 2];
	smem[dy_index] = smem[dy_index] + smem[dy_index + 1];

	rmem[tid] = rmem[tid] + rmem[tid + 8];
	rmem[tid] = rmem[tid] + rmem[tid + 4];
	rmem[tid] = rmem[tid] + rmem[tid + 2];
	rmem[tid] = rmem[tid] + rmem[tid + 1];
      }
    __syncthreads();


    // write dy and |dy| result out (order matches SURF)
    if (tid == 0)
      {
	int block_start = __mul24(blockIdx.x, ASRL_SURF_DESCRIPTOR_DIM) + __mul24(blockIdx.y,4);
	d_descriptors[block_start+2] = smem[25];
	d_descriptors[block_start+3] = rmem[0];
      }
    // at this time, d_descriptors is composed of unnormalized values of: (dx, dy, |dx|, |dy|)
  }
  
  
  
  // Computes unnormalized 64 dimensional descriptor without Gaussian weighting, puts it into d_descriptors in the correct location
  __global__ void compute_descriptors_unweighted_kernel(float * d_descriptors, Keypoint * d_features)
  {
    // compute thread IDs (row-major)
    int tid = __mul24(threadIdx.y,blockDim.x) + threadIdx.x;

    // allocate shared memory
    __shared__ float smem[2*5*5];		// 2 floats (dx,dy) for each thread (5x5 sample points in each sub-region)

    // get the interest point parameters (x, y, scale, strength, theta)
    __shared__ float ipt[5];
    if (tid < 5)
      {
	//ipt[tid] = d_features[__mul24(blockIdx.x, SF_FEATURE_STRIDE) + tid];
	ipt[tid] = ((float*)&d_features[blockIdx.x])[tid];
      }
    __syncthreads();


    // compute sin(theta), cos(theta)
    // (there are faster, but less accurate trig functions: __sinf, __cosf)
    __shared__ float sin_theta;
    __shared__ float cos_theta;
    if (tid == 0)
      {
	sin_theta = sinf(ipt[SF_ANGLE]);
      }
    else if (tid == 24)	// another number in a different half-warp (to ensure no repeated branching logic)
      {
	cos_theta = cosf(ipt[SF_ANGLE]);
      }
    __syncthreads();


    // Compute sampling points
    // since grids are 2D, need to compute xBlock and yBlock indices
    int xBlock = (blockIdx.y & 3);	// blockIdx.y % 4
    int yBlock = (blockIdx.y >> 2);	// floor(blockIdx.y/4)
    int xIndex = __mul24(xBlock, blockDim.x) + threadIdx.x;
    int yIndex = __mul24(yBlock, blockDim.y) + threadIdx.y;

    // Compute rotated sampling points
    // (clockwise rotation since we are rotating the lattice)
    // (subtract 9.5f to start sampling at the top left of the lattice, 0.5f is to space points out properly - there is no center pixel)
    float sample_x = ipt[SF_X] + (  cos_theta * ((float) (xIndex-9.5f)) * ipt[SF_SIZE]
				    + sin_theta * ((float) (yIndex-9.5f)) * ipt[SF_SIZE]);
    float sample_y = ipt[SF_Y] + ( -sin_theta * ((float) (xIndex-9.5f)) * ipt[SF_SIZE]
				   + cos_theta * ((float) (yIndex-9.5f)) * ipt[SF_SIZE]);

    // gather integral image lookups for Haar wavelets at each point (some lookups are shared between dx and dy)
    //	a b c
    //	d	 f
    //	g h i
    float a = tex2D(d_integralTex, sample_x - ipt[SF_SIZE], sample_y - ipt[SF_SIZE]);
    float b = tex2D(d_integralTex, sample_x, 					 sample_y - ipt[SF_SIZE]);
    float c = tex2D(d_integralTex, sample_x + ipt[SF_SIZE], sample_y - ipt[SF_SIZE]);
    float d = tex2D(d_integralTex, sample_x - ipt[SF_SIZE], sample_y);
    float f = tex2D(d_integralTex, sample_x + ipt[SF_SIZE], sample_y);
    float g = tex2D(d_integralTex, sample_x - ipt[SF_SIZE], sample_y + ipt[SF_SIZE]);
    float h = tex2D(d_integralTex, sample_x, 					 sample_y + ipt[SF_SIZE]);
    float i = tex2D(d_integralTex, sample_x + ipt[SF_SIZE], sample_y + ipt[SF_SIZE]);	

    // compute axis-aligned HaarX, HaarY
    // (could group the additions together into multiplications)   
    float aa_dx = (-(a-b-g+h) + (b-c-h+i));		// unrotated dx
    float aa_dy = (-(a-c-d+f) + (d-f-g+i));		// unrotated dy

    // rotate responses (store all dxs then all dys)
    // - counterclockwise rotation to rotate back to zero orientation
    smem[tid] =	 aa_dx*cos_theta - aa_dy*sin_theta;		// rotated dx
    smem[25+tid] = aa_dx*sin_theta + aa_dy*cos_theta;		// rotated dy
    __syncthreads();


    // sum (reduce) 5x5 area response
    __shared__ float rmem[5*5];		// buffer for conducting reductions

    // copy all of the dx responses to a |dx| array
    rmem[tid] = fabs(smem[tid]);	// |dx| array
    __syncthreads();


    // sum (reduce) dx and |dx|
    // first step is to reduce from 25 to 16
    if (tid < 9)	// use 9 threads
      {
	smem[tid] = smem[tid] + smem[tid + 16];
	rmem[tid] = rmem[tid] + rmem[tid + 16];
      }
    __syncthreads();


    // sum (reduce) from 16 to 1 (unrolled - aligned to a half-warp)
    if (tid < 16)
      {
	smem[tid] = smem[tid] + smem[tid + 8];
	smem[tid] = smem[tid] + smem[tid + 4];
	smem[tid] = smem[tid] + smem[tid + 2];
	smem[tid] = smem[tid] + smem[tid + 1];

	rmem[tid] = rmem[tid] + rmem[tid + 8];
	rmem[tid] = rmem[tid] + rmem[tid + 4];
	rmem[tid] = rmem[tid] + rmem[tid + 2];
	rmem[tid] = rmem[tid] + rmem[tid + 1];
      }
    __syncthreads();


    // write dx and |dx| result out (order matches SURF)
    if (tid == 0)
      {
	int block_start = __mul24(blockIdx.x,ASRL_SURF_DESCRIPTOR_DIM) + __mul24(blockIdx.y,4);
	d_descriptors[block_start] = smem[0];
	d_descriptors[block_start+1] = rmem[0];
      }
    __syncthreads();


    // index shift for the dy values
    int dy_index = tid + 25;

    // copy all of the dy responses to a |dy| array
    rmem[tid] = fabs(smem[dy_index]);	// |dy| array
    __syncthreads();


    // sum (reduce) dy and |dy|
    // first step is to reduce from 25 to 16
    if (tid < 9)	// use 9 threads
      {
	smem[dy_index] = smem[dy_index] + smem[dy_index + 16];
	rmem[tid] = rmem[tid] + rmem[tid + 16];
      }
    __syncthreads();


    // sum (reduce) from 16 to 1 (unrolled - aligned to a half-warp)
    if (tid < 16)
      {
	smem[dy_index] = smem[dy_index] + smem[dy_index + 8];
	smem[dy_index] = smem[dy_index] + smem[dy_index + 4];
	smem[dy_index] = smem[dy_index] + smem[dy_index + 2];
	smem[dy_index] = smem[dy_index] + smem[dy_index + 1];

	rmem[tid] = rmem[tid] + rmem[tid + 8];
	rmem[tid] = rmem[tid] + rmem[tid + 4];
	rmem[tid] = rmem[tid] + rmem[tid + 2];
	rmem[tid] = rmem[tid] + rmem[tid + 1];
      }
    __syncthreads();


    // write dy and |dy| result out (order matches SURF)
    if (tid == 0)
      {
	int block_start = __mul24(blockIdx.x, ASRL_SURF_DESCRIPTOR_DIM) + __mul24(blockIdx.y,4);
	d_descriptors[block_start+2] = smem[25];
	d_descriptors[block_start+3] = rmem[0];
      }
    // at this time, d_descriptors is composed of unnormalized values of: (dx, dy, |dx|, |dy|)
  }



  __global__ void normalize_descriptors_kernel(float * d_descriptors)
  {
    // no need for thread ID
    int descriptor_base = __mul24(blockIdx.x, ASRL_SURF_DESCRIPTOR_DIM);


    // read in the unnormalized descriptor values (squared)
    __shared__ float sqDesc[ASRL_SURF_DESCRIPTOR_DIM];
    float lookup = d_descriptors[descriptor_base + threadIdx.x];

    sqDesc[threadIdx.x] = lookup * lookup;
    __syncthreads();

    // This is broken in compute capability 2.1 for whatever reason.
    // // reduction to get total
    // if (threadIdx.x < 32)
    //   {
    // 	sqDesc[threadIdx.x] = sqDesc[threadIdx.x] + sqDesc[threadIdx.x + 32];
    // 	sqDesc[threadIdx.x] = sqDesc[threadIdx.x] + sqDesc[threadIdx.x + 16];
    // 	sqDesc[threadIdx.x] = sqDesc[threadIdx.x] + sqDesc[threadIdx.x + 8];
    // 	sqDesc[threadIdx.x] = sqDesc[threadIdx.x] + sqDesc[threadIdx.x + 4];
    // 	sqDesc[threadIdx.x] = sqDesc[threadIdx.x] + sqDesc[threadIdx.x + 2];
    // 	sqDesc[threadIdx.x] = sqDesc[threadIdx.x] + sqDesc[threadIdx.x + 1];
    //   }
    // __syncthreads();


    // compute length (square root)
    __shared__ float recip_len;
    if (threadIdx.x == 0)
      {
	float sum = 0.f;
	for(int i = 0; i < ASRL_SURF_DESCRIPTOR_DIM; i++)
	  {
	    sum += sqDesc[i];
	  }

	recip_len = rsqrtf(sum);
      }
    __syncthreads();

    // normalize and store in output
    d_descriptors[descriptor_base + threadIdx.x] = lookup * recip_len;	
  }
  

  //__global__ void print_len_kernel(float * d_descriptors)
  // {
  //   // no need for thread ID
  //   int descriptor_base = __mul24(blockIdx.x, ASRL_SURF_DESCRIPTOR_DIM);


  //   // read in the unnormalized descriptor values (squared)
  //   __shared__ float sqDesc[ASRL_SURF_DESCRIPTOR_DIM];
  //   float lookup = d_descriptors[descriptor_base + threadIdx.x];

  //   sqDesc[threadIdx.x] = lookup * lookup;
  //   __syncthreads();

  //   // This is broken in compute capability 2.1 for whatever reason.
  //   // // reduction to get total
  //   // if (threadIdx.x < 32)
  //   //   {
  //   // 	sqDesc[threadIdx.x] = sqDesc[threadIdx.x] + sqDesc[threadIdx.x + 32];
  //   // 	sqDesc[threadIdx.x] = sqDesc[threadIdx.x] + sqDesc[threadIdx.x + 16];
  //   // 	sqDesc[threadIdx.x] = sqDesc[threadIdx.x] + sqDesc[threadIdx.x + 8];
  //   // 	sqDesc[threadIdx.x] = sqDesc[threadIdx.x] + sqDesc[threadIdx.x + 4];
  //   // 	sqDesc[threadIdx.x] = sqDesc[threadIdx.x] + sqDesc[threadIdx.x + 2];
  //   // 	sqDesc[threadIdx.x] = sqDesc[threadIdx.x] + sqDesc[threadIdx.x + 1];
  //   //   }
  //   // __syncthreads();


  //   // compute length (square root)
   
  //   if (threadIdx.x == 0)
  //     {
  // 	float sum = 0.f;
  // 	for(int i = 0; i < ASRL_SURF_DESCRIPTOR_DIM; i++)
  // 	  {
  // 	    sum += sqDesc[i];
  // 	  }
	
  // 	printf("[%03d] %f\n",blockIdx.x, sqrtf(sum));
  //     }
  //   __syncthreads();

  // }
  

  void compute_descriptors(float * d_descriptors, Keypoint * d_features, int nFeaturesFound, bool weighted)
  {
    // Should we be checking for errors here?
    // compute unnormalized descriptors, then normalize them - odd indexing since grid must be 2D
    if (weighted)
      compute_descriptors_kernel <<< dim3(nFeaturesFound,16,1), dim3(5,5,1) >>> (d_descriptors, d_features);
    else
      compute_descriptors_unweighted_kernel <<< dim3(nFeaturesFound,16,1), dim3(5,5,1) >>> (d_descriptors, d_features);
      

    normalize_descriptors_kernel <<< dim3(nFeaturesFound,1,1), dim3(ASRL_SURF_DESCRIPTOR_DIM,1,1) >>> (d_descriptors);

  }

  // void print_len(float * d_descriptors, int nFeaturesFound)
  // {
  //   print_len_kernel <<< dim3(nFeaturesFound,1,1), dim3(ASRL_SURF_DESCRIPTOR_DIM,1,1) >>> (d_descriptors);
  // }

} // namespace asrl
