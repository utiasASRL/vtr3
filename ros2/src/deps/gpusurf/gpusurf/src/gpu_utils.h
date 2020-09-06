/**
 * @file   gpu_utils.h
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Wed Apr 21 06:44:18 2010
 * 
 * @brief  Helper inline functions for kernels.
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


#ifndef ASRL_GPU_UTILS_FTK_H
#define ASRL_GPU_UTILS_FTK_H

#ifdef __CUDACC__
/**
 * \def ASRL_CUDA_DEVICE
 * \brief a macro that allows us to use functions on the host and the device.
 */
#  define ASRL_CUDA_DEVICE __device__
#else
#  define ASRL_CUDA_DEVICE
#endif

/** 
 * Clears the last bit on an integer
 * 
 * @param f 
 */
ASRL_CUDA_DEVICE inline void clearLastBit(int  * f)
{
  *f &= ~0x1;
}

/** 
 * Clears the last bit on a float.
 * 
 * @param f 
 */
ASRL_CUDA_DEVICE inline void clearLastBit(float &f)
{
  clearLastBit((int*)&f);
}

/** 
 * Sets the last bit on an integer
 * 
 * @param f 
 */
ASRL_CUDA_DEVICE inline void setLastBit(int * f)
{
  *f |= 0x1;
}

/** 
 * Sets the last bit on a float
 * 
 * @param f 
 */
ASRL_CUDA_DEVICE inline void setLastBit(float & f)
{
  setLastBit((int*)&f);
}

/** 
 * Checks if the last bit is set on an integer
 * 
 * @param f The integer to check
 * 
 * @return True if the last bit on the integer is set, fals otherwise
 */
ASRL_CUDA_DEVICE inline bool isLastBitSet(const int * f)
{
  return (*f & 0x1);
}


/** 
 * Checks if the last bit is set on a float.
 * 
 * @param f The float to check
 * 
 * @return True if the last bit on the integer is set, fals otherwise
 */
ASRL_CUDA_DEVICE inline bool isLastBitSet(const float & f)
{
  return isLastBitSet((const int*)&f);
}


#endif  // ASRL_GPU_UTILS_FTK_H


