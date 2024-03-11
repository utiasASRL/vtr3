// -------------------------------------------------------------
// cuDPP -- CUDA Data Parallel Primitives library
// -------------------------------------------------------------
// $Revision$
// $Date$
// ------------------------------------------------------------- 
// This source code is distributed under the terms of license.txt
// in the root directory of this source distribution.
// ------------------------------------------------------------- 
#ifndef _MAXIMAL_LAUNCH_H_
#define _MAXIMAL_LAUNCH_H_

#include "cuda_runtime.h"

extern "C"
size_t maxBlocks(cudaFuncAttributes &attribs, 
                 cudaDeviceProp &devprop, 
                 size_t bytesDynamicSharedMem,
                 size_t threadsPerBlock);

extern "C"
size_t maxBlocksFromPointer(void* kernel, 
                            size_t   bytesDynamicSharedMem,
                            size_t   threadsPerBlock);

#ifdef __cplusplus

template <typename T>
size_t maxBlocks(T   kernel, 
                 size_t bytesDynamicSharedMem,
                 size_t threadsPerBlock)
{
    return maxBlocksFromPointer((void*)kernel, bytesDynamicSharedMem, threadsPerBlock);
}
#endif

#endif // _MAXIMAL_LAUNCH_H_
