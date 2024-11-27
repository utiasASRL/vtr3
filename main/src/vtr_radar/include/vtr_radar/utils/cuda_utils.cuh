#pragma once
#include <cuda_runtime.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
// force align to 16B for eigen-cuda-cpp classes
#ifdef __CUDACC__
#define ALIGN(x) __align__(x)
#else
#define ALIGN(x) alignas(x)
#endif
namespace vtr {
namespace radar {
  static void HandleError(cudaError_t err, const char* file, int line) {
    // CUDA error handeling from the "CUDA by example" book
    if (err != cudaSuccess) {
      printf("%s in %s at line %d\n", cudaGetErrorString(err), file, line);
      exit(EXIT_FAILURE);
    }
  }
#define CUDA_CHECK(err) (HandleError(err, __FILE__, __LINE__))
  inline int getDeviceInfo() {
    int n_devices = 0;
    // int available_shmem = 0;
    cudaGetDeviceCount(&n_devices);
    // check for devices
    for (int i = 0; i < n_devices; ++i) {
      cudaDeviceProp prop;
      cudaGetDeviceProperties(&prop, i);
      printf("device number: %d\n", i);
      printf("  device name: %s\n", prop.name);
      printf("  memory clock rate [KHz]: %d\n", prop.memoryClockRate);
      printf("  memory bus width [bits]: %d\n", prop.memoryBusWidth);
      printf("  peak memory bandwidth [GB/s]: %f\n",
             2.0 * prop.memoryClockRate * (prop.memoryBusWidth / 8) / 1.0e6);
      printf("  shared mem size per block [KB]: %li\n", prop.sharedMemPerBlock);
      std::cerr << "_______________________________________________" << std::endl;
    }
    if (n_devices > 1) {
      std::cerr << "multiple devices found, using devices number 0" << std::endl;
      std::cerr << "_______________________________________________" << std::endl;
    }
    std::cerr << std::endl;
    return n_devices;
  }
  template<class T> struct numeric_limits{
      typedef T type;
      __host__ __device__ static type min()  { return type(); }
      __host__ __device__ static type max() { return type(); }
  };
  template<> struct numeric_limits<unsigned long>{
      typedef unsigned long type;
      __host__ __device__ static type min() { return 0; }
      __host__ __device__ static type max() { return ULONG_MAX; }
  };
  template<> struct numeric_limits<unsigned long long>{
      typedef unsigned long long type;
      __host__ __device__ static type min() { return 0; }
      __host__ __device__ static type max() { return UINT64_MAX; }
  };
  template<> struct numeric_limits<float>{
      typedef float type;
      __host__ __device__ static type min() { return 1.175494351e-38f; }
      __host__ __device__ static type max() { return 3.402823466e+38f; }
  };
  template<> struct numeric_limits<double>{
      typedef double type;
      __host__ __device__ static type min() { return 2.2250738585072014e-308; }
      __host__ __device__ static type max() { return 1.7976931348623158e+308; }
  };
} // namespace radar
} // namespace vtr