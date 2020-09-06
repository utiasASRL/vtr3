/**
 * @file   CudaSynchronizedMemory.hpp
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Thu Feb 18 21:55:11 2010
 *
 * @brief  A class that simplifies allocation/deallocation
 *         and synchronization of a buffer allocated on the
 *         host and on the device.
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

#ifndef ASRL_CUDA_SYNCHRONIZED_MEMORY_HPP
#define ASRL_CUDA_SYNCHRONIZED_MEMORY_HPP

#include <builtin_types.h>
#include <cuda_runtime_api.h>
#include "assert_macros.hpp"
#include <limits>
#include <iostream>
#include <string.h> // memset
#include <typeinfo> // typeid
namespace asrl {

  /**
   * @class CudaSynchronizedMemory
   * @brief a class that handles synchronization of memory between the host and device
   *
   * A class that simplifies allocation/deallocation
   * and synchronization of a buffer allocated on the
   * host and on the device.
   *
   */
  template<typename T>
  class CudaSynchronizedMemory
  {
    /**
     * The object is non-copyable as we need to be careful about memory leaks.
     *
     */
    CudaSynchronizedMemory(CudaSynchronizedMemory const & rhs);

    /**
     * The object is non-copyable as we need to be careful about memory leaks.
     *
     */
    CudaSynchronizedMemory & operator=(CudaSynchronizedMemory const & rhs);
  public:
    /// The underlying type of the allocated memory
    typedef T value_type;

    /**
     * A constructor that initializes both the host and device pointers with null values
     *
     */
    CudaSynchronizedMemory();

    /**
     * A constructor that allocates memory for N elements of type T
     * on both the host and the device.
     *
     * @param N The number of elements to allocate
     * @param pageLocked Should the host memory be page-locked
     */
    CudaSynchronizedMemory(unsigned N, bool pageLocked = false);


    /**
     * A destructor to clean up the allocated memory
     */
    ~CudaSynchronizedMemory();

    /**
     * A function that allocates memory for N elements of type T
     * on both the host and the device.
     *
     * @param N The number of elements to allocate
     * @param pageLocked Should the host memory be page-locked
     */
    void init(unsigned N, bool pageLocked = false);


    

    /**
     * Clear both the host and device memory.
     *
     */
    void reset();


    /**
     *
     * @return a pointer to the beginning of the host memory
     */
    T * begin();

    /**
     *
     * @return a pointer to the end of the host memory
     *
     */
    T * end();

    /**
     *
     * @return the host memory pointer
     */
    T * h_get();

    /**
     *
     * @return the host memory pointer
     *
     */
    T const * h_get() const;


    /**
     *
     * @return the device memory pointer
     *
     */
    T * d_get();

    /**
     *
     * @return the device memory pointer
     *
     */
    T const * d_get() const;

    /**
     *
     * @return the number of elements allocated
     *
     */
    size_t size() const;


    /**
     * An convenience operator that dereferences the host array
     *
     * @param i the index to dereference the host array
     *
     * @return a reference to the value at host_memory[i]
     */
    T & operator[](size_t i);

    /**
     * An convenience operator that dereferences the host array
     *
     * @param i the index to dereference the host array
     *
     * @return a const reference to the value at host_memory[i]
     */
    T const & operator[](size_t i) const;

    /**
     * Pushes bytes from the host to the device.
     *
     * @param nElements The number of elements to push. If this is greater than size() or -1, all allocated elements are pushed to the device.
     */
    void pushToDevice(size_t nElements = std::numeric_limits<size_t>::max());

    /**
     * Pulls bytes from the device to the host.
     *
     * @param nElements The number of elements of the array to pull. If this is greater than size() or -1, all allocated elements are pulled from the device.
     */
    void pullFromDevice(size_t nElements = std::numeric_limits<size_t>::max());

    /**
     * Initiate an asynchronous transfer from the host to the device.
     *
     * @param stream The cuda stream associated with the transfer.
     * @param nElements The number of elements to pull. If this is greater than size() or -1, all allocated elements are pulled from the device.
     */
    void pullFromDeviceAsync(cudaStream_t stream, size_t nElements = std::numeric_limits<size_t>::max());

    /**
     * calls memset() on the host memory.
     *
     */
    void memsetHost(int val);

    /**
     * calls cudaMemset() on the device memory.
     *
     */
    void memsetDevice(int val);

    /**
     * Calls memset on both the host and device memory.
     *
     */
    void memset(int val);

    /**
     *
     * @return returns true if the underlying host memory is page-locked
     */
    bool isPageLocked();
  private:

    void freeHostMemory();
    void freeDeviceMemory();

    /// A reference-counted pointer to the host memory
    T * m_host;
    /// A reference-counted pointer to the device memory
    T * m_device;
    /// The number of elements of type T allocated
    size_t m_size;
    /// Is the host memory page-locked
    bool m_pageLocked;

  };


  template<typename T>
  CudaSynchronizedMemory<T>::CudaSynchronizedMemory() : m_host(0), m_device(0), m_size(0), m_pageLocked(false) {}

  template<typename T>
  CudaSynchronizedMemory<T>::CudaSynchronizedMemory(unsigned N, bool pageLocked) : m_host(0), m_device(0), m_size(0), m_pageLocked(false)
  {
    init(N, pageLocked);
  }

  template<typename T>
  CudaSynchronizedMemory<T>::~CudaSynchronizedMemory()
  {
    try 
      {
	reset();
      } 
    catch(std::exception const & e)
      {
	std::cerr << "Error in " << __FUNCTION__ << ": " << e.what() << std::endl;
      }
  }

  template<typename T>
  void CudaSynchronizedMemory<T>::reset()
  {

    freeHostMemory();
    freeDeviceMemory();
    m_size = 0;
    m_pageLocked = false;
  }

  template<typename T>
  void CudaSynchronizedMemory<T>::freeHostMemory()
  {
    if(m_host != NULL)
      {
	  if(m_pageLocked)
	    {
	      cudaError_t err = cudaFreeHost(m_host);
	      m_host = NULL;
	      ASRL_ASSERT_EQ(err, cudaSuccess,
			     "Unable to free page-locked host memory for " << m_size << " elements of type "
			     << typeid(T).name() << ": " << cudaGetErrorString(err));
	    }
	  else
	    {
	      delete [] m_host;
	      m_host = NULL;
	    }

      }
  }

  template<typename T>
  void CudaSynchronizedMemory<T>::freeDeviceMemory()
  {
    if(m_device != NULL)
      {
	cudaError_t err = cudaFree(m_device);
	m_device = NULL;
	ASRL_ASSERT_EQ(err, cudaSuccess,
		       "Unable to free device memory for " << m_size << " elements of type "
		       << typeid(T).name() << ": " << cudaGetErrorString(err));
	
      }

  }


  template<typename T>
  void CudaSynchronizedMemory<T>::memset(int val)
  {
    ASRL_ASSERT_GT_DBG(m_size, 0, "The array is empty");
    memsetHost(val);
    memsetDevice(val);
  }

  template<typename T>
  void CudaSynchronizedMemory<T>::memsetHost(int val)
  {
    ASRL_ASSERT_GT_DBG(m_size, 0, "The array is empty");
    ::memset(m_host, val, m_size * sizeof(T));
  }

  template<typename T>
  void CudaSynchronizedMemory<T>::memsetDevice(int val)
  {
    ASRL_ASSERT_GT_DBG(m_size, 0, "The array is empty");
    cudaError_t err = cudaMemset(m_device,val,m_size*sizeof(T));
    ASRL_ASSERT_EQ(err, cudaSuccess, "Unable to set device memory of " << m_size << " elements of type " << typeid(T).name() );
  }


  template<typename T>
  void CudaSynchronizedMemory<T>::init(unsigned N, bool pageLocked)
  {
    try {
      ASRL_ASSERT_GT(N,0,"Trying to allocate zero bytes");
      // Free any existing memory.
      reset();
      
      // Allocate the new memory.
      if(pageLocked) {
	cudaError_t e = cudaMallocHost((void**)&m_host,N*sizeof(T));
	ASRL_ASSERT_EQ(e, cudaSuccess,
			 "Unable to allocate page-locked host memory for " << N << " elements of type "
			 << typeid(T).name() << ": " << cudaGetErrorString(e));

      } else {
	try {
	  m_host = new T[N];
	} catch(std::exception const & e) {
	  ASRL_THROW(
		    "Unable to allocate host memory for " << N << " elements of type "
		    << typeid(T).name() << ": " << e.what());
	}
      }

      cudaError_t err = cudaMalloc((void**)&m_device,N*sizeof(T));
      ASRL_ASSERT_EQ(err, cudaSuccess,
		       "Unable to allocate device memory for " << N << " elements of type "
		       << typeid(T).name() << ": " << cudaGetErrorString(err));

      m_size = N;
      m_pageLocked = pageLocked;
    } 
    catch(std::exception const &)
      {
	reset();
	throw;
      }
  }

  template<typename T>
  bool CudaSynchronizedMemory<T>::isPageLocked()
  {
    return m_pageLocked;
  }

  template<typename T>
  T * CudaSynchronizedMemory<T>::begin()
  {
    ASRL_ASSERT_GT_DBG(m_size, 0, "The array is empty");
    return m_host;
  }

  template<typename T>
  T * CudaSynchronizedMemory<T>::end()
  {
    ASRL_ASSERT_GT_DBG(m_size, 0, "The array is empty");
    return m_host + m_size;
  }

  template<typename T>
  T * CudaSynchronizedMemory<T>::h_get()  { return m_host;   }

  template<typename T>
  T const * CudaSynchronizedMemory<T>::h_get() const  { return m_host;   }

  template<typename T>
  T * CudaSynchronizedMemory<T>::d_get()  { return m_device; }

  template<typename T>
  T const * CudaSynchronizedMemory<T>::d_get() const  { return m_device; }

  template<typename T>
  size_t CudaSynchronizedMemory<T>::size() const { return m_size;         }

  template<typename T>
  T & CudaSynchronizedMemory<T>::operator[](size_t i) {
    ASRL_ASSERT_GT_DBG(m_size, 0, "The array is empty");
    ASRL_ASSERT_LT_DBG(i, m_size, "Index out of range: " << i << " >= " << m_size);//@}
    return m_host[i];
  }

  template<typename T>
  T const & CudaSynchronizedMemory<T>::operator[](size_t i) const {
    ASRL_ASSERT_GT_DBG(m_size, 0, "The array is empty");
    ASRL_ASSERT_LT_DBG(i, m_size, "Index out of range: " << i << " >= " << m_size);
    return m_host[i];
  }

  template<typename T>
  void CudaSynchronizedMemory<T>::pushToDevice(size_t nElements)
  {
    ASRL_ASSERT_GT_DBG(m_size,0, "The array is empty");
    ASRL_ASSERT_NE_DBG(m_host,0, "The host pointer is null");
    ASRL_ASSERT_NE_DBG(m_device,0, "The device pointer is null");
    if(nElements > m_size)
      nElements = m_size;

    cudaError_t err = (cudaMemcpy((void*) m_device, (void *)m_host, nElements*sizeof(T), cudaMemcpyHostToDevice));
    ASRL_ASSERT_EQ(err, cudaSuccess, "Unable to copy to device " << nElements << "/" << m_size << " elements of type " << typeid(T).name() << " (" << nElements * sizeof(T) << " bytes): " << cudaGetErrorString(err) << " (errorId: " << err << "). Host pointer: " << ((void*)m_host) << ", device pointer: " << ((void*) m_device));
  }

  template<typename T>
  void CudaSynchronizedMemory<T>::pullFromDevice(size_t nElements)
  {

    ASRL_ASSERT_GT_DBG(m_size,0, "The array is empty");
    ASRL_ASSERT_NE_DBG(m_host,0, "The host pointer is null");
    ASRL_ASSERT_NE_DBG(m_device,0, "The device pointer is null");
    if(nElements > m_size)
      nElements = m_size;

    cudaError_t err = (cudaMemcpy((void*) m_host, (void *)m_device, nElements*sizeof(T), cudaMemcpyDeviceToHost));
    ASRL_ASSERT_EQ(err, cudaSuccess, "Unable to copy from device " << nElements << "/" << m_size << " elements of type " << typeid(T).name() << " (" << nElements * sizeof(T) << " bytes): " << cudaGetErrorString(err) << " (errorId: " << err << "). Host pointer: " << ((void*)m_host) << ", device pointer: " << ((void*) m_device));

  }

  template<typename T>
  void CudaSynchronizedMemory<T>::pullFromDeviceAsync(cudaStream_t stream, size_t nElements)
  {
    ASRL_ASSERT_GT(m_size,0, "The array is empty");
    ASRL_ASSERT(m_pageLocked, "Asynchronous transfer is only valid for page-locked host memory");
    if(nElements > m_size)
      nElements = m_size;
    cudaError_t err = (cudaMemcpyAsync((void*) m_host, (void *)m_device, nElements*sizeof(T), cudaMemcpyDeviceToHost, stream));
    ASRL_ASSERT_EQ(err,cudaSuccess, "Unable to copy " << typeid(T).name() << " array of size " << m_size << " from device. Stream " << stream << ": (" << err << "): " << cudaGetErrorString(err));
  }

} // namespace asrl

#endif // ASRL_CUDA_SYNCHRONIZED_MEMORY_HPP
