//////////////////////////////////////////////////////////////////////////////////////////////
/// \file OpenMpPool-inl.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/evaluator/blockauto/OpenMpPool.hpp>

namespace steam {

////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Default constructor
////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, int MAX_SIZE>
OmpPool<TYPE,MAX_SIZE>::OmpPool() {

  // Initialize pointers
  for (int i = 0; i < MAX_NUM_THREADS; i++) {
    pools_[i] = NULL;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Destructor
////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, int MAX_SIZE>
OmpPool<TYPE,MAX_SIZE>::~OmpPool() {

  // Deallocate pools
  for (int i = 0; i < MAX_NUM_THREADS; i++) {
    if (pools_[i]) {
      delete pools_[i];
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get an object
////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, int MAX_SIZE>
TYPE* OmpPool<TYPE,MAX_SIZE>::getObj() {

  // Unique OpenMP thread identifier
  int tid = omp_get_thread_num();

  // Get thread identifier is in range
  if (tid < 0 || tid >= MAX_NUM_THREADS) {
    throw std::runtime_error("Thread ID is higher than maximum number of threads allowed in pool.");
  }

  // Check if we have allocated a pool for this thread
  if (pools_[tid] == NULL) {
    pools_[tid] = new Pool<TYPE,MAX_SIZE>();
  }

  // Return result from pool
  return pools_[tid]->getObj();
}

////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Return an object to the pool
////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, int MAX_SIZE>
void OmpPool<TYPE,MAX_SIZE>::returnObj(TYPE* object) {

  // Unique OpenMP thread identifier
  int tid = omp_get_thread_num();

  // Get thread identifier is in range
  if (tid < 0 || tid >= MAX_NUM_THREADS) {
    throw std::runtime_error("Thread ID is higher than maximum number of threads allowed in pool.");
  }

  // Check if we have allocated a pool for this thread
  if (pools_[tid] == NULL) {
    throw std::runtime_error("Resource returned to an OpenMP Pool that does not exist.");
  }

  // Return object to pool
  pools_[tid]->returnObj(object);
}

} // steam

