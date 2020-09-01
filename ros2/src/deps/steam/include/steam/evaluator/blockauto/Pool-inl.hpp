//////////////////////////////////////////////////////////////////////////////////////////////
/// \file Pool.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/evaluator/blockauto/Pool.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Default constructor
//////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, int MAX_SIZE>
Pool<TYPE,MAX_SIZE>::Pool() {
  index_ = 0;
  resources_ = new TYPE[MAX_SIZE];
  for (unsigned int i = 0; i < MAX_SIZE; i++) {
    available_[i] = true;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Destructor
////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, int MAX_SIZE>
Pool<TYPE,MAX_SIZE>::~Pool() {
  if (resources_) {
    delete [] resources_;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get an object
////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, int MAX_SIZE>
TYPE* Pool<TYPE,MAX_SIZE>::getObj() {

  // Check if current is availble
  if (!available_[index_]) {

    // Increment index (wraps at end)
    index_ = ((MAX_SIZE-1) == index_) ? 0 : (index_+1);

    // Loop over entire array once
    unsigned int i = 0;
    for (; i < MAX_SIZE; i++) {
      if (available_[index_]) {
        // Found an available object, break out and return it
        break;
      } else {
        // Increment index (wraps at end)
        index_ = ((MAX_SIZE-1) == index_) ? 0 : (index_+1);
      }
    }

    // Check that we found an available resource, otherwise we need to throw a runtime error
    if (i == MAX_SIZE) {
      throw std::runtime_error("Pool ran out of entries... make sure they are being released.");
    }
  }

  // Mark as not available and give away resource
  available_[index_] = false;
  TYPE* result = &resources_[index_];
  index_ = ((MAX_SIZE-1) == index_) ? 0 : (index_+1);
  return result;
}

////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Return an object to the pool
////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, int MAX_SIZE>
void Pool<TYPE,MAX_SIZE>::returnObj(TYPE* object) {

  // Reset the objects data
  object->reset();

  // Calculate the index from the pointer
  std::ptrdiff_t index = object - &resources_[0];

  // Set its available to true
  available_[index] = true;
}

} // steam

