//////////////////////////////////////////////////////////////////////////////////////////////
/// \file Pool.hpp
/// \brief Implements a basic singleton object pool. The implementation is fairly naive,
///        but should be fast given its assumptions. The purpose of this object to avoid
///        making many small dynamic allocations during block automatic evaluation.
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_POOL_HPP
#define STEAM_POOL_HPP

#include <iostream>
#include <vector>
#include <stdexcept>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Pool class
///
/// While the object pool class could be implemented with a linked list, we choose to use
/// a simple array of max size for efficiency. It makes the logic a bit simpler, but more
/// importantly, if someone forgets to return an object, we do not have memory leaks.
//////////////////////////////////////////////////////////////////////////////////////////////
template<typename TYPE, int MAX_SIZE = 50>
class Pool {
 public:

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Default constructor
  ////////////////////////////////////////////////////////////////////////////////////////////
  Pool();

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Destructor
  ////////////////////////////////////////////////////////////////////////////////////////////
  ~Pool();

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Get an object
  ////////////////////////////////////////////////////////////////////////////////////////////
  TYPE* getObj();

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Return an object to the pool
  ////////////////////////////////////////////////////////////////////////////////////////////
  void returnObj(TYPE* object);

 private:

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Array of objects
  ////////////////////////////////////////////////////////////////////////////////////////////
  TYPE* resources_;

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Array of availability
  ////////////////////////////////////////////////////////////////////////////////////////////
  bool available_[MAX_SIZE];

  ////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Current index of next most likely available resource
  ////////////////////////////////////////////////////////////////////////////////////////////
  unsigned int index_;
};

} // steam

#include <steam/evaluator/blockauto/Pool-inl.hpp>

#endif // STEAM_POOL_HPP