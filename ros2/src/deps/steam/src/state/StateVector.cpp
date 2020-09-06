//////////////////////////////////////////////////////////////////////////////////////////////
/// \file StateVector.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/state/StateVector.hpp>
#include <steam/blockmat/BlockVector.hpp>

#include <iostream>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Default constructor
//////////////////////////////////////////////////////////////////////////////////////////////
StateVector::StateVector() : numBlockEntries_(0) {}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Copy constructor -- deep copy
//////////////////////////////////////////////////////////////////////////////////////////////
StateVector::StateVector(const StateVector& other) : states_(other.states_),
  numBlockEntries_(other.numBlockEntries_) {

  // Map is copied in initialization list to avoid re-hashing all the entries,
  // now we go through the entries and perform a deep copy
  boost::unordered_map<StateID, StateContainer>::iterator it = states_.begin();
  for(; it != states_.end(); ++it) {
    it->second.state = it->second.state->clone();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Assignment operator -- deep copy
//////////////////////////////////////////////////////////////////////////////////////////////
StateVector& StateVector::operator= (const StateVector& other) {

  // Copy-swap idiom
  StateVector tmp(other); // note, copy constructor makes a deep copy
  std::swap(states_, tmp.states_);
  std::swap(numBlockEntries_, tmp.numBlockEntries_);
  return *this;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Copy the values of 'other' into 'this' (states must already align, typically
///        this means that one is already a deep copy of the other)
//////////////////////////////////////////////////////////////////////////////////////////////
void StateVector::copyValues(const StateVector& other) {

  // Check state vector are the same size
  if (this->states_.empty() ||
      this->numBlockEntries_ != other.numBlockEntries_ ||
      this->states_.size() != other.states_.size()) {
    throw std::invalid_argument("StateVector size was not the same in copyValues()");
  }

  // Iterate over the state vectors and perform a "deep" copy without allocation new memory.
  // Keeping the original pointers is important as they are shared in other places, and we
  // want to update the shared memory.
  // todo: can we avoid a 'find' here?
  boost::unordered_map<StateID, StateContainer>::iterator it = states_.begin();
  for(; it != states_.end(); ++it) {

    // Find matching state by ID
    boost::unordered_map<StateID, StateContainer>::const_iterator itOther = other.states_.find(it->second.state->getKey().getID());

    // Check that matching state was found and has the same structure
    if (itOther == other.states_.end() ||
        it->second.state->getKey().getID() != itOther->second.state->getKey().getID() ||
        it->second.localBlockIndex != itOther->second.localBlockIndex) {
      throw std::runtime_error("StateVector was missing an entry in copyValues(), "
                               "or structure of StateVector did not match.");
    }

    // Copy
    it->second.state->setFromCopy(itOther->second.state);
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Add state variable
//////////////////////////////////////////////////////////////////////////////////////////////
void StateVector::addStateVariable(const StateVariableBase::Ptr& state) {

  // Verify that state is not locked
  if (state->isLocked()) {
    throw std::invalid_argument("Tried to add locked state variable to "
                                "an optimizable state vector");
  }

  // Verify we don't already have this state
  StateKey key = state->getKey();
  if (this->hasStateVariable(key)) {
    throw std::runtime_error("StateVector already contains the state being added.");
  }

  // Create new container
  StateContainer newEntry;
  newEntry.state = state; // copy the shared_ptr (increases ref count)
  newEntry.localBlockIndex = numBlockEntries_;
  states_[key.getID()] = newEntry;

  // Increment number of entries
  numBlockEntries_++;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Check if a state variable exists in the vector
//////////////////////////////////////////////////////////////////////////////////////////////
bool StateVector::hasStateVariable(const StateKey& key) const {

  // Find the StateContainer for key
  boost::unordered_map<StateID, StateContainer>::const_iterator it = states_.find(key.getID());

  // Return if found
  return it != states_.end();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get a state variable using a key
//////////////////////////////////////////////////////////////////////////////////////////////
StateVariableBase::ConstPtr StateVector::getStateVariable(const StateKey& key) const {

  // Find the StateContainer for key
  boost::unordered_map<StateID, StateContainer>::const_iterator it = states_.find(key.getID());

  // Check that it was found
  if (it == states_.end()) {
    throw std::runtime_error("State variable was not found in call to getStateVariable()");
  }

  // Return state variable reference
  return it->second.state;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get number of state variables
//////////////////////////////////////////////////////////////////////////////////////////////
unsigned int StateVector::getNumberOfStates() const {
  return states_.size();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get the block index of a state
//////////////////////////////////////////////////////////////////////////////////////////////
int StateVector::getStateBlockIndex(const StateKey& key) const {

  // Find the StateContainer for key
  boost::unordered_map<StateID, StateContainer>::const_iterator it = states_.find(key.getID());

  // Check that the state exists in the state vector
  //  **Note the likely causes that this occurs:
  //      1)  A cost term includes a state that is not added to the problem
  //      2)  A cost term is not checking whether states are locked, and adding a Jacobian for a locked state variable
  if (it == states_.end()) {
    std::stringstream ss; ss << "Tried to find a state that does not exist "
                                "in the state vector (ID: " << key.getID() << ").";
    throw std::runtime_error(ss.str());
  }

  // Return block index
  return it->second.localBlockIndex;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Get an ordered list of the sizes of the 'block' state variables
//////////////////////////////////////////////////////////////////////////////////////////////
std::vector<unsigned int> StateVector::getStateBlockSizes() const {

  // Init return
  std::vector<unsigned int> result;
  result.resize(states_.size());

  // Iterate over states and populate result
  for (boost::unordered_map<StateID, StateContainer>::const_iterator it = states_.begin();
       it != states_.end(); ++it ) {

    // Check that the local block index is in a valid range
    if (it->second.localBlockIndex < 0 ||
        it->second.localBlockIndex >= (int)result.size()) {
      throw std::logic_error("localBlockIndex is not a valid range");
    }

    // Populate return vector
    result[it->second.localBlockIndex] = it->second.state->getPerturbDim();
  }

  return result;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Update the state vector
//////////////////////////////////////////////////////////////////////////////////////////////
void StateVector::update(const Eigen::VectorXd& perturbation) {

  // Convert single vector to a block-vector of perturbations (this checks sizes)
  BlockVector blkPerturb(this->getStateBlockSizes(), perturbation);

  // Iterate over states and update each
  for ( boost::unordered_map<StateID, StateContainer>::const_iterator it = states_.begin(); it != states_.end(); ++it ) {

    // Check for valid index
    if (it->second.localBlockIndex < 0) {
      throw std::runtime_error("localBlockIndex is not initialized");
    }

    // Update state
    it->second.state->update(blkPerturb.at(it->second.localBlockIndex));
  }
}

} // steam
