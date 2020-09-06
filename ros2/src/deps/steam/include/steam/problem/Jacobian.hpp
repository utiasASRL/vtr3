//////////////////////////////////////////////////////////////////////////////////////////////
/// \file Jacobian.hpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_JACOBIAN_HPP
#define STEAM_JACOBIAN_HPP

#include <Eigen/Dense>
#include <steam/state/StateVariableBase.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Simple structure to hold Jacobian information
//////////////////////////////////////////////////////////////////////////////////////////////
template <int LHS_DIM = Eigen::Dynamic,
          int MAX_STATE_DIM = Eigen::Dynamic>
struct Jacobian
{

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Default constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  Jacobian() {}

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Constructor
  //////////////////////////////////////////////////////////////////////////////////////////////
  Jacobian(const StateKey& key, const Eigen::Matrix<double, LHS_DIM, MAX_STATE_DIM>& jac)
    : key(key), jac(jac) {
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Go through vector of Jacobians and check for Jacobians which are with respect to
  ///        the same state variable, and merge them.
  ///
  /// For efficiency, specify a hintIndex, which specifies that Jacobians before hintIndex
  /// cannot be multiples of eachother.
  //////////////////////////////////////////////////////////////////////////////////////////////
  static void merge(std::vector<Jacobian<LHS_DIM,MAX_STATE_DIM> >* outJacobians, unsigned int hintIndex) {

    // Check inputs
    if (hintIndex > outJacobians->size()) {
      throw std::invalid_argument("The specified hintIndex is beyond the size of outJacobians");
    }

    // Iterate over the 'safe' non-duplicate Jacobians
    for (unsigned int j = 0; j < hintIndex; j++) {

      // Iterate over the branched (other) Jacobians
      for (unsigned int k = hintIndex; k < outJacobians->size();) {

        // Check if Jacobian j and k are w.r.t the same state variable.
        // If so, we must merge them and erase the second entry
        if (outJacobians->at(j).key.equals(outJacobians->at(k).key)) {

          // Merge
          outJacobians->at(j).jac += outJacobians->at(k).jac;

          // Erase duplicate
          outJacobians->erase(outJacobians->begin() + k);

          // Assuming merge has been called consistently, there should not exist
          // more than one duplicate in second half.. so we can go to the next 'j'
          break;

        } else {
          k++;
        }
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Key of the associated state variable
  //////////////////////////////////////////////////////////////////////////////////////////////
  StateKey key;

  //////////////////////////////////////////////////////////////////////////////////////////////
  /// \brief Jacobian matrix
  //////////////////////////////////////////////////////////////////////////////////////////////
  Eigen::Matrix<double, LHS_DIM, MAX_STATE_DIM> jac;
};

} // steam

#endif // STEAM_JACOBIAN_HPP
