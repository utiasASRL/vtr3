////////////////////////////////////////////////////////////////////////////////
/// @brief Header file for the ASRL vision package
/// @details
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <vtr_vision/types.hpp>
#include <vtr_vision/sensors/sensor_model_types.hpp>
#include <iostream>

namespace vtr {
namespace vision {

////////////////////////////////////////////////////////////////////
/// @brief This class provides the base interface for vision sensors
///
/// @details
////////////////////////////////////////////////////////////////////
template <typename M, typename C>
class CameraModelInterface {

public:
  //
  typedef std::shared_ptr<CameraModelInterface> Ptr;

  ////////////////////////////////////////////////////////////////////
  /// @brief Verify the points that will be used for model evaluations
  /// @param [in] matches The matches to verify
  /// @return true if the matches are acceptable
  ////////////////////////////////////////////////////////////////////
  virtual bool verifyMatches(const SimpleMatches& matches) const = 0;

  ////////////////////////////////////////////////////////////////////
  /// @brief Set the points that will be used for model evaluations (owned by user)
  /// @param [in] pts_ref The points seen at the reference frame
  /// @param [in] pts_query The points seen at the query frame
  ////////////////////////////////////////////////////////////////////
  virtual void setPoints(const M* pts_ref,
                         const C* pts_query) {
    // Save to members
    pts_ref_ = pts_ref;
    pts_query_ = pts_query;
  }

protected:

  /// @brief The points from the reference frame
  const M* pts_ref_;

  /// @brief The points from frame b
  const C* pts_query_;

};

} // namespace vision
} // namespace vtr_vision
