////////////////////////////////////////////////////////////////////////////////
/// @brief Header file for the ASRL vision package
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#pragma once

// Internal
#include "vtr_vision/sensors/sensor_model_types.hpp"
#include "vtr_vision/outliers/sampler/verify_sample_indices.hpp"

namespace vtr {
namespace vision {

////////////////////////////////////////////////////////////////////
/// @brief This class is a sensor model callback interface for ransac
///
/// @details
////////////////////////////////////////////////////////////////////
template<typename SolutionType>
class SensorModelBase : public VerifySampleIndices {
public:
  /// @brief Class shared pointer
  typedef std::shared_ptr<SensorModelBase> Ptr;

  ////////////////////////////////////////////////////////////////////
  /// @brief Virtual destructor
  ////////////////////////////////////////////////////////////////////
  virtual ~SensorModelBase() = default;

  ////////////////////////////////////////////////////////////////////
  /// @brief Get the number of points required for a sample
  /// @return The number of points
  ////////////////////////////////////////////////////////////////////
  virtual unsigned int getN() const = 0;

  ////////////////////////////////////////////////////////////////////
  /// @brief Solve for the state hypothesis given point pairs
  /// @param [in] matches The matches used in the solve
  /// @param [out] hypothesis The hypothesis generated
  /// @param [in] theshold The threshold used to check that the matches are inliers themselves
  ////////////////////////////////////////////////////////////////////
  virtual bool solveModel(const SimpleMatches& matches,
                          SolutionType * hypothesis,
                          double threshold) const = 0;

  ////////////////////////////////////////////////////////////////////
  /// @brief Compute the Mahalanobis distance for each point pair given the model
  /// @param [in] matches The matches for which to compute the error
  /// @param [in] hypothesis The state hypothesis being tested
  /// @param [out] whitened_error The whitened error in standard deviations for each match
  /// @param [out] robust_error The Geman-McClure cumulative robust cost for all of the points
  /// @param [in] stop_error The error at which to stop evaluating, since there is a better hypothesis
  /// @param [in] sigma2 The sigma2 for the Geman-McClure robust cost function
  /// @return true if the error was not aborted early by the stop_error
  ////////////////////////////////////////////////////////////////////
  virtual bool computeError(const SimpleMatches& matches,
                            const SolutionType& hypothesis,
                            ErrorList* whitened_error,
                            double * robust_error,
                            double stop_error,
                            double sigma2) const = 0;
};

extern template class SensorModelBase<Eigen::Matrix3d>;
extern template class SensorModelBase<Eigen::Matrix4d>;

} // namespace vision
} // namespace vtr_vision
