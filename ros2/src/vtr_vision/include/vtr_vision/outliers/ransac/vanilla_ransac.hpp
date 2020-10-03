////////////////////////////////////////////////////////////////////////////////
/// @brief VanillaRansac.h Header file for the ASRL vision package
/// @details This header file declares the VanillaRansac class
///
/// @author Kirk MacTavish, ASRL
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include "vtr_vision/outliers/ransac/ransac_base.hpp"

namespace vtr {
namespace vision {

////////////////////////////////////////////////////////////////////
/// @brief This class provides a basic RANSAC implementation
///
/// @details Uses a robust cost decision metric, instead of #inliers.
/// Stops the error calculation early if it exceeds the previous cost.
////////////////////////////////////////////////////////////////////
template<typename SolutionType>
class VanillaRansac
    : public RansacBase<SolutionType> {

public:

  /// @brief Class shared pointer
  typedef std::shared_ptr<VanillaRansac> Ptr;

  // Other helpful typedefs
  typedef RansacBase<SolutionType> Base;

  ////////////////////////////////////////////////////////////////////
  /// @brief Constructor
  /// @param [in] sampler The sampler generates random samples of matches
  /// @param [in] sigma The sigma for the Geman-McClure robust cost
  ///             function \f$\frac{e^2/2}{\sigma+e^2}\f$
  /// @param [in] threshold The inlier/outlier decision boundary in standard
  ///             deviations of measurement error
  ////////////////////////////////////////////////////////////////////
  VanillaRansac(const std::shared_ptr<BasicSampler>& sampler
                = std::make_shared<BasicSampler>(),
                const double& sigma = 3.5,
                const double& threshold = 5.0,
                unsigned int iterations = 1000,
                double early_stop_ratio = 0.9,
                double early_stop_min_inliers = 400,
                bool enable_local_opt = false,
                unsigned num_threads = 8)
    : Base(sampler), iterations_(iterations),
      sigma_(sigma), threshold_(threshold), 
      early_stop_ratio_(early_stop_ratio),
      early_stop_min_inliers_(early_stop_min_inliers),
      enable_local_opt_(enable_local_opt),
      num_threads_(num_threads) {
        
  }

  ////////////////////////////////////////////////////////////////////
  /// @brief Destructor
  ////////////////////////////////////////////////////////////////////
  ~VanillaRansac() = default;

  ////////////////////////////////////////////////////////////////////
  // Inherited
  ////////////////////////////////////////////////////////////////////
  virtual int run(const SimpleMatches& matches,
                  SolutionType * hypothesis,
                  SimpleMatches * inliers) const;

  ////////////////////////////////////////////////////////////////////
  // Inherited
  ////////////////////////////////////////////////////////////////////
  virtual int findInliers(const SimpleMatches& matches,
                          const ErrorList& whitened_error,
                          SimpleMatches * inliers) const;

protected:

  /// @brief The model that is being solved
  using Base::sampler_;

  /// @brief The model that is being solved
  using Base::cb_;

  /// @brief The number of iterations to run
  unsigned int iterations_;

  /// @brief The number of parallel threads
  unsigned int num_threads_;

  /// @brief The number of iterations to run
  double sigma_;

  /// @brief The number of iterations to run
  double threshold_;

  /// @brief The ratio required for early stopping
  double early_stop_ratio_;
        
  /// @brief The minimum inlier count required for early stopping
  double early_stop_min_inliers_;

  /// @brief Enables local optimization producing 2nd internal iteration
  bool enable_local_opt_;
}; // VanillaRansac

extern template class VanillaRansac<Eigen::Matrix3d>;
extern template class VanillaRansac<Eigen::Matrix4d>;

} // namespace vision
} // namespace vtr_vision

#include "vtr_vision/outliers/ransac/vanilla_ransac.inl"
