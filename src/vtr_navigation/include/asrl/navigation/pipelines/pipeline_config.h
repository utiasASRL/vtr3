#pragma once

#include <Eigen/Dense>

namespace asrl {
namespace navigation {

struct PipelineConfig {
  // merge pipeline configuration options
  Eigen::Matrix<double, 6, 6> default_loc_cov;

  /// Mostly for lancaster AutoMerge, integrate the poses
  /// to estimate the prior for localisation
  bool use_integrated_loc_prior;

  // localization search pipeline config options
  int loc_search_step;
};

}  // namespace navigation
}  // namespace asrl
