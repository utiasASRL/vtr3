#pragma once

#include <vtr/navigation/modules/base_module.h>

#include <asrl/messages/exp_recog_status.pb.h>
#include <asrl/common/timing/TimeUtils.hpp>

namespace vtr {
namespace navigation {

////////////////////////////////////////////////////////////////////////////////
/// @brief Match the current live view to a multi-experience map.
///
/// Builds a local Bag-of-Words vocabulary for each 'place' along the teach.
/// Describes each image using its local vocabulary, incrementally adding new
/// words. Compares the live view to past experiences using the Bag-of-Words
/// descriptors.
////////////////////////////////////////////////////////////////////////////////
class TodRecognitionModule : public BaseModule {
 public:
  static constexpr auto type_str_ = "timeofday_recognition";
  typedef asrl::common::timing::time_point time_point;

  /// Configuration for the module, generally set by the module factory.
  struct Config {
    /// Enables debugging logs.
    bool verbose = true;

    // INTERFACING //
    /// The number of experiences (including the privileged)
    /// that we should be recommended for localization.
    int num_exp = 4;
    /// Whether we should broadcast our recommendations (are we enabled).
    bool in_the_loop = false;

    // PARAMETERS //
    /// The weight to convert time-of-day difference to a distance
    float time_of_day_weight = 1.f;
    /// The weight to convert total time difference to a distance
    float total_time_weight = 1.f / 24.f;
  };

  TodRecognitionModule(std::string name = type_str_) : BaseModule{name} {}

  /// The main entry point to running this module.
  /// See the module-level description for more info.
  virtual void run(QueryCache& qdata, MapCache& mdata,
                   const std::shared_ptr<const Graph>& graph);

  /// Does nothing
  virtual void updateGraph(QueryCache& qdata, MapCache& /*mdata*/,
                           const std::shared_ptr<Graph>& graph, VertexId vid);

  /// Set the config for the module.
  void setConfig(const Config& config  ///< The config we'd like to use
  ) {
    config_ = config;
  }

 private:
  /// Algorithm configuration
  Config config_;

  /// The status message to save to the graph
  asrl::status_msgs::ExpRecogStatus status_msg_;
};

/// Compute the time/time-of-day distance for the experiences in the submap
/// based on distance from a query point in time.
ScoredRids  /// @return The scored experiences
scoreExperiences(
    const TodRecognitionModule::time_point& query_tp,  ///< The query time point
    const asrl::pose_graph::RCGraphBase& submap,  ///< The submap to process
    const TodRecognitionModule::Config&
        config);  ///< Configuration (for weights)

}  // namespace navigation
}  // namespace vtr
