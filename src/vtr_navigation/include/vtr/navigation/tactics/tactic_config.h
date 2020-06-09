#pragma once

#include <Eigen/Dense>

#include <vtr/navigation/pipelines/pipeline_config.h>

// #include <asrl/navigation/memory/MemoryConfig.hpp>
#include <asrl/pose_graph/path/LocalizationChain.hpp>

namespace asrl {
namespace navigation {

struct TacticConfig {
  // Configuration for the localization chain
  pose_graph::LocalizationChain::Config locchain_config;
  PipelineConfig pipeline_config;

#if 0
  // Memory Managers
  MapMemoryManagerConfig map_memory_config;
  LiveMemoryManagerConfig live_memory_config;
#endif

  // these define the localization covariance thresholds for defining if the
  // robot is dead reckoning or actually lost
  Eigen::Vector3d loc_deadreckoning_thresh;
  Eigen::Vector3d loc_lost_thresh;

  /// @brief Flag to enable parallelization of keyframe processing in the
  /// pipeline.
  bool keyframe_parallelization;

  /// @brief Flag to enable skipping of keyframe processing if the thread is not
  /// available.
  bool keyframe_skippable;

  /// @brief Flag to enable parallelization of localization processing in the
  /// pipeline.
  bool localization_parallelization;

  /// @brief Flag to enable skipping of localization processing if the thread is
  /// not available.
  bool localization_skippable;

  /// @brief Flag to enable extrapolation of VO to the path tracker.
  bool extrapolate_VO;

  /// The timeout [s] to stop using the extrapolation,
  /// since the velocity is likely wrong now.
  double extrapolate_timeout;

  /// Add a run on startup, for UAV state machine
  bool insert_initial_run;

  std::string data_directory;
  std::string sensor_frame;
  std::string vehicle_base_frame;

  int graph_index;
  int robot_index;

  /// Flag to enable running terrain assessment in a background thread.
  bool ta_parallelization;
};

}  // namespace navigation
}  // namespace asrl
