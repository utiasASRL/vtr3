#pragma once
#include <vtr_mission_planning/state_machine_interface.hpp>
#include <vtr_navigation/caches.hpp>
#include <vtr_navigation/publisher_interface.hpp>
#include <vtr_navigation/tactics/tactic_config.hpp>
#include <vtr_navigation/types.hpp>

#if 0
#include <vtr/navigation/pipelines.h>  // should not include anything related to pipling, use forward declearation instead.
#include <vtr/navigation/Navigator.hpp>
#include <vtr_navigation/pipelines/base_pipeline.hpp>

#include <vtr/vision/features/extractor/feature_extractor_factory.h>
#include <asrl/path_tracker/base.hpp>
#include <asrl/pose_graph/path/LocalizationChain.hpp>
#endif

namespace vtr {
namespace navigation {

/// \todo These forward declarations are here because we need to include tactic
/// when building pipelines. We need to clean this up somehow as this is very
/// confusing.
class BasePipeline;
class ConverterAssembly;
class QuickVoAssembly;
class RefinedVoAssembly;
class LocalizerAssembly;

#if false
class LiveMemoryManager;
class MapMemoryManager;
#endif

using QueryCachePtr = std::shared_ptr<QueryCache>;
using MapCachePtr = std::shared_ptr<MapCache>;

/**
 * \brief Supposed to be the base class of tactic. API for a tactic is not clear
 */
class BasicTactic : public mission_planning::StateMachineInterface {
 public:
  using LockType = std::unique_lock<std::recursive_timed_mutex>;

  BasicTactic(TacticConfig& config,
              const std::shared_ptr<ConverterAssembly>& converter,
              const std::shared_ptr<QuickVoAssembly>& quick_vo,
              const std::shared_ptr<RefinedVoAssembly>& refined_vo,
              const std::shared_ptr<LocalizerAssembly>& localizer,
              std::shared_ptr<Graph> graph = nullptr);

  ~BasicTactic() override { this->halt(); }

  /** \brief Necessary for the factory. */
  bool verify() const { return true; }

  /** \brief Calling halt stops all associated processes/threads. */
  virtual void halt();

  /** \brief Set the operational mode (which pipeline to use). */
  void setPipeline(const mission_planning::PipelineType& pipeline) override;

  /**
   * \brief Clears the pipeline and stops callbacks.
   * \return LockType A lock that blocks the pipeline
   */
  LockType lockPipeline() override;

  /** \brief Set the path being followed. */
  void setPath(const mission_planning::PathType& path,
               bool follow = false) override;

  /** \brief Set the current privileged vertex (topological localization) */
  void setTrunk(const VertexId& v) override;

  double distanceToSeqId(const uint64_t&) override;
  mission_planning::TacticStatus status() const override;
#if false
  mission_planning::LocalizationStatus tfStatus(
      const EdgeTransform& tf) const override;
#endif
  const VertexId& connectToTrunk(bool privileged = false) override;
  const Localization& persistentLoc() const override {
    return persistentLocalization_;
  }
  const Localization& targetLoc() const override { return targetLocalization_; }

  virtual inline void incrementLocCount(int8_t diff) {
#if false
    persistentLocalization_.successes =
        std::max(persistentLocalization_.successes + diff, 0);
    persistentLocalization_.successes =
        std::min(int8_t(5), persistentLocalization_.successes);
    targetLocalization_.successes =
        std::max(targetLocalization_.successes + diff, 0);
    targetLocalization_.successes =
        std::min(int8_t(5), targetLocalization_.successes);
#endif
  }

  /** brief Add a new run to the graph and reset localization flags */
  virtual void addRun(bool ephemeral = false, bool extend = false,
                      bool save = true) {
    LOG(DEBUG) << "[Lock Requested] addRun";
    auto lck = lockPipeline();
    LOG(DEBUG) << "[Lock Acquired] addRun";

    pose_graph_->addRun(config_.robot_index, ephemeral, extend, save);

    // mostly for monocular, extend the previous map if we already have one
    if (extend && !first_frame_) {
      map_status_ = MAP_EXTEND;
    } else {
      // otherwise, just re-initialize the run as normal
      first_frame_ = true;
      map_status_ = MAP_NEW;
    }

    targetLocalization_ = Localization();  // \todo yuchen why initialized here?

    LOG(DEBUG) << "[Lock Released] addRun";
  }

#if 0
  /** \brief Remove any temporary runs. */
  virtual void removeEphemeralRuns() {
    LOG(DEBUG) << "[Lock Requested]removeEphemeralRuns";
    auto lck = lockPipeline();
    LOG(DEBUG) << "[Lock Acquired] removeEphemeralRuns";

    pose_graph_->removeEphemeralRuns();
    first_frame_ = true;
    map_status_ = MAP_NEW;

    LOG(DEBUG) << "[Lock Released] removeEphemeralRuns";
  }
#endif
  /** \brief Trigger a graph relaxation. */
  virtual void relaxGraph() {
#if 0
    pose_graph_->callbacks()->updateRelaxation(steam_mutex_ptr_);
#endif
  }

  /** \brief Save the graph. */
  virtual void saveGraph() {
    LOG(DEBUG) << "[Lock Requested] saveGraph";
    auto lck = lockPipeline();
    LOG(DEBUG) << "[Lock Acquired] saveGraph";
    pose_graph_->save();
    LOG(DEBUG) << "[Lock Released] saveGraph";
  }

  /** \return The pose graph that's being navigated */
  std::shared_ptr<Graph> poseGraph() override { return pose_graph_; }

#if 0
  /// @brief Start the path tracker along the path specified by chain
  void startControlLoop(pose_graph::LocalizationChain& chain);

  /// @brief Stop the path tracker
  void stopPathTracker(void);

  /// @brief Stop the path tracker controller and start the hover controller
  bool startHover(const asrl::planning::PathType& path);

  /// @brief Stop both the path tracker controller and the hover controller
  void stopAllControl();

  /// @brief Stop the hover controller and start the path tracker
  bool startFollow(const asrl::planning::PathType& path);

  /// @brief Stop both the path tracker controller and the hover controller
  void stopControl();

  /// @brief Set the path tracker. Separate from the constructor because not
  /// everything needs the PT. e.g. Lancaster.
  void setPathTracker(std::shared_ptr<asrl::path_tracker::Base> path_tracker);

  /// @brief Set the hover controller. Separate from the constructor because not
  /// everything needs the HC. e.g. Grizzly.
  void setHoverController(
      std::shared_ptr<asrl::path_tracker::Base> hover_controller);

  /// @brief Set the hover controller. Separate from the constructor because not
  /// everything needs the GC. e.g. Grizzly.
  void setGimbalController(
      std::shared_ptr<asrl::path_tracker::Base> gimbal_controller);
#endif

  /** brief Run the pipeline on the data */
  virtual void runPipeline(QueryCachePtr query_data);

#if 0
  /// @brief Should this be moved out of tactic? Into the assemblies?
  void visualizeInliers(std::shared_ptr<const QueryCache> product,
                        VertexId vertex_id, std::string frame_title);
#endif
  /** \brief localization frame to privileged map */
  std::shared_ptr<ConverterAssembly> getDataConverter() const {
    return converter_;
  }
  /** \brief every-frame to keyframe vo. */
  std::shared_ptr<QuickVoAssembly> getQuickVo() const { return quick_vo_; }

  /** \brief keyframe sliding window vo. */
  std::shared_ptr<RefinedVoAssembly> getRefinedVo() const {
    return refined_vo_;
  }
#if false
  /** \brief localization frame to privileged map. */
  std::shared_ptr<LocalizerAssembly> getLocalizer() const {
    return localizer_;
  }
#endif

#if 0
  /// @brief Create a new live vertex (when a new keyframe has been detected)
  VertexId createLiveVertex();

  /// @return true if the frame exceeded the new keyframe conditions
  bool needNewVertex(const QueryCache& frame_data, const MapCache&) const;

  /// @return true if localization has not failed (e.g., gone more than 20 m on
  /// VO)
  bool isLocalizationOk(const QueryCache& frame_data,
                        const MapCache& map_data) const;
#endif

  /** \brief Add a new vertex (keyframe) not connected to anything. */
  VertexId addDanglingVertex(const vtr_messages::msg::TimeStamp& stamp);

  /** \brief Add a new vertex (keyframe) connected to the last one. */
  VertexId addConnectedVertex(const vtr_messages::msg::TimeStamp& stamp,
                              const EdgeTransform& T_q_m);

  //////////////////////////////////////////////////////////////////////////////
  // Simple helpers

  /** \return The current (live) vertex id */
  const VertexId& currentVertexID() const override {
    return current_vertex_id_;
  }
#if false
  /**
   * \return The id of the closest vertex in privileged path to the current
   * vertex.
   */
  virtual const VertexId& closestVertexID() const {
    return chain_.trunkVertexId();
  }
#endif
  /** \param[in] T_s_v extrinsic calibration (vehicle to sensor). */
  void setTSensorVehicle(EdgeTransform T_s_v) { T_sensor_vehicle_ = T_s_v; }
  /** \return extrinsic calibration (vehicle to sensor). */
  EdgeTransform TSensorVehicle() { return T_sensor_vehicle_; }
#if 0
  /// @brief make the next frame a standalone vertex/keyframe
  ///        this indicates the start of an experience, and results in a break
  ///        in vo
  void setFirstFrame(bool flag) { first_frame_ = flag; }
#endif
#if false
  void setPublisher(PublisherInterface* publisher) { publisher_ = publisher; }

  void updateLocalization(QueryCachePtr q_data, MapCachePtr m_data);

  inline void updatePersistentLocalization(const VertexId& v,
                                           const EdgeTransform& T) {
    if (T.covarianceSet()) {
      persistentLocalization_.v = v;
      persistentLocalization_.T = T;
      persistentLocalization_.localized = true;
    } else {
      LOG(WARNING) << "Attempted to set persistent loc without a covariance!";
    }
#if 0
    if (publisher_ != nullptr) {
      publisher_->publishRobot(persistentLocalization_,
                               chain_.trunkSequenceId(), targetLocalization_);
    }
#endif
  }
#endif
#if 0
  inline void updateTargetLocalization(const VertexId& v,
                                       const EdgeTransform& T) {
    if (T.covarianceSet()) {
      targetLocalization_.v = v;
      targetLocalization_.T = T;
      targetLocalization_.localized = true;
    } else {
      LOG(WARNING) << "Attempted to set target loc without a covariance!";
    }
  }

#endif
  /** \brief accessor for the tactic configuration. */
  const TacticConfig& config() { return config_; }
#if false
  /** \brief Get a reference to the pipeline */
  std::shared_ptr<BasePipeline> pipeline(void) { return pipeline_; }

  pose_graph::LocalizationChain chain_;

#if 0
  /// @brief Path tracker base pointer
  std::shared_ptr<asrl::path_tracker::Base> path_tracker_;
  std::shared_ptr<asrl::path_tracker::Base> hover_controller_;
  std::shared_ptr<asrl::path_tracker::Base> gimbal_controller_;
#endif

#endif
 protected:
  /** \brief add initial data to the cache */
  void setupCaches(QueryCachePtr query_data, MapCachePtr map_data);

  /** \brief setup and run the pipeline processData(); */
  void processData(QueryCachePtr query_data, MapCachePtr map_data);

  virtual void wait(void) {
    if (keyframe_thread_future_.valid()) keyframe_thread_future_.wait();
  };

  TacticConfig config_;
  // first frame of VO
  bool first_frame_;
  // the map has been initialized
  int map_status_;  // enum type MapStatus

  /**
   * \brief the converter assembly that does image framing and other
   * pre-processing (image 2 frame)
   */
  std::shared_ptr<ConverterAssembly> converter_;
  /** \brief the navigation assembly that does quick vo (frame 2 keyframe) */
  std::shared_ptr<QuickVoAssembly> quick_vo_;
  /** \brief the navigation assembly that does refined vo (sliding window) */
  std::shared_ptr<RefinedVoAssembly> refined_vo_;
  /** \brief the navigation assembly that does localization (frame 2 map). */
  std::shared_ptr<LocalizerAssembly> localizer_;

  /// Graph stuff
  std::shared_ptr<Graph> pose_graph_;

  /** \brief Localization against the map, that persists across goals. */
  Localization persistentLocalization_;
  /** \brief Localization against a target for merging. */
  Localization targetLocalization_;

  std::shared_ptr<std::mutex> steam_mutex_ptr_;

  std::recursive_timed_mutex pipeline_mutex_;

  std::future<void> keyframe_thread_future_;

  std::shared_ptr<BasePipeline> pipeline_;

  EdgeTransform T_sensor_vehicle_;

  VertexId current_vertex_id_;
#if false
  PublisherInterface* publisher_;

  // memory management
  std::shared_ptr<MapMemoryManager> map_memory_manager_;
  std::shared_ptr<LiveMemoryManager> live_memory_manager_;
#endif
};

}  // namespace navigation
}  // namespace vtr
