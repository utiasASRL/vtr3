#pragma once

#include <memory>

#include <steam.hpp>

#include <vtr_mission_planning/state_machine_interface.hpp>
#include <vtr_pose_graph/evaluator/common.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_pose_graph/path/localization_chain.hpp>

#include <vtr_messages/msg/time_stamp.hpp>

// vision related stuff
#include <vtr_messages/msg/h_vec3.hpp>
#include <vtr_messages/msg/keypoint.hpp>
#include <vtr_messages/msg/match.hpp>
#include <vtr_messages/msg/velocity.hpp>
#include <vtr_vision/messages/bridge.hpp>
#include <vtr_vision/types.hpp>

namespace vtr {
namespace tactic {

/// pose graph structures
using Graph = pose_graph::RCGraph;
using RunId = pose_graph::RCRun::IdType;
using RunIdSet = std::set<RunId>;
using Vertex = pose_graph::RCVertex;
using VertexId = pose_graph::VertexId;
using EdgeId = pose_graph::EdgeId;
using EdgeTransform = pose_graph::RCEdge::TransformType;
/**
 * \brief Privileged edge mask.
 * \details This is used to create a subgraph on privileged edges.
 */
using PrivilegedEvaluator = pose_graph::eval::Mask::PrivilegedDirect<Graph>;
/** \brief Temporal edge mask */
using TemporalEvaluator = pose_graph::eval::Mask::SimpleTemporalDirect<Graph>;

using LocalizationChain = pose_graph::LocalizationChain;

/// mission planning
using PipelineMode = mission_planning::PipelineMode;
using Localization = mission_planning::Localization;

/// vertex creation status
/** \brief the vertex creation test result */
enum class KeyframeTestResult : int {
  CREATE_VERTEX = 0,
  CREATE_CANDIDATE = 1,
  FAILURE = 2,
  DO_NOTHING = 3
};

/// image related structures
/** \brief Landmarks in a single privileged frame */
struct LandmarkFrame {
  /** \brief Currently observed landmarks, for each rig */
  vision::RigLandmarks landmarks;
  /** \brief corresponding landmark observations */
  vision::RigObservations observations;
};
using LandmarkFrames = std::vector<LandmarkFrame>;

/** \brief collection of pointers to observations and their origins. */
struct LandmarkObs {
  std::vector<vtr_messages::msg::Keypoint> keypoints;
  std::vector<float> precisions;
  std::vector<std::vector<float>> covariances;
  vtr_messages::msg::Match origin_ref;
};

/**
 * \brief collection of pointers to landmarks and their associated steam
 * containers.
 */
struct LandmarkInfo {
  vtr_messages::msg::HVec3 point;
  std::vector<float> covariance;
  uint8_t descriptor;
  uint32_t num_vo_observations;
  steam::se3::LandmarkStateVar::Ptr steam_lm;
  std::vector<LandmarkObs> observations;
  bool valid;
};
using LandmarkMap = std::unordered_map<vision::LandmarkId, LandmarkInfo>;

/** \brief A steam TransformStateVar Wrapper, keeps track of locking */
class SteamPose {
 public:
  /** \brief Default constructor */
  SteamPose() = default;

  /**
   * \brief Constructor
   * \param T The transformation associated with this pose.
   * \param lock_flag Whether this pose should be locked or not.
   */
  SteamPose(EdgeTransform T, bool lock_flag) : lock(lock_flag) {
    tf_state_var.reset(new steam::se3::TransformStateVar(T));
    tf_state_var->setLock(lock);
    tf_state_eval.reset(new steam::se3::TransformStateEvaluator(tf_state_var));
  }

  /** \brief Sets the transformation. */
  void setTransform(const EdgeTransform &transform) {
    tf_state_var.reset(new steam::se3::TransformStateVar(transform));
    tf_state_var->setLock(lock);
    tf_state_eval.reset(new steam::se3::TransformStateEvaluator(tf_state_var));
  }

  void setVelocity(Eigen::Matrix<double, 6, 1> &vel) {
    velocity.reset(new steam::VectorSpaceStateVar(vel));
    velocity->setLock(lock);
  }
  /** \brief Sets the lock */
  void setLock(bool lock_flag) {
    lock = lock_flag;
    tf_state_var->setLock(lock);
  }

  bool isLocked() { return lock; }
  /** \brief The steam state variable. */
  steam::se3::TransformStateVar::Ptr tf_state_var;
  steam::se3::TransformStateEvaluator::Ptr tf_state_eval;
  steam::Time time;
  steam::VectorSpaceStateVar::Ptr velocity;
  std::shared_ptr<vtr_messages::msg::Velocity> proto_velocity;

 private:
  /** \brief The lock flag. */
  bool lock;
};
using SteamPoseMap = std::map<pose_graph::VertexId, SteamPose>;

using SensorVehicleTransformMap =
    std::map<pose_graph::VertexId, lgmath::se3::TransformationWithCovariance>;

using MigrationMap = std::unordered_map<vision::LandmarkId, int>;

// experience recognition
using ScoredRids = std::multimap<float, RunId>;
using ScoredRid = std::pair<float, RunId>;
/** \brief the BoW cosine distance of the query to each run in the map. */
using ExperienceDifferences = std::map<RunId, float>;
/** \brief a BoW cosine distance from the query to the run: <run, distance> */
using ExperienceDifference = std::pair<RunId, float>;
}  // namespace tactic
}  // namespace vtr