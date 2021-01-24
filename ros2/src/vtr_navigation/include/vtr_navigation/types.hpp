#pragma once

#include <steam.hpp>
#include <vtr_mission_planning/state_machine_interface.hpp>
#include <vtr_vision/types.hpp>

#include <vtr_messages/msg/h_vec3.hpp>
#include <vtr_messages/msg/keypoint.hpp>
#include <vtr_messages/msg/match.hpp>
#include <vtr_messages/msg/rig_landmarks.hpp>
#include <vtr_messages/msg/velocity.hpp>

// Pose Graph
#if false
#include <robochunk_msgs/Velocity.pb.h>
#endif
#include <vtr_pose_graph/evaluator/common.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_edge.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_run.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_vertex.hpp>
// #include <lgmath/se3/TransformationWithCovariance.hpp>

namespace vtr {
namespace navigation {

// Pose graph
using Graph = pose_graph::RCGraph;
#if 0
using GraphBase = pose_graph::RCGraphBase ;
#endif
using Vertex = pose_graph::RCVertex;
using VertexId = pose_graph::RCVertex::IdType;
#if 0
using SimpleVertexId= pose_graph::RCVertex::SimpleIdType ;
#endif
using EdgeId = pose_graph::RCEdge::IdType;
using RunId = pose_graph::RCRun::IdType;
using EdgeTransform = pose_graph::RCEdge::TransformType;

/**
 * \brief Privileged Edge mask.
 * \details This is used to create a subgraph on priveleged edges.
 */
using PrivilegedEvaluator
  = pose_graph::eval::Mask::PrivilegedDirect<pose_graph::RCGraph>;

/** \brief Privileged Edge mask Pointer. */
using PrivilegedEvaluatorPtr = PrivilegedEvaluator::Ptr;
#if false
/**
 * \brief Spatial edge mask
 * \details This is used to create a subgraph on spatial edges.
 */
using SpatialEvaluator =
    pose_graph::eval::Mask::SpatialDirect<pose_graph::RCGraph>;

/** \brief Privileged Edge mask Pointer. */
using SpatialEvaluatorPtr = SpatialEvaluator::Ptr;
#endif
/** \brief Temporal edge mask */
using TemporalEvaluator = pose_graph::eval::Mask::SimpleTemporalDirect<Graph>;

/** \brief Temporal edge mask pointer */
using TemporalEvaluatorPtr = TemporalEvaluator::Ptr;


// Experience Recognition
/** \brief Experience scores, indexed by run id */
using ScoredRids = std::multimap<float, RunId>;
using ScoredRid = std::pair<float, RunId>;
/** \brief contain the BoW cosine distance of the query to each run in the map. */
using ExperienceDifferences = std::map<RunId, float>;
/** \brief A single BoW cosine distance from the query to the run: <run, distance> */
using ExperienceDifference = std::pair<RunId, float>;

/// A set of experiences, specified by run id, used to collect experiences we
/// should use for localization
using RunIdSet= std::set<RunId> ;
#if false
/// A list of landmarks, ordered by utility for matching
using LandmarkIdVec= std::vector<vtr::vision::LandmarkId> ;
#endif
////////////////////////////////////////////////////////////////////////////////
// Map representations
using Localization = mission_planning::Localization;

#if 0
/// @brief Pointer into the graph to a feature keypoint or landmark.
struct GraphFeature {
  unsigned feature;
  unsigned camera;
  SimpleVertexId vertex;
};
#endif

/** \brief Landmarks in a single privileged frame */
struct LandmarkFrame {
  /** \brief Currently observed landmarks, for each rig */
  vtr::vision::RigLandmarks landmarks;
  /** \brief corresponding landmark observations */
  vtr::vision::RigObservations observations;
};
typedef std::vector<LandmarkFrame> LandmarkFrames;

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

#if 0
/** \brief Maps VertexIDs to steam poses */
typedef std::map<VertexId, SteamPose> SteamPoseMap;

/** \brief Maps VertexIDs to sensor->vehicle transforms */
typedef std::map<pose_graph::VertexId,
                 lgmath::se3::TransformationWithCovariance>
    SensorVehicleTransformMap;

/// \brief Maps LandmarkIds landmarks/observations.
typedef std::unordered_map<vtr::vision::LandmarkId, LandmarkInfo> LandmarkMap;
typedef std::unordered_map<vtr::vision::LandmarkId, int> MigrationMap;
#endif

typedef Eigen::Matrix<double, 3, Eigen::Dynamic> EigenMatrix3Dynamic;
#if 0
typedef Eigen::Matrix<double, 2, Eigen::Dynamic> EigenMatrix2Dynamic;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
    EigenMatrixDynamic;

/// @brief Temporary ransac data, used to keep track of multi channel results.
struct RansacData {
  EigenMatrixDynamic query_points;
  EigenMatrixDynamic map_points;

  std::map<int, int> query_channel_offset;
  std::map<int, int> map_channel_offset;

  vtr::vision::SimpleMatches inliers;
};
#endif

/** \brief the vertex creation test result */
typedef enum : int {
  CREATE_VERTEX = 0,
  CREATE_CANDIDATE = 1,
  FAILURE = 2,
  DO_NOTHING = 3
} VertexTestResult;

/** \brief the map initializion status for monocular VO */
typedef enum : int {
  MAP_NEW = 0,         // the map is not initialized and must be initialized
  MAP_EXTEND = 1,      // the map should be extended from the last run
  MAP_INITIALIZED = 2  // the map is initialized
} MapStatus;

#if 0
//@ GPS Position: latitude, longitude, altitude
//@ Odom Position: x, y, z
typedef Eigen::Vector3d Position;
//@ IMU or Odom Orientation: x, y, z, w
typedef Eigen::Quaterniond Orientation;
//@ IMU Angular Velocity (roll, pitch, yaw)
typedef Eigen::Vector3d AngularVelocity;
//@ IMU Linear Acceleration (x, y, z)
typedef Eigen::Vector3d LinearAcceleration;

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Terrain Assessment
////////////////////////////////////////////////////////////////////////////////
typedef std::vector<asrl::pose_graph::VertexId> VertexIdVector;
typedef std::map<asrl::pose_graph::VertexId, lgmath::se3::Transformation>
    TCache;
#endif

}  // namespace navigation
}  // namespace vtr