#pragma once

#include <steam.hpp>
// Pose Graph
#include <robochunk_msgs/Velocity.pb.h>
#include <asrl/pose_graph/evaluator/Common.hpp>
#include <asrl/pose_graph/index/RCEdge.hpp>
#include <asrl/pose_graph/index/RCGraph.hpp>
#include <asrl/pose_graph/index/RCRun.hpp>
#include <asrl/pose_graph/index/RCVertex.hpp>
// #include <lgmath/se3/TransformationWithCovariance.hpp>
// Vision
#include <asrl/vision/Types.hpp>
// Planning
#include <vtr/navigation/tactics/state_machine_interface.h>
// #include <asrl/planning/StateMachineInterface.hpp>

namespace vtr {
namespace navigation {

// Pose graph
typedef asrl::pose_graph::RCGraph Graph;
#if 0
typedef asrl::pose_graph::RCGraphBase GraphBase;
#endif
typedef asrl::pose_graph::RCVertex Vertex;
typedef asrl::pose_graph::RCVertex::IdType VertexId;
#if 0
typedef asrl::pose_graph::RCVertex::SimpleIdType SimpleVertexId;
#endif
typedef asrl::pose_graph::RCEdge::IdType EdgeId;
#if 0
typedef asrl::pose_graph::RCRun::IdType RunId;
#endif
typedef asrl::pose_graph::RCEdge::TransformType EdgeTransform;

/** \brief Privileged Edge mask. This is used to create a subgraph on priveleged
 * edges.
 */
typedef asrl::pose_graph::Eval::Mask::PrivilegedDirect<
    asrl::pose_graph::RCGraph>
    PrivilegedEvaluator;

/** \brief Privileged Edge mask Pointer.
 */
typedef PrivilegedEvaluator::Ptr PrivilegedEvaluatorPtr;

/** \brief Privileged Edge mask. This is used to create a subgraph on privileged
 * edges.
 */
typedef asrl::pose_graph::Eval::Mask::SpatialDirect<asrl::pose_graph::RCGraph>
    SpatialEvaluator;

/** \brief Privileged Edge mask Pointer.
 */
typedef SpatialEvaluator::Ptr SpatialEvaluatorPtr;

/** \brief Privileged Edge mask. This is used to create a subgraph on privileged
 * edges.
 */
typedef asrl::pose_graph::Eval::Mask::SimpleTemporalDirect<Graph>
    TemporalEvaluator;

/** \brief Privileged Edge mask Pointer.
 */
typedef TemporalEvaluator::Ptr TemporalEvaluatorPtr;

#if 0
////////////////////////////////////////////////////////////////////////////////
// Experience Recognition

/// Experience scores, indexed by run id,
typedef std::multimap<float, RunId> ScoredRids;
typedef std::pair<float, RunId> ScoredRid;
/// contain the BoW cosine distance of the query to each run in the map.
typedef std::map<RunId, float> ExperienceDifferences;
/// A single BoW cosine distance from the query to the run: <run, distance>
typedef std::pair<RunId, float> ExperienceDifference;
/// A set of experiences, specified by run id, used to collect experiences we
/// should use for localization
typedef std::set<RunId> RunIdSet;
/// A list of landmarks, ordered by utility for matching
typedef std::vector<asrl::vision::LandmarkId> LandmarkIdVec;

#endif
////////////////////////////////////////////////////////////////////////////////
// Map representations
using Localization = asrl::planning::Localization;

#if 0
/// @brief Pointer into the graph to a feature keypoint or landmark.
struct GraphFeature {
  unsigned feature;
  unsigned camera;
  SimpleVertexId vertex;
};
#endif

/** \brief Landmarks in a single privileged frame
 */
struct LandmarkFrame {
  /** \brief Currently observed landmarks, for each rig
   */
  asrl::vision::RigLandmarks landmarks;

  /** \brief corresponding landmark observations
   */
  asrl::vision::RigObservations observations;
};
typedef std::vector<LandmarkFrame> LandmarkFrames;

/** \brief collection of pointers to observations and their origins.
 */
struct LandmarkObs {
  std::vector<asrl::vision_msgs::Keypoint *> keypoints;
  std::vector<float *> precisions;
  std::vector<float *> covariances;
  asrl::vision_msgs::Match *origin_ref;
};

/** \brief collection of pointers to landmarks and their associated steam
 * containers.
 */
struct LandmarkInfo {
  robochunk::kinematic_msgs::HVec3 *point;
  google::protobuf::RepeatedField<float> *covariance;
  uint8_t *descriptor;
  uint32_t *num_vo_observations;
  steam::se3::LandmarkStateVar::Ptr steam_lm;
  std::vector<LandmarkObs> observations;
  bool *valid;
};

/** \brief A steam TransformStateVar Wrapper, keeps track of locking
 */
class SteamPose {
 public:
  /** \brief Default constructor
   */
  SteamPose() = default;

  /** \brief Constructor
   *
   * \param T The transformation associated with this pose.
   * \param lock_flag Whether this pose should be locked or not.
   */
  SteamPose(EdgeTransform T, bool lock_flag) : lock(lock_flag) {
    tf_state_var.reset(new steam::se3::TransformStateVar(T));
    tf_state_var->setLock(lock);
    tf_state_eval.reset(new steam::se3::TransformStateEvaluator(tf_state_var));
  }

  /** \brief Sets the transformation.
   */
  void setTransform(const EdgeTransform &transform) {
    tf_state_var.reset(new steam::se3::TransformStateVar(transform));
    tf_state_var->setLock(lock);
    tf_state_eval.reset(new steam::se3::TransformStateEvaluator(tf_state_var));
  }

  void setVelocity(Eigen::Matrix<double, 6, 1> &vel) {
    velocity.reset(new steam::VectorSpaceStateVar(vel));
    velocity->setLock(lock);
  }
  /** \brief Sets the lock
   */
  void setLock(bool lock_flag) {
    lock = lock_flag;
    tf_state_var->setLock(lock);
  }

  bool isLocked() { return lock; }
  /** \brief The steam state variable.
   */
  steam::se3::TransformStateVar::Ptr tf_state_var;
  steam::se3::TransformStateEvaluator::Ptr tf_state_eval;
  steam::Time time;
  steam::VectorSpaceStateVar::Ptr velocity;
  std::shared_ptr<robochunk::kinematic_msgs::Velocity> proto_velocity;

 private:
  /** \brief The lock flag.
   */
  bool lock;
};

#if 0
/// @brief Maps VertexIDs to steam poses
typedef std::map<VertexId, SteamPose> SteamPoseMap;

/// @brief Maps VertexIDs to sensor->vehicle transforms
typedef std::map<pose_graph::VertexId,
                 lgmath::se3::TransformationWithCovariance>
    SensorVehicleTransformMap;

/// @brief Maps LandmarkIds landmarks/observations.
typedef std::unordered_map<asrl::vision::LandmarkId, LandmarkInfo> LandmarkMap;
typedef std::unordered_map<asrl::vision::LandmarkId, int> MigrationMap;
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

  asrl::vision::SimpleMatches inliers;
};
#endif

/** \brief the vertex creation test result
 */
typedef enum : int {
  CREATE_VERTEX = 0,
  CREATE_CANDIDATE = 1,
  FAILURE = 2,
  DO_NOTHING = 3
} VertexTestResult;

/** \brief the map initializion status for monocular VO
 */
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
