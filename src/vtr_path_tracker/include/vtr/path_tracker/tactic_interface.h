#pragma once


#include <mutex>

#include <asrl/pose_graph/id/VertexId.hpp>
#include <asrl/common/timing/TimeUtils.hpp>
#include <asrl/pose_graph/path/LocalizationChain.hpp>

#include <lgmath.hpp>
#include <steam/trajectory/SteamTrajInterface.hpp>
#include <vtr/path_tracker/base.h>  /* for typedefs */

namespace vtr {
namespace path_tracker {


/**
 * @brief Class for storing information about the pose from VO using STEAM if it is avaiable..
 */
class VisionPose
{
public:

  // Default Constructor.
  VisionPose() {}

  /**
   * @brief updateLeaf: Update the pose used in the path tracker using a contant transformation
   * @param trunk_seq_id: trunk sequence id
   * @param T_leaf_trunk:
   * @param leaf_stamp: time-stamp corresponding to when the frame used to compute T_leaf_trunk was taken
   * @param live_vid: the live vertex id (last key-frame in the live run)
   */
  void updateLeaf(const Chain & chain,
                  const Stamp leaf_stamp,
                  const Vid live_vid) {

    std::lock_guard<std::mutex> lock(vo_update_mutex_);
    vo_update_.trunk_seq_id = chain.trunkSequenceId();
    vo_update_.T_leaf_trunk = chain.T_leaf_trunk();
    vo_update_.leaf_stamp   = leaf_stamp;
    vo_update_.live_vid     = live_vid;
    vo_update_.traj_valid   = false;
    is_updated_ = true;
  }

  ///< \brief report the difference between two std::chrono::time_point (aka Stamp) in seconds
  /// Accurate to one microsecond.
  float dtSecs(Stamp start, Stamp end)
  {
    return (std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() * 1.e-6);
  }

  /**
   * @brief updateLeaf: Method for updating the pose that uses a STEAM trajectory
   * @param trunk_seq_id: trunk sequence ID
   * @param T_petiole_trunk
   * @param trajectory: Steam trajectory from the petiole
   * @param T_leaf_petiole_cov: 6x6 covariance of T_leaf_petiole
   */
  void updateLeaf(const Chain & chain,
                  const steam::se3::SteamTrajInterface & trajectory,
                  const Vid live_vid,
                  const uint64_t image_stamp) {

    // Update fields
    std::lock_guard<std::mutex> lock(vo_update_mutex_);
    vo_update_.trunk_seq_id       = chain.trunkSequenceId();
    vo_update_.T_petiole_trunk    = chain.T_petiole_trunk();
    vo_update_.T_leaf_trunk       = chain.T_leaf_trunk();
    vo_update_.trajectory         = trajectory;
    vo_update_.T_leaf_petiole_cov = chain.T_leaf_petiole().cov();
    vo_update_.live_vid           = live_vid;
    vo_update_.leaf_stamp         = ::asrl::common::timing::toChrono(image_stamp);
    vo_update_.traj_valid         = true;
    is_updated_ = true;
    return;
  }


  /**
   * @brief updateFixedPose: Update the fixed pose. Call at the beginning of each controlStep
   *
   * The fixed pose is the pose used by the getters to make sure only one
   * (pose,time-stamp) pair is used at each control step.
   *
   * @param query_time. The time we want the pose to be valid for.
   */
  bool updateFixedPose(::asrl::common::timing::time_point query_time_point) {


    // return immediately if we haven't received any pose updates.
    if (!is_updated_)
      return false;

    // Convert to STEAM time
    int64_t stamp = ::asrl::common::timing::toUnix(query_time_point);
    steam::Time query_time = steam::Time(stamp);

    std::lock_guard<std::mutex> lock(vo_update_mutex_);

    // Extrapolate if we have a steam trajectory, otherwise, use the most recent update from VO
    if (vo_update_.traj_valid) {
      // Extrapolate the pose
      TfCov T_leaf_petiole;
      T_leaf_petiole = vo_update_.trajectory.getInterpPoseEval(query_time)->evaluate();
      T_leaf_petiole.setCovariance(vo_update_.T_leaf_petiole_cov);

      // Update pose and time-stamp
      T_leaf_trunk_ = T_leaf_petiole * vo_update_.T_petiole_trunk;
      leaf_stamp_  = ::asrl::common::timing::toChrono(query_time.nanosecs());

      // Get the velocity.
      velocity_ = -1. * vo_update_.trajectory.getVelocity(query_time);

      // fill in the remaining fields
      live_vid_     = vo_update_.live_vid;
      trunk_seq_id_ = vo_update_.trunk_seq_id;
    } else {

      // Update velocity
      TfCov dT  = vo_update_.T_leaf_trunk * T_leaf_trunk_.inverse();
      double dt = dtSecs(vo_update_.leaf_stamp, leaf_stamp_);
      velocity_ = -1. * dT.vec() / dt;

      // Just use the latest VO update.
      T_leaf_trunk_ = vo_update_.T_leaf_trunk;
      leaf_stamp_   = vo_update_.leaf_stamp;
      live_vid_     = vo_update_.live_vid;
      trunk_seq_id_ = vo_update_.trunk_seq_id;
      LOG_EVERY_N(10, WARNING) << "Path tracker did not receive a valid STEAM trajectory from the tactic! Using the last pose from VO instead.";
    }

    return true;
  }

  inline void reset() { is_updated_ = false; }

  /// return T_leaf_trunk
  inline TfCov T_leaf_trunk() const { return T_leaf_trunk_;}

  /// return the trunk sequence ID
  inline unsigned trunkSeqId() const { return trunk_seq_id_; }

  /// return the live vertex ID
  inline Vid liveVertexId() const { return live_vid_; }

  /// Return the time-stamp of the leaf
  inline Stamp leafStamp() const { return leaf_stamp_; }

  /// Return the time-stamp of the image used for the last VO update
  inline Stamp voLeafStamp() const { return vo_update_.leaf_stamp; }

  /// Return t_leaf_trunk from VO
  inline TfCov voT_leaf_trunk() const { return vo_update_.T_leaf_trunk; }

  /// Return trunk vertex id from VO
  inline unsigned votrunkSeqId() const { return vo_update_.trunk_seq_id; }

  /// Return true if the pose estimate is valid
  inline bool isUpdated() const { return is_updated_; }

  /// Return the instantanious velocity estimate. Only works with non-steam updates.
  inline Eigen::Matrix<double, 6, 1> velocity() const { return velocity_; }

private:

  typedef struct VisionPoseState {
    // Member variables used when we don't have a STEAM trajectory
    unsigned trunk_seq_id; ///< vertex ID for the trunk
    TfCov T_leaf_trunk; ///< The vision-based pose
    Stamp leaf_stamp; ///< The pose time-stamp
    Vid live_vid; ///< The most recent vertex ID of the live run
    Eigen::Matrix<double,6,1> velocity = Eigen::Matrix<double,6,1>::Zero();

    // Member variables used when we have a STEAM trajectory
    steam::se3::SteamTrajInterface trajectory; ///< Steam trajectory from the petiole.
    TfCov T_petiole_trunk;
    Eigen::Matrix<double,6,6> T_leaf_petiole_cov; ///< The 6x6 covariance of T_leaf_petiole
    bool traj_valid = false;  ///< true if the last updated was done using the trajectory method.

  } VisionPoseState;

  std::mutex vo_update_mutex_; ///< mutex since the tactic and path tracker both access this class
  VisionPoseState vo_update_; ///< updated by the tactic whenever a new localization estimate is available.

  // Member variables accessed by getters
  unsigned trunk_seq_id_; ///< vertex ID for the trunk
  TfCov T_leaf_trunk_; ///< The vision-based pose
  Stamp leaf_stamp_; ///< The pose time-stamp
  Vid live_vid_; ///< The most recent vertex ID of the live run
  bool is_updated_ = false; ///< True if we have received an update from VO.

  Eigen::Matrix<double, 6, 1> velocity_;///< The instantanious velocity from finite difference between the last two pose estimates

};

}}
