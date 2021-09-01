// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * \file tactic_interface.hpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <mutex>

#include <lgmath.hpp>
#include <steam.hpp>

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_pose_graph/id/graph_id.hpp>
#include <vtr_pose_graph/path/localization_chain.hpp>

#include <vtr_path_tracker/base.hpp> /* for typedefs */

namespace vtr {
namespace path_tracker {

/**
 * \brief Class for storing information about the pose from VO using STEAM if
 * it is available.
 */
class VisionPose {
 public:
  /** \brief Default Constructor */
  VisionPose() = default;

  /**
   * \brief Report the difference between two std::chrono::time_point (aka
   * Stamp) in seconds
   * \note Accurate to one microsecond.
   */
  static double dtSecs(Stamp start, Stamp end) {
    return (std::chrono::duration_cast<std::chrono::microseconds>(end - start)
                .count() *
            1.e-6);
  }

  /**
   * \brief Update the pose used in the path tracker using a constant
   * transformation
   * \param chain
   * \param leaf_stamp time-stamp corresponding to when the frame used to
   * compute T_leaf_trunk was taken
   * \param live_vid the live vertex id (last key-frame in the live run)
   */
  void updateLeaf(const Chain::ConstPtr &chain, const Stamp leaf_stamp,
                  const Vid live_vid) {
    std::lock_guard<std::mutex> lock(vo_update_mutex_);
    vo_update_.trunk_seq_id = chain->trunkSequenceId();
    vo_update_.T_leaf_trunk = chain->T_leaf_trunk();
    vo_update_.leaf_stamp = leaf_stamp;
    vo_update_.live_vid = live_vid;
    vo_update_.traj_valid = false;
    is_updated_ = true;

    CLOG(DEBUG, "path_tracker")
        << "Path tracker updating leaf and trajectory - trunk_seq_id: "
        << vo_update_.trunk_seq_id << ", live_vid: " << vo_update_.live_vid
        << ", leaf_stamp: "
        << std::chrono::duration_cast<std::chrono::nanoseconds>(
               vo_update_.leaf_stamp.time_since_epoch())
               .count()
        << std::endl
        << "T_petiole_trunk: " << vo_update_.T_petiole_trunk.vec().transpose()
        << std::endl
        << "T_leaf_trunk: " << vo_update_.T_leaf_trunk.vec().transpose();
  }

  /**
   * \brief Method for updating the pose that uses a STEAM trajectory
   * \param chain
   * \param trajectory: Steam trajectory from the petiole
   * \param live_vid: the live vertex id (last key-frame in the live run)
   * \param image_stamp
   */
  void updateLeaf(const Chain::ConstPtr &chain,
                  const steam::se3::SteamTrajInterface &trajectory,
                  const Vid live_vid, const uint64_t image_stamp) {
    // Update fields
    std::lock_guard<std::mutex> lock(vo_update_mutex_);
    vo_update_.trunk_seq_id = chain->trunkSequenceId();
    vo_update_.T_petiole_trunk = chain->T_petiole_trunk();
    vo_update_.T_leaf_trunk = chain->T_leaf_trunk();
    vo_update_.trajectory = trajectory;

    // manually copying this 6x6 matrix due to weird Eigen bug giving segfaults
    // with default copy
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 6; j++)
        vo_update_.T_leaf_petiole_cov(i, j) =
            chain->T_leaf_petiole().cov()(i, j);

    vo_update_.live_vid = live_vid;
    vo_update_.leaf_stamp = common::timing::toChrono(image_stamp);
    vo_update_.traj_valid = true;
    is_updated_ = true;

    CLOG(DEBUG, "path_tracker")
        << "Path tracker updating leaf and trajectory - trunk_seq_id: "
        << vo_update_.trunk_seq_id << ", live_vid: " << vo_update_.live_vid
        << ", leaf_stamp: "
        << std::chrono::duration_cast<std::chrono::nanoseconds>(
               vo_update_.leaf_stamp.time_since_epoch())
               .count()
        << std::endl
        << "T_petiole_trunk: " << vo_update_.T_petiole_trunk.vec().transpose()
        << std::endl
        << "T_leaf_trunk: " << vo_update_.T_leaf_trunk.vec().transpose();
  }

  /**
   * \brief updateFixedPose: Update the fixed pose. Call at the beginning of
   * each controlStep The fixed pose is the pose used by the getters to make
   * sure only one (pose,time-stamp) pair is used at each control step.
   * \param query_time_point the time we want the pose to be valid for
   */
  bool updateFixedPose(common::timing::time_point query_time_point) {
    // return immediately if we haven't received any pose updates.
    if (!is_updated_) return false;

    // Convert to STEAM time
    int64_t stamp = common::timing::toUnix(query_time_point);
    auto query_time = steam::Time(stamp);

    std::lock_guard<std::mutex> lock(vo_update_mutex_);

    // Extrapolate if we have a steam trajectory, otherwise, use the most recent
    // update from VO
    if (vo_update_.traj_valid) {
      // Extrapolate the pose
      TfCov T_leaf_petiole;
      T_leaf_petiole =
          vo_update_.trajectory.getInterpPoseEval(query_time)->evaluate();
      T_leaf_petiole.setCovariance(vo_update_.T_leaf_petiole_cov);

      // Update pose and time-stamp
      T_leaf_trunk_ = T_leaf_petiole * vo_update_.T_petiole_trunk;
      leaf_stamp_ = common::timing::toChrono(query_time.nanosecs());

      // Get the velocity.
      velocity_ = -1. * vo_update_.trajectory.getVelocity(query_time);

      // fill in the remaining fields
      live_vid_ = vo_update_.live_vid;
      trunk_seq_id_ = vo_update_.trunk_seq_id;
    } else {
      // Update velocity
      TfCov dT = vo_update_.T_leaf_trunk * T_leaf_trunk_.inverse();
      double dt = dtSecs(vo_update_.leaf_stamp, leaf_stamp_);
      velocity_ = -1. * dT.vec() / dt;

      // Just use the latest VO update.
      T_leaf_trunk_ = vo_update_.T_leaf_trunk;
      leaf_stamp_ = vo_update_.leaf_stamp;
      live_vid_ = vo_update_.live_vid;
      trunk_seq_id_ = vo_update_.trunk_seq_id;
      CLOG_EVERY_N(10, WARNING, "path_tracker")
          << "Path tracker did not receive a valid STEAM trajectory from the "
             "tactic! Using the last pose from VO instead.";
    }

    CLOG(DEBUG, "path_tracker")
        << "Fixed pose update - trajectory valid: " << vo_update_.traj_valid
        << ", leaf_stamp_: "
        << std::chrono::duration_cast<std::chrono::nanoseconds>(
               leaf_stamp_.time_since_epoch())
               .count()
        << ", live_vid_: " << live_vid_ << ", trunk_sid_: " << trunk_seq_id_
        << std::endl
        << "T_leaf_trunk: " << T_leaf_trunk_.vec().transpose() << std::endl
        << "velocity: " << velocity_.transpose();
    return true;
  }

  inline void reset() { is_updated_ = false; }

  /// return T_leaf_trunk
  inline TfCov T_leaf_trunk() const { return T_leaf_trunk_; }

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

  /// Return the instantaneous velocity estimate. Only works with non-steam
  /// updates.
  inline Eigen::Matrix<double, 6, 1> velocity() const { return velocity_; }

 private:
  typedef struct VisionPoseState {
    // Member variables used when we don't have a STEAM trajectory
    /** \brief Vertex ID for the trunk */
    unsigned trunk_seq_id;

    /** \brief The vision-based pose */
    TfCov T_leaf_trunk;

    /** \brief The pose time-stamp */
    Stamp leaf_stamp;

    /** \brief The most recent vertex ID of the live run */
    Vid live_vid;

    Eigen::Matrix<double, 6, 1> velocity = Eigen::Matrix<double, 6, 1>::Zero();

    // Member variables used when we have a STEAM trajectory
    /** \brief Steam trajectory from the petiole */
    steam::se3::SteamTrajInterface trajectory;

    TfCov T_petiole_trunk;

    /** \brief The 6x6 covariance of T_leaf_petiole */
    Eigen::Matrix<double, 6, 6> T_leaf_petiole_cov;

    /** \brief true if the last updated was done using the trajectory method */
    bool traj_valid = false;

  } VisionPoseState;

  /** \brief  */
  std::mutex vo_update_mutex_;  ///< mutex since the tactic and path tracker
                                ///< both access this class

  /** \brief  */
  VisionPoseState vo_update_;  ///< updated by the tactic whenever a new
                               ///< localization estimate is available.

  // Member variables accessed by getters
  /** \brief Vertex ID for the trunk */
  unsigned trunk_seq_id_;

  /** \brief The vision-based pose */
  TfCov T_leaf_trunk_;

  /** \brief The pose time-stamp */
  Stamp leaf_stamp_;

  /** \brief The most recent vertex ID of the live run */
  Vid live_vid_;

  /** \brief True if we have received an update from VO */
  bool is_updated_ = false;

  /** \brief The instantaneous velocity from finite difference between the last
   * two pose estimates */
  Eigen::Matrix<double, 6, 1> velocity_;
};

}  // namespace path_tracker
}  // namespace vtr
