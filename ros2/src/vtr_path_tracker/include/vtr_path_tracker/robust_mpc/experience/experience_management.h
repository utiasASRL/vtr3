#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

#include <vtr_path_tracker/robust_mpc/optimization/path_tracker_mpc_nominal_model.h>
#include <vtr_path_tracker/base.h>
#include <vtr_path_tracker/robust_mpc/mpc/mpc_types.h>

#include <vtr_common/rosutils/transformations.hpp>
#include <vtr_common/timing/time_utils.hpp>

#include <vtr_pose_graph/index/rc_graph/rc_run.hpp>

#include <vtr_messages/msg/experience.hpp>
#include <vtr_messages//msg/pt_status.hpp>
#if 0
#include <vtr_messages/msg/lgmath_conversions.hpp>
#include <vtr_messages/msg/PredStatus.pb.h>
#endif
#include <vtr_path_tracker/robust_mpc/experience/experience_management_base.h>

namespace vtr {
namespace path_tracker {

using Duration = common::timing::duration_ms;
using Clock = common::timing::clock;

using RosExperience = vtr_messages::msg::Experience;
#if 0
using path_tracker_msgs::PredStatus;
using path_tracker_msgs::GpPred;
#endif

class RCExperienceManagement : public ExperienceManagement {
  friend class ExperienceManagement;

 protected:

  struct ResultStream {
    static constexpr auto status = "control_status";
    static constexpr auto experience = "control_experience";
    static constexpr auto prediction = "control_prediction";
  };

  Clock clock_;
  MpcNominalModel nominal_model_;
  std::shared_ptr<Graph> graph_;
  double min_exp_age_ = 30.;

#if 0
  /**
   * @brief robochunk_to_experience_t: Convert a robochunk experience type to a MpcNominalModel::experience_t type
   * @param rc_experience: the robochunk experience.
   * @return the robochunk experience as an MpcNominalModel::experience_t
   *
   * TODO: This should be a member of the experience type. Overload the constructor?
   */
  MpcNominalModel::experience_t experience_tFromRobochunk(const RosExperience &rc_experience);

  /**
   * @brief RCExperienceManagement::experience_tToRobochunk Convert the important fields of an experience_t message to Robochunk type
   * @param experience: the MpcNominalModel::experince_t type that we wish to convert
   * @return a robochunk experience containing the info from the experience_t
   */
  RosExperience experience_tToRobochunk(MpcNominalModel::experience_t experience);

  /**
   * @brief RCExperienceManagement::loadSpatialExperiences: get all the experiences under the stream "/control/experience" at spatial neighbours of a vertex.
   * @param vertex: The vertex whos spatial neighbours we would like to get experiences from
   * @return a vector of all the experiences at spatial neighbours of a vertex
   */
  std::vector<MpcNominalModel::experience_t> loadSpatialExperience(const Vid vertex);


  /**
   * @brief RCExperienceManagement::enforceFifoBins
   * @param experience_list: List of experiences to be sorted into First-in-first-out (FIFO) bins.
   *
   * Given a list of experiences, assemble experiences into a list that enforces the
   * first-in-first-out logic implemented in the old path tracker in the storeExperienceInfo function.
   */
  vertexExperienceVec_t enforceFifoBins(vertexExperienceVec_t &new_experience_list);
#endif

 public:

  // Used for old experience management logic to determine if an experience is from the current run.
  /// \todo remove?
  rclcpp::Time start_of_current_trial_;

  /**
   * @brief RCExperienceManagement::setMinExpAge min age of experiences in seconds before they are used in the GP
   */
  void setMinExpAge(const double &min_exp_age) {
    min_exp_age_ = min_exp_age;
  }

  /**
   * @brief RCExperienceManagement::RCExperienceManagement Initialize the graph and call the constructor to the old ExperienceManagement
   * @param graph: shared pointer to the graph
   */
  RCExperienceManagement(const std::shared_ptr<Graph> &graph);

#if 0
  /**
   * @brief RCExperienceManagement::getGpDataFromExperimentExperience Get list of experiences for the GP
   *
   * - Get experiences from spatial neightbours of vertices in the list
   * - sort by store_time
   * - enforce first-in-first-out bins like the old PT.
   * - sort the experiences into bins by speed relative to v_km1
   * - Select points to include in the GP. Do this by iterating through a list of
   *   experiences sorted roughly by speed relative to v_km1. Choose points that
   *   are separated by (delta_distance + delta_speed) > 0.3 where delta_* is
   *   relative to experiences already in the GP.
   *
   * @param vertex_list: list of vertices to fetch experience for
   * @param speed_profile_vec: scheduled speed at each vertex
   * @param pathLengthByVertex: path length at each vertex
   * @param v_km1: Previous input from the solver
   * @return a vector of gp_data_t from the list of vertices
   */
  std::vector<MpcNominalModel::gp_data_t> getGpDataFromRCExperience(const std::vector<Vid> & vertex_list,
                                                                     const std::vector<float> & speed_profile_vec,
                                                                     const std::unordered_map<Vid, double> &pathLengthByVertex,
                                                                     float v_km1);

  /**
   * @brief getGpDataFromRCExperience
   * Get a list of experiences from Robochunk around a  vertex. This is a wrapper around the other getGpDataFromRCExperience that actually gets the experiences.
   * @param current_pose_num: Sequence ID of the first vertex
   * @param mpc_size: number of poses in the longest possible look-ahead window. Computed using computeLookahead
   * @param vertexIds: list of vertex IDs in the path.
   * @param pathLengthByVertex: path length at each vertex
   * @param v_km1: Previous input from the solver
   * @return
   */
  std::vector<MpcNominalModel::gp_data_t> getGpDataFromRCExperience(const int &current_pose_num,
                                                                     const int &mpc_size,
                                                                     const std::vector<Vid> & vertex_ids,
                                                                     const std::vector<double> &speed_schedule,
                                                                     const std::unordered_map<Vid, double> & pathLengthByVertex,
                                                                     const float &v_km1);

  /**
   @brief ExperienceManagement::log_pt_status General status logging for the PT
    Should be able to reconstruct most of what happened during a run from this info
 * @param log_vertex: the vertex to log to (used for the run id)
 * @param t_leaf_trunk_vo: t_leaf_trunk from VO
 * @param vo_stamp: time-stamp from the image used for VO
 * @param t_leaf_trunk_steam: t_leaf_trunk extrapolated using STEAM
 * @param steam_stamp: time-stamp that we extrapolated to using STEAM
 * @param trunk_vid: current vertex we are localizing to.
 * @param omega_cmd: Current commanded turn rate
 * @param v_cmd: Current commanded forward speed
 */
  void logPtStatus(const Vid &log_vertex,
                   const TfCov &t_leaf_trunk_vo,
                   const Stamp &vo_stamp,
                   const Vid &vo_trunk_vid,
                   const TfCov &t_leaf_trunk_steam,
                   const Stamp &steam_stamp,
                   const Vid &trunk_vid,
                   const Eigen::Matrix<double, 6, 1> velocity,
                   const double &omega_cmd,
                   const double &v_cmd,
                   const uint &n_gp_pts,
                   const float &max_lateral_3_sig,
                   const float &max_head_3_sig,
                   const float &max_gp_x_stdev,
                   const float &max_gp_y_stdev,
                   const float &max_gp_theta_stdev);
#endif
#if 0
  /**
   * @brief RCExperienceManagement::logPredStatus Message for debugging the GP prediction
   * @param log_vertex: the vertex to log to (used for the run id)
   * @param t_leaf_trunk_stamp
   * @param t_leaf_trunk: current T_leaf_trunk
   * @param trunk_vid
   * @param pred_traj: vector of model predictions
   */
  void logPredStatus(const Vid & log_vertex,
                     const Stamp & t_leaf_trunk_stamp,
                     const TfCov & t_leaf_trunk,
                     const Vid & trunk_vid,
                     const MpcNominalModel::model_trajectory_t & pred_traj);
#endif
#if 0
  /**
   * @brief RCExperienceManagement::logExperience save experience to RoboChunk at the run from vertex log_vertex
   * @param log_vertex: current live vertex. This contains the current run, which is where the experience will be saved
   * @param experience: the experience message to save

   * The order of gp_data.x_meas is as follows:
   * x_test(DIST_ALONG_PATH) = x_input.dist_along_path_k;
   * x_test(OMEGA_ACT_KM1)   = x_input.velocity_km1[1]; // omega act
   * x_test(V_ACT_KM1)       = x_input.velocity_km1[0]; // v act
   * x_test(OMEGA_CMD_KM1)   = x_input.command_km1[1]; // omega cmd
   * x_test(V_CMD_KM1)       = x_input.command_km1[0]; // v cmd
   * x_test(OMEGA_CMD_K)     = x_input.command_k[1];   // omega cmd
   * x_test(V_CMD_K)         = x_input.command_k[0];   // v cmd
   * x_test(HEAD_ERROR_K)    = x_input.tracking_error_k[2]; // Heading error
   * x_test(LAT_ERROR_K)     = x_input.tracking_error_k[1]; // Lateral error
   */
  void logExperience(const Vid log_vertex, const MpcNominalModel::experience_t &experience);
#endif
  /**
   * @brief Wrapper around MpcNominalModel::compute_disturbance_for_experience_km2
   *
   * Computing for km2 because we only know velocity for km1 at time k, which
   * is the velocity that should be compared to the commanded velocity at
   * time km2
   * @return
   */
  bool computeDisturbancesForExperienceKm2();

  /**
   * @brief RCExperienceManagement::computeVelocitiesForExperienceKm1
   * Set the velocity for experience_km1 using the state at the current
   * time-step and the state at the previous step, which are stored
   * in experience_k and experience_km1.
   */
  void computeVelocitiesForExperienceKm1();

  /**
   * @brief RCExperienceManagement::computeVelocitiesFromState
   * use finite difference to compute velocity between state_km1 and state_k
   * @param velocity: output is [v, omega]^T
   * @param state_km1: previous state
   * @param state_k: current state
   * @param d_t: time difference
   */
  void computeVelocitiesFromState(Eigen::VectorXf &velocity,
                                  const Eigen::VectorXf &state_km1,
                                  const Eigen::VectorXf &state_k,
                                  const float d_t);

  /**
   * @brief getExperienceVertexList: get the list of vertices around the current vertex that we will use to retreive experiences.
   * @param current_poseNum: The sequence ID of the current pose
   * @param mpcSize: number of poses in the longest possible look-ahead window. Computed using computeLookahead
   * @param vertexIds: list of vertex IDs in the path.
   * @return list of vertices to look up.
   */
  std::vector<Vid> getExperienceVertexList(const int &current_poseNum,
                                           const int &mpcSize,
                                           const std::vector<Vid> &vertexIds);

  /**
   * @brief getExpSpeedList: get the list of vertices around the current vertex that we will use to retreive experiences.
   * @param current_poseNum: The sequence ID of the current pose
   * @param mpcSize: number of poses in the longest possible look-ahead window. Computed using computeLookahead
   * @param scheduled_speed: vector of scheduled speed indexed by path sequence ID
   * @return list of scheduled speeds in the vertices used to retreive experience
   */
  std::vector<float> getExperienceSpeedList(const int &current_poseNum,
                                            const int &mpcSize,
                                            const std::vector<double> &scheduled_speed);

};

} // path_tracker
} // vtr
