#pragma once

// ignore warnings for unused parameters from ROS code and Chris O's optimization code
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wreturn-type"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

#include <ros/ros.h>
#include <tf/transform_listener.h>

#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop
#pragma GCC diagnostic pop

// System includes
#include <Eigen/Core>
#include <memory>

// ASRL includes
#include <asrl/pose_graph/path/LocalizationChain.hpp>
#include <asrl/pose_graph/path/Path.hpp>
#include <lgmath/se3/Types.hpp>
#include <asrl/common/timing/SimpleTimer.hpp>
#include <asrl/common/rosutil/transformation_utilities.hpp>

// MPC includes
#include <vtr/path_tracker/robust_mpc/mpc/utilities.h>
#include <vtr/path_tracker/robust_mpc/mpc/mpc_types.h>
#include <vtr/path_tracker/robust_mpc/mpc/mpc_path.h>
#include <vtr/path_tracker/base.h>
#include <vtr/path_tracker/robust_mpc/optimization/path_tracker_mpc_nominal_model.h>
#include <vtr/path_tracker/robust_mpc/optimization/path_tracker_mpc_solver_XUopt.h>
#include <vtr/path_tracker/robust_mpc/experience/experience_management.h>
//#include <asrl/path_tracker/robust_mpc/experience/experience_recommendation.hpp>
#include <vtr/path_tracker/robust_mpc/mpc/time_delay_compensation.h>
//#include <asrl/path_tracker/robust_mpc/models/original_gaussian_process.hpp>
#include <vtr/path_tracker/tactic_interface.h>

// Debugging and visualization includes
//#include <asrl/path_tracker/robust_mpc/rvizdebugplotter.hpp>

//Definition of the safety monitor messages
#include <asrl__messages/DesiredActionIn.h>

// For callback to Navigator
#include <actionlib_msgs/GoalStatus.h>
#include <std_msgs/UInt8.h>

// Maximum speed before a maneuver is treated as a Turn-On-the-Spot.
// Above this, normal post-processing applies. See rateLimitOutputs
#define MAX_TOS_SPEED 0.001

namespace vtr {
namespace path_tracker {

using lgmath::se3::Transformation;
using lgmath::se3::TransformationWithCovariance;

class PathTrackerMPC : public Base {
public:

  /**
     * @brief      Basic constructor for the path tracker.
     *             Base has graph_
     * @param  graph  pointer to the graph
     * @param  chain  pointer to the chain to follow
     * @param  nh     ros node handle used for fetching parameters
     */
  PathTrackerMPC(const std::shared_ptr<Graph> & graph,
                 ros::NodeHandle& nh,
                 double control_period_ms,
                 std::string param_prefix);

  // The path for now. Some of this may be pushed to the localization chain.
  std::shared_ptr<MpcPath> path_;
  void loadMpcParams();
#if 0
  bool loadGpParams();
#endif

  void notifyNewLeaf(const Chain & chain,
                     const Stamp leaf_stamp,
                     const Vid live_vid);

  void notifyNewLeaf(const Chain & chain,
                     const steam::se3::SteamTrajInterface& trajectory, ///< Steam trajectory correesponding to T_leaf_petiole_cov
                     const Vid live_vid, ///< Vid of the current vertex in the live run
                     const uint64_t image_stamp);

  unsigned trunk_seq_id;

  // Temporary code to interface with existing safety monitor
  ros::Subscriber safety_subscriber_;
  void safetyMonitorCallback(const asrl__messages::DesiredActionIn::Ptr & msg);

  void checkIfPastVertex(int pose_i, float v_des, MpcNominalModel NominalModel, local_path_t local_path, int pose_im1, int pred_index);

  static std::shared_ptr<Base> Create(const std::shared_ptr<Graph> graph,
                                      ros::NodeHandle * nh);
  static constexpr auto type_name = "robust_mpc_path_tracker";

protected:

#if 0
  std::shared_ptr<RvizDebugPlotter> rviz_debug_plt_;
#endif

  std::thread path_tracker_thread_;
  ::asrl::common::timing::SimpleTimer timer_; ///< for querying the current time with get_time()

  // Pose from state estimation
  VisionPose vision_pose_;


  // Experience manager
  RCExperienceManagement rc_experience_management_;
#if 0
  ExperienceRecommendation rc_exp_rec_;
  GpFunctionApproximator gp_model_;
#endif

  // old time delay compensation
  MpcTimeDelayComp time_delay_comp2_;

  bool previously_paused_ = false;

  // Solver
  vtr::path_tracker::MpcSolverXUopt solver_;

  // Parameters and temporary variables
  vtr::path_tracker::mpc_params_t mpc_params_;
  ros::NodeHandle& nh_;
  std::string param_prefix_; ///< namespace for parameters. e.g. node namespace + "/path_tracker"

  // ROS publisher
  ros::Publisher publisher_;
  ros::Publisher pub_done_path_;

  // Methods
  void loadSolverParams();
  void getParams();

  bool resetIfPreviouslyPaused();
  bool checkPathComplete();
  void finishControlLoop();
  void publishCommand(Command &command);
  void controlLoopSleep();

  void getLocalPathErrors(const local_path_t local_path,
                          float & heading_error, float & look_ahead_heading_error,
                          float & lateral_error, float & longitudional_error, float & look_ahead_longitudional_error,
                          const int &tos_look_ahead_poses);
  void getErrorToEnd(double &linear_distance, double &angular_distance);
  void computeCommandFdbk(float &linear_speed_cmd, float &angular_speed_cmd,
                          const bool use_tos_ctrl, const bool use_end_ctrl, const bool use_dir_sw_ctrl,
                          float &target_linear_speed, gain_schedule_t & gain_schedule,
                          const local_path_t local_path, const int num_tos_poses_ahead);
  void computeFeedbackLinearizedControl(float &linear_speed_cmd,
                                        float &angular_speed_cmd,
                                        const local_path_t local_path);

  // convenience functions
  void setLatestCommand(const double linear_speed_cmd, const double angular_speed_cmd);
  bool checkEndCtrl(const int pose_n);
  bool checkTOS(const int pose_n);
  bool checkDirSw(const int pose_n);

  // MPC optimization loop
  int computeLookahead(const std::vector<VertexCtrlType> &scheduled_ctrl_mode, const int & current_pose_num, const int & max_lookahead);

  // additional checks
  bool rateLimitOutputs(float & v_cmd, float & w_cmd, const float & v_cmd_km1, const path_params_t & params, float d_t);

  // Virtual methods from Base
  void loadConfigs();
  Command controlStep();

  // Other methods related to computing the MPC command
  void initializeModelTrajectory(int & mpcSize,
                                   MpcNominalModel & NominalModel,
                                   MpcSolverBase & Solver,
                                   local_path_t local_path);
  bool computeCommandMPC(float & v_cmd,
                         float & w_cmd,
                         local_path_t& local_path);
  void rotateDisturbanceIntoPoseNumFrame(MpcNominalModel::model_state_t & x_input);
  void compute2DError(const unsigned seq_id, Eigen::Vector3f &error);

  void flattenDesiredPathAndGet2DRobotPose(local_path_t & local_path, int & tos_lookaheadPose);
  void geometryPoseToTf(const geometry_msgs::Pose & pose,
                        tf::Point & point,
                        tf::Quaternion & quaternion);
  void locateNearestPose(local_path_t & local_path,
                         unsigned initialGuess,
                         unsigned radiusForwards,
                         unsigned radiusBackwards);


  void initializeModelTrajectory(int & mpcSize, MpcNominalModel & NominalModel, MpcSolverBase & Solver, const RCExperienceManagement & experience_management, local_path_t local_path);

#if 0
  // Learning-related methods
  void computeDisturbance(MpcNominalModel::model_state_t & x_input,
                          MpcNominalModel & NominalModel,
                          GpFunctionApproximator & GpModel,
                          const float & th_des,
                          const float & d_t);
#endif
  void initializeExperienceManagement();

#if 0
  void initializeExperienceRecommendation();
#endif

  void reset();
};

} // namespace path_tracker
} // namespace vtr

