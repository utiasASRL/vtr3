#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <memory>

#include <vtr_pose_graph/path/localization_chain.hpp>
#include <vtr_pose_graph/path/path.hpp>
#include <lgmath/se3/Types.hpp>
#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_common/rosutils/transformations.hpp>

#include <vtr_path_tracker/robust_mpc/mpc/utilities.h>
#include <vtr_path_tracker/robust_mpc/mpc/mpc_types.h>
#include <vtr_path_tracker/robust_mpc/mpc/mpc_path.h>
#include <vtr_path_tracker/base.h>
#include <vtr_path_tracker/robust_mpc/optimization/mpc_nominal_model.h>
#include <vtr_path_tracker/robust_mpc/optimization/mpc_solver_XUopt.h>
#include <vtr_path_tracker/robust_mpc/experience/experience_management.h>
#include <vtr_path_tracker/robust_mpc/mpc/time_delay_compensation.h>
#include <vtr_path_tracker/tactic_interface.h>

// Definition of the safety monitor messages
#include <vtr_messages/msg/desired_action_in.hpp>

// For callback to Navigator
#include <action_msgs/msg/goal_status.hpp>
#include <std_msgs/msg/u_int8.hpp>

// Maximum speed before a maneuver is treated as a Turn-On-the-Spot.
// Above this, normal post-processing applies. See rateLimitOutputs
#define MAX_TOS_SPEED 0.001

namespace vtr {
namespace path_tracker {

using lgmath::se3::Transformation;
using lgmath::se3::TransformationWithCovariance;

/**
 * @brief Loads parameters and set the speed profile
 *
 * Fetches parameters from configuration files and the ros parameter server,
 * Extracts desired path poses from the localization chain (for interpolation etc. later)
 */
class PathTrackerMPC : public Base {
 public:

  /**
 * @brief PathTrackerMPC::PathTrackerMPC Constructor
 * @param graph: pointer to the graph. Used for saving experiences.
 * @param nh: node handle. Used for getting ros parameters. Should have
 *            the namespace where the params for the path tracker are kept
 * @param control_period_ms: control period in ms.
 */
  PathTrackerMPC(const std::shared_ptr<Graph> &graph,
                 const std::shared_ptr<rclcpp::Node> node,
                 double control_period_ms,
                 std::string param_prefix);

  // The path for now. Some of this may be pushed to the localization chain.
  std::shared_ptr<MpcPath> path_;

  /** @brief Set up MPC flags and parameters.
 */
  void loadMpcParams();

  /**
 * @brief Method for updating t_leaf_trunk, its time-stamp, and the trunk vid given that leaf
 *
 * This should be called each time there is a VO update.
 * This version is called when STEAM trajectory is not valid or extrapolate_VO parameter is false
 *
 * @param chain
 * @param leaf_stamp common::timing::time_point. Time instance with helpful utilities for converting between time types
 * @param live_vid Vertex id of the current vertex in the live run
 */
  void notifyNewLeaf(const Chain &chain,
                     const Stamp leaf_stamp,
                     const Vid live_vid) override;

  /**
 * @brief Method for updating t_leaf_trunk, given a STEAM trajectory
 *
 * @param chain
 * @param trajectory: STEAM trajectory based at the petiole
 * @param live_vid Vertex id of the current vertex in the live run
 * @param image_stamps
 */
  void notifyNewLeaf(const Chain &chain,
                     const steam::se3::SteamTrajInterface &trajectory,
                     const Vid live_vid,
                     const uint64_t image_stamp) override;

  /** \brief Code to interface with the safety monitor */
  rclcpp::Subscription<vtr_messages::msg::DesiredActionIn>::SharedPtr safety_subscriber_;

  /**
 * @brief PathTrackerMPC::safetyMonitorCallback:
 *          Process requests from the safety monitor.
 * @param msg:
 *            Message from the safety monitor. Can have states CONTINUE,
 *            or SLOW (which don't affect ctrl), and PAUSE or PAUSE_AND_RELOCALIZE
 *            which pause the controller.
 */
  void safetyMonitorCallback(const vtr_messages::msg::DesiredActionIn::SharedPtr msg);

  static std::shared_ptr<Base> Create(const std::shared_ptr<Graph> graph,
                                      const std::shared_ptr<rclcpp::Node> node);
  static constexpr auto type_name = "robust_mpc_path_tracker";

 protected:

  /** \brief Pose from state estimation  */
  VisionPose vision_pose_;

  /** \brief Experience manager */
  RCExperienceManagement rc_experience_management_;

  /** \brief old (?) time delay compensation */
  MpcTimeDelayComp time_delay_comp2_;

  bool previously_paused_ = false;

  /** \brief The solver */
  vtr::path_tracker::MpcSolverXUopt solver_;

  /** \brief Parameters and temporary variables */
  vtr::path_tracker::mpc_params_t mpc_params_;

  /** \brief A pointer to the Navigator node */
  const std::shared_ptr<rclcpp::Node> node_;

  /** \brief Namespace for parameters. e.g. node namespace + "/path_tracker" */
  std::string param_prefix_;

  /** \brief ROS2 publisher for velocity command */
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_done_path_;

  /** \brief ROS2 publisher notifying path is completed */
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  /** @brief Set up optimization Flags and parameters. */
  void loadSolverParams();

  /** @brief Get parameters related to speed scheduling, optimization, and MPC flags */
  void getParams();

  /**
 * @brief resetIfPreviouslyPaused
 *
 *  This method resets the commands and "hot start" for the solver
 *  if the robot has previously paused. It also checks the state and sets the previously paused
 *  flag. This should be called at the beginning of each control step.
 *
 * @return true if the robot is resuming from a pause.
 */
  bool resetIfPreviouslyPaused();

  /**
 * @brief Check if the path has been completed.
 *
 *  Returns true if we are within path_end_heading_threshold_ and path_end_x_threshold of the last
 *  node in the path.
 *
 * @return complete=true/false
 */
  bool checkPathComplete();

  /**
 * @brief PathTrackerMPC::finishControlLoop
 * Function to call when the path is finished. This sends a stop command to the vehicle and status to the navigator.
 *
 * Sends a ROS message to the Navigator indicating the status of the path.
 */
  void finishControlLoop() override;

  /**
 * @brief PathTrackerMPC::publishCommand Publish the command to ROS
 * @param command: TwistStamped message
 */
  void publishCommand(Command &command) override;

  /**
 * @brief Base::controlLoopSleep Sleep for remaining time in control loop
 *
 * Behaviour depends on mpc_params_.flg_use_fixed_ctrl_rate.
 *  true: sleep for remaining time in control loop
 *  false: sleep for 35 ms (this is what the old path tracker did)
 */
  void controlLoopSleep() override;

  /**
 * @brief PathTrackerBase::getLocalPathErrors Compute the local and look-ahead errors
 * @param local_path
 * @param heading_error: heading error based on the current pose relative to XX
 * @param look_ahead_heading_error: heading error to the end of the look-ahead window
 * @param lateral_error: lateral error based on the current pose relative to XX
 * @param longitudinal_error: longitudinal error based on the current pose relative to XX
 * @param look_ahead_longitudinal_error: longitudinal error relative to the end of the look-ahead window
 * @param tos_look_ahead_poses: the number of TURN_ON_SPOT vertices in the look-ahead window.
 */
  void getLocalPathErrors(const local_path_t local_path,
                          float &heading_error, float &look_ahead_heading_error,
                          float &lateral_error, float &longitudinal_error, float &look_ahead_longitudinal_error,
                          const int &tos_look_ahead_poses);

  /**
 * @brief PathTrackerBase::getErrorToEnd Get the linear and angular error to the last vertex in the path.
 * @param linear_distance:  the Euclidean distance from the leaf to the last vertex in the path.
 * @param angular_distance: The Euclidean norm of the angle between the leaf and the last vertex in the path
 */
  void getErrorToEnd(double &linear_distance, double &angular_distance);

  /**
 * @brief PathTrackerBase::computeCommandPControl
 *
 * OUTPUT
 * @param linear_speed_cmd   output
 * @param angular_speed_cmd  output
 *
 * INPUT
 * @param use_tos_ctrl  flag for turn on the spot
 * @param use_end_ctrl  flag for end
 * @param use_dir_sw_ctrl  flag for dir sw
 * @param target_linear_speed
 * @param gain_schedule THIS MUST BE THE CURRENT GAIN SCHEDULE!
 * @param look_ahead_longitudinal_error
 * @param look_ahead_heading_error
 * @param longitudinal_error
 * @param dist_to_end
 */
  void computeCommandFdbk(float &linear_speed_cmd, float &angular_speed_cmd,
                          const bool use_tos_ctrl, const bool use_end_ctrl, const bool use_dir_sw_ctrl,
                          float &target_linear_speed, gain_schedule_t &gain_schedule,
                          const local_path_t local_path, const int num_tos_poses_ahead);

  /**
 * @brief PathTrackerBase::computeFeedbackLinearizedControl Feedback linearized control in case MPC doesn't work.
 * @param linear_speed_cmd: commanded linear speed
 * @param angular_speed_cmd: commanded linear speed
 * @param local_path: local_path_t struct containing the current local path.
 * @param num_tos_poses_ahead: number of TOS poses ahead?
 * It is important to aim for the pose ahead (particularly in the angle) because the localization chain weights angle much
 * higher than position. Therefore, the trunk will only move forward if the vehicle is aligned with the next vertex or
 * if vertices are very far apart.
 *
 * NOTE: THIS CONTROLLER GOES AT 0.3 m/s. Hard coded.
 */
  void computeFeedbackLinearizedControl(float &linear_speed_cmd,
                                        float &angular_speed_cmd,
                                        const local_path_t local_path);

  // convenience functions
  /** @brief PathTrackerBase::setLatestCommand Convenience function for setting the command to send to the robot
 *
 * @param linear_speed_cmd
 * @param angular_speed_cmd
 */
  void setLatestCommand(const double linear_speed_cmd, const double angular_speed_cmd);

  /** @brief Check pose n for end_ctrl */
  bool checkEndCtrl(const int pose_n);

  /** @brief Check pose n for tos control */
  bool checkTOS(const int pose_n);

  /** @brief Check pose n for dir_sw control   */
  bool checkDirSw(const int pose_n);

  /**
 * @brief Compute the length of the MPC window given the current pose is current_pose_num.
 * @param scheduled_ctrl_mode: scheduled control mode for vertices ahead
 * @param current_pose_num: Current closest vertex (trunk)
 * @param max_lookahead: maximum length of the MPC look-ahead window.
 * @return maximum number of poses to look ahead in MPC horizon
 */
  int computeLookahead(const std::vector<VertexCtrlType> &scheduled_ctrl_mode,
                       const int &current_pose_num,
                       const int &max_lookahead);

  /**
 * @brief PathTrackerBase::rateLimitOutputs Limit the commands based on a max acceleration and wheel speed.
 * @param v_cmd: current commanded translational speed
 * @param w_cmd: current commanded angular speed
 * @param v_cmd_km1: previous commanded translational speed
 * @param params: includes max_accel, v_max, w_max
 * @param d_t: time between the current and previous commands
 * @return
 */
  bool rateLimitOutputs(float &v_cmd, float &w_cmd, const float &v_cmd_km1, const path_params_t &params, float d_t);

  // Virtual methods from Base
  /** @brief Load ROS parameters and do path pre-processing */
  void loadConfigs() override;

  /**
 * @brief Main code to compute the control action
 *
 * @return A ROS2 "twist" velocity msg
 */
  Command controlStep() override;

  /**
 * @brief Compute the commanded linear and angular velocity using MPC
 * @param linear_speed_cmd
 * @param angular_speed_cmd
 * @return true if MPC output is valid. i.e. no errors during the optimization.
 */
  bool computeCommandMPC(float &v_cmd,
                         float &w_cmd,
                         local_path_t &local_path);

  /**
 * @brief PathTrackerMPC::rotateDisturbanceIntoPoseNumFrame
 * Disturbances are measured and predicted in the robots frame
 *  but MPC computes the robot trajectory in the frame of the next
 *  desired pose.
 *  g(a_k) = predicted disturbance in robot frame
 *  g(a_k_des_frame) = predicted disturbance in frame of desired pose
 * @param x_input
 */
  void rotateDisturbanceIntoPoseNumFrame(MpcNominalModel::model_state_t &x_input);

  /**
 * @brief Project poses ahead and behind the vehicle to the 2D plane.
 *
 * local_path.x_des_fwd contains the 2D transform from pose k to k+1 expressed in frame
 * local_path.x_des_bck ... same as x_des_fwd but behind the vehicle.
 * local_path.x_act contains the 2D pose of the vehicle wrt frame k expressed in frame k
 * local_path.T_0_v is the transform from the vehicle frame to the root.
 *
 * local_path is probably filled out with differential transforms to avoid wrapping angles where possible.
 *
 * @param local_path contains information about the differential transformations between poses in 2D
 * @param tos_lookaheadPose the number of TURN_ON_SPOT vertices in the look-ahead window.
 */
  void flattenDesiredPathAndGet2DRobotPose(local_path_t &local_path, int &tos_lookaheadPose);

  /** @brief Converts a ROS geometry_msgs::Pose to a ROS 3D point and quaternion */
  void geometryPoseToTf(const geometry_msgs::msg::Pose &pose,
                        tf2::Vector3 &point,
                        tf2::Quaternion &quaternion);

  /**
 * @brief Chris O's method for finding the closest vertex in the path.
 *
 * @param local_path The current local path
 * @param initialGuess the initial guess for the sequence id of the closest vertex
 * @param radiusForwards: how many vertices to search forwards
 * @param radiusBackwards: how many vertices to search backwards
 */
  void locateNearestPose(local_path_t &local_path,
                         unsigned initialGuess,
                         unsigned radiusForwards,
                         unsigned radiusBackwards);

  /**
 * @brief Set x_pred and x_opt to zero, except the first element which contains the current state.
 * @param mpcSize
 * @param NominalModel
 * @param Solver
 * @param experience_management
 * @param local_path
 */
  void initializeModelTrajectory(int &mpcSize,
                                 MpcNominalModel &NominalModel,
                                 MpcSolverBase &Solver,
                                 const RCExperienceManagement &experience_management,
                                 local_path_t local_path);

  /**
 * @brief Set up experience management
 *
 * Fetch params and initialize internal variables.
 */
  void initializeExperienceManagement();

  /** \brief Sets the VisionPose isUpdated to false */
  void reset() override;
};

} // namespace path_tracker
} // namespace vtr
