#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string>
#include <Eigen/Dense>

#include <vtr_common/timing/time_utils.hpp>
#include <vtr_path_tracker/robust_mpc/optimization/mpc_nominal_model.h>

namespace vtr {
namespace path_tracker {

/** \brief
 */
class ExperienceManagement {
  friend class RCExperienceManagement;

 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** \brief Constructor
 */
  explicit ExperienceManagement(const rclcpp::Clock& node_clock);

  /** \brief Destructor
 */
  ~ExperienceManagement();

  /** \brief "Experience" at timestep k
  */
  MpcNominalModel::experience_t experience_k_;

  /** \brief "Experience" at timestep k - 1
  */
  MpcNominalModel::experience_t experience_km1_;

  /** \brief "Experience" at timestep k - 2
  */
  MpcNominalModel::experience_t experience_km2_;

  /** Typedef vector of experiences, likely stored at a single vertex **/
  typedef std::vector<MpcNominalModel::experience_t> vertexExperienceVec_t;

  /**
   * @brief ExperienceManagement::InitializeExperiences
   *
   * Initialize fields of the experience that are used to the correct size.
   * This relies on correct definition of STATE_SIZE and VELOCITY_SIZE from
   * ...NominalModel.hpp
   */
  void initializeExperience(MpcNominalModel::experience_t &experience);

  /**
   * @brief ExperienceManagement::initialize_running_experiences
   * @param MpcNominalModel: Used since it has the methods to initialize an experience
   * @param at_vertex_id: The current vertex id
   * @param to_vertex_id: The next vertex id along the path
   * @param turn_radius: the current turn radius along the path
   */
  void initialize_running_experiences(MpcNominalModel &MpcNominalModel,
                                      uint64_t &at_vertex_id,
                                      uint64_t &to_vertex_id,
                                      double &turn_radius);

  /** \brief Handles time in ROS2
  */
  rclcpp::Clock ros_clock;

 private:
  /** \brief
 */
  bool refresh_experiences;
# if 0
  /** \brief
 */
  bool flg_recall_live_data_;
  /** \brief
 */
  int max_num_experiences_per_bin_;
  /** \brief
 */
  int target_model_size_;
#endif

};

} // path_tracker
} // vtr
