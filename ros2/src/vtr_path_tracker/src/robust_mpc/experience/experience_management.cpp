
#include <vtr/path_tracker/robust_mpc/experience/experience_management.h>

namespace vtr {
namespace path_tracker {

RCExperienceManagement::RCExperienceManagement(const std::shared_ptr<Graph> &graph)
    : graph_(graph), ExperienceManagement() {
}

MpcNominalModel::experience_t RCExperienceManagement::experience_tFromRobochunk(const RobochunkExperience &rc_experience) {

  MpcNominalModel::experience_t experience;
  initializeExperience(experience);

  // Load simple values
  experience.at_vertex_id = rc_experience.at_vid();
  experience.to_vertex_id = rc_experience.to_vid();
  experience.path_curvature = rc_experience.path_curvature();
  experience.disturbance_is_valid =
      true; // The experience is only logged if it is valid, so all experiences read from the graph are valid.

  /// \todo: (old) Remove. Set time to current time if none is available. This is for backwards compatibility. Can remove once tested on the vehicle.
  if (rc_experience.has_store_time()) {
    experience.store_time =
        ::asrl::common::timing::toRosTime(::asrl::common::timing::toChrono(rc_experience.store_time()));
  } else {
    experience.store_time = ros::Time(0);
    experience.disturbance_is_valid = false;
  }

  /// \todo: (old) Store commands if available. This can be removed once tested on the vehicle and experience type is constant.
  if (rc_experience.command_size() == VELOCITY_SIZE) {
    experience.x_k.command_k << rc_experience.command(0), rc_experience.command(1);
  }

  // Load the gp feature
  unsigned input_dim = rc_experience.gp_meas().input_size();
  unsigned output_dim = rc_experience.gp_meas().output_size();

  // Load data into the GP input/output pairs making sure the dimensions match up
  if (input_dim == DIST_DEP_SIZE) {
    for (unsigned i = 0; i < input_dim; ++i) {
      experience.gp_data.x_meas[i] = rc_experience.gp_meas().input(i);
    }
  } else {
    LOG(ERROR) << "ERROR: Saved features for the path tracker are not the right size (9x1)! Setting to zero.";
  }

  if (output_dim == STATE_SIZE) {
    for (unsigned i = 0; i < output_dim; ++i) {
      experience.gp_data.g_x_meas[i] = rc_experience.gp_meas().output(i);
    }
  } else {
    LOG(ERROR) << "ERROR: Saved output for the path tracker GP are not the right size (3x1)! Setting to zero.";
  }

  return experience;
}

std::vector<MpcNominalModel::experience_t> RCExperienceManagement::loadSpatialExperience(const Vid vertex) {

  std::vector<MpcNominalModel::experience_t> experience_list;

  // Get vid of spatial neighbours in the graph
  auto current_vertex = graph_->at(vertex);
  auto spatial_neighbours = current_vertex->spatialNeighbours();

  // Store all messages
  for (auto &vid : spatial_neighbours) {
    auto neighbour = graph_->at(vid); //get the vertex
    std::vector<std::shared_ptr<RobochunkExperience>>
        tmp_robochunk_msgs = neighbour->retrieveData<RobochunkExperience>("/control/experience");

    // Make sure there is experience at the current vertex.
    // If so, push to experience_list_by_vertex_
    for (auto itr = tmp_robochunk_msgs.begin(); itr != tmp_robochunk_msgs.end(); ++itr) {
      experience_list.emplace_back(experience_tFromRobochunk(**itr));
    }
  }

  return experience_list;
}

#if 0
std::vector<MpcNominalModel::gp_data_t> RCExperienceManagement::getGpDataFromRCExperience(const std::vector<Vid> & vertex_list,
                                                                                          const std::vector<float> & speed_profile_vec,
                                                                                          const boost::unordered_map<Vid, double> & pathLengthByVertex,
                                                                                          float v_km1) {
  std::vector<MpcNominalModel::gp_data_t> gpBasisExperiences;

  constexpr float max_deltaV = 1.5;
  constexpr std::array<float, 8> speed_ranges {0.05, 0.1, 0.18, 0.3, 0.5, 0.6, 0.7, max_deltaV};
  std::array<std::vector<MpcNominalModel::gp_data_t>, speed_ranges.size()> gpBasisExperiences_by_speed;

  const int numVertices = vertex_list.size();

  float v_des = 0;

  int count_failVertexCheck = 0;
  int count_failTimeCheck = 0;
  int count_failVelocityCheck = 0;

  float v_min = 100, v_max = -100;
  float d_min = 100, d_max = -100;
  int count = 0;

  // get all experiencese close to vertices in the list
  vertexExperienceVec_t list_of_experiences, local_experiences, more_experiences;
  for (Vid v : vertex_list) {
    more_experiences = loadSpatialExperience(v);
    local_experiences.insert(local_experiences.end(), more_experiences.begin(), more_experiences.end());
  }

  // Sort experiences in the order they are stored (oldest first) so that FIFO binning sorts experience in is the same order as Chris O's.
  // Sort using a lambda expression as the comparison operator.
  std::sort(local_experiences.begin(),
            local_experiences.end(),
            [](const MpcNominalModel::experience_t & exp_0,
               const MpcNominalModel::experience_t & exp_1) {
                return exp_0.store_time.toNSec() < exp_1.store_time.toNSec();
               }
            );

  // Enforce first-in-first-out storage and other conditions from Chris O's code.
  list_of_experiences = enforceFifoBins(local_experiences);

  // Iterate through the vertex list
  for (int vertex_ind = 0; vertex_ind < numVertices -1 ; vertex_ind++) {
    const Vid & atVertex = vertex_list[vertex_ind];
    const Vid & toVertex = vertex_list[vertex_ind+1];

    // Get the desired speed at the vertex we are loading experience for
    v_des = speed_profile_vec[vertex_ind];
    if (fabs(v_km1) > 0){
      v_des = v_km1;
    }

    // Iterate through the experiences at the current vertex and sort them into bins by velocity
    // as well as modifying the distance along path by vertex since this changes for each run.
    for (uint exp_ind = 0; exp_ind < list_of_experiences.size(); exp_ind++){

      // Check if experience i is going to the correct toVertex. if so, add the experiences
      // to gpBasisExperiences_by_speed, which is a vector of experiences sorted by speed.
      if ((list_of_experiences[exp_ind].to_vertex_id != toVertex) or
          (list_of_experiences[exp_ind].at_vertex_id != atVertex)) {
        ++count_failVertexCheck;
        continue;
      }

      // Check if the experience is old enough - if it's too new, it can cause instability
      float deltaT = (ros::Time::now()).toSec() - list_of_experiences[exp_ind].store_time.toSec();
      bool data_old_enough;
      if (flg_recall_live_data_ == true && deltaT > 1.0){
        data_old_enough = true;
      } else if (deltaT > min_exp_age_){
        data_old_enough = true;
      } else{
        data_old_enough = false;
        count_failTimeCheck++;
      }

      // Check if the experience occured at close to the right velocity
      float deltaV = std::abs(list_of_experiences[exp_ind].x_k.command_k[0] - v_des);

      // Check if the experience is valid
      bool valid_disturbance = list_of_experiences[exp_ind].disturbance_is_valid;

      if(!data_old_enough or !valid_disturbance) continue;

      // Sort experiences into bins by speed relative to the current desired speed
      // and store this in gpBasisExperiences_by_speed
      // uses a binary search, and gives you the greater-than-max check for free
      auto speed_range_it = std::upper_bound(speed_ranges.begin(), speed_ranges.end(), deltaV);
      if (speed_range_it == speed_ranges.end()) {
        ++count_failVelocityCheck;
        continue;
      }
      int v = std::distance(speed_ranges.begin(), speed_range_it);

      // Set the dist along path field of x_meas.  This is trial specific since the path length
      // changes as a function of the desired path for this trial
      list_of_experiences[exp_ind].gp_data.x_meas[DIST_ALONG_PATH] = pathLengthByVertex.at(atVertex);

      // Add the GP data to the gpBasisExperiences_by_speed
      gpBasisExperiences_by_speed[v].push_back((list_of_experiences[exp_ind].gp_data));
      v_min = std::min(v_min, list_of_experiences[exp_ind].gp_data.x_meas(6));
      v_max = std::max(v_max, list_of_experiences[exp_ind].gp_data.x_meas(6));
      d_min = std::min(d_min, list_of_experiences[exp_ind].gp_data.x_meas(0));
      d_max = std::max(d_max, list_of_experiences[exp_ind].gp_data.x_meas(0));

    }
  }

  v_min = 100; v_max = -100;
  d_min = 100; d_max = -100;
  count = 0;

  // Add experiences to the list of experiences returned to construct the GP.
  // Start from experiences where the commanded velocity was the closest to the current commanded velocity until the target model size is reached.
  bool done_getting_experiences = false;

  for (const auto & gpBasisExperiences_v: gpBasisExperiences_by_speed) {
    for (const auto & gpBasisexperiences_v_e : gpBasisExperiences_v) {

      // Reject points that are too close together... to make sure experiences are diverse enough??
      bool add_datapoint = std::all_of(gpBasisExperiences.begin(), gpBasisExperiences.end(),
                                       [&](const MpcNominalModel::gp_data_t & gpbe) -> bool {
        float distance = std::abs((gpbe).x_meas(V_CMD_K) - (gpBasisexperiences_v_e).x_meas(V_CMD_K)) +
            std::abs((gpbe).x_meas(DIST_ALONG_PATH) - (gpBasisexperiences_v_e).x_meas(DIST_ALONG_PATH));
        return distance >= 0.3;
      });

      if (add_datapoint) {
        gpBasisExperiences.push_back(gpBasisexperiences_v_e);
        v_min = std::min(v_min, (gpBasisexperiences_v_e).x_meas(V_CMD_K));
        v_max = std::max(v_max, (gpBasisexperiences_v_e).x_meas(V_CMD_K));
        d_min = std::min(d_min, (gpBasisexperiences_v_e).x_meas(DIST_ALONG_PATH));
        d_max = std::max(d_max, (gpBasisexperiences_v_e).x_meas(DIST_ALONG_PATH));
        count++;
      }
      if (count >= target_model_size_) {
        done_getting_experiences = true;
        break;
      }
    }
    if (done_getting_experiences){
      break;
    }
  }

  return gpBasisExperiences;

}
#endif

RCExperienceManagement::vertexExperienceVec_t RCExperienceManagement::enforceFifoBins(vertexExperienceVec_t &new_experience_list) {

  // Vector to store experiences as if they were sorted into FIFO binds
  vertexExperienceVec_t experience_fifo_list;

  // Iterate through all new experiences and push them into fifo bins
  for (MpcNominalModel::experience_t &experience_in : new_experience_list) {

    int num_entries = 0; // number of entries already stored that have speed close to the current experience
    uint oldest_experience_index = 1;
    uint current_trial_experience_index = 0;

    ros::Time oldest_experience_time = ros::Time::now();

    bool flg_found_exp_from_curr_trial = false;
    bool flg_replace_exp_from_curr_trial = false;
    float tmp_experience_x_err = std::abs(experience_in.x_k.tracking_error_k(0));

    // Index begins at 1 because element 0 is always the default element... ?? Not sure if this applies
    // if all we sort through all the data each time. Starting from zero. The output of the old/new PTs
    // match in neither case. Might be related to flg_do_not_delete, which is no longer used because I
    // don't understand why it is there and it does not help to match the output of the old/new PTs.
    // Iterate through all experiences and check if they fall in the same speed bin as each new experience.
    // If so, count the number of experiences in the same bin, and get the index of the oldest experience.
    for (uint i = 1; i < experience_fifo_list.size(); i++) {

      // only place experiences that are going from/to the same vertices in the same bin
      // If an experience has a new from/to vid combo, add it to the list.
      if ((experience_fifo_list[i].to_vertex_id == experience_in.to_vertex_id) and
          (experience_fifo_list[i].at_vertex_id == experience_in.at_vertex_id)) {

        // Check if experience_in is close to speed of existing experience[i]
        // This is what defines the speed bins. If none are close to the new experience, add it to the list.
        if (std::abs(experience_fifo_list[i].x_k.command_k[0] - experience_in.x_k.command_k[0]) < 0.1) {
          num_entries++;
          float delta_t_oldest = experience_fifo_list[i].store_time.toSec() - oldest_experience_time.toSec();
          float delta_t_current_trial = experience_fifo_list[i].store_time.toSec() - start_of_current_trial_.toSec();

          // Get index to oldest experience.
          // Used to replace the oldest experience if a speed bin is full.
          if (delta_t_oldest < 0. && experience_fifo_list[i].flg_do_not_delete == false) {
            oldest_experience_time = experience_fifo_list[i].store_time;
            oldest_experience_index = i;
          }

          // Check if experience[i] is from the current trial
          if (delta_t_current_trial > 0.) {
            flg_found_exp_from_curr_trial = true;

            // Check if the new experience has lower tracking error
            if (std::abs(experience_fifo_list[i].x_k.tracking_error_k(0)) > tmp_experience_x_err) {
              flg_replace_exp_from_curr_trial = true;
              current_trial_experience_index = i;
              tmp_experience_x_err = std::abs(experience_fifo_list[i].x_k.tracking_error_k(0));
            }
          }
        }
      }
    }

    // push an experience if it is in a new bin. Otherwise,
    // either replace an experience from the current run with higher tracking error or
    // the oldest experience in the bin if none is found from the current run.
    if (num_entries < max_num_experiences_per_bin_) {
      if (num_entries == 0) {
        experience_in.flg_do_not_delete = true;
      }
      experience_fifo_list.push_back(experience_in);
    } else if (refresh_experiences == true) {
      if (flg_found_exp_from_curr_trial == true) {
        if (flg_replace_exp_from_curr_trial == true) {
          experience_fifo_list[current_trial_experience_index] = experience_in;
        }
      } else {
        experience_fifo_list[oldest_experience_index] = experience_in;
      }
    }
  } // done processing all experiences in the list.

  return experience_fifo_list;
}

void RCExperienceManagement::logPtStatus(const Vid &log_vertex,
                                         const TfCov &t_leaf_trunk_vo,
                                         const Stamp &vo_stamp,
                                         const Vid &vo_trunk_vid,
                                         const TfCov &t_leaf_trunk_steam,
                                         const Stamp &steam_stamp,
                                         const Vid &trunk_vid,
                                         const Eigen::Matrix<double, 6, 1> velocity,
                                         const double &omega_cmd,
                                         const double &v_cmd,
                                         const uint &n_pts_gp,
                                         const float &max_lateral_3_sig,
                                         const float &max_head_3_sig,
                                         const float &max_gp_x_stdev,
                                         const float &max_gp_y_stdev,
                                         const float &max_gp_theta_stdev) {

  // Make sure the stream is registered
  RunId rid = log_vertex.majorId();
  std::string results_stream = RCExperienceManagement::ResultStream::status;
  if (!graph_->hasVertexStream(rid, results_stream)) {
    graph_->registerVertexStream(rid, results_stream, true);
    LOG(WARNING) << "Path tracker registered new stream at run " << rid << " on stream " << results_stream;
    return;
  }

  // Fill in the status message
  ::asrl::path_tracker_msgs::PtStatus status_msg;

  // Transformation data
  *status_msg.mutable_t_leaf_trunk() << t_leaf_trunk_vo;
  *status_msg.mutable_t_leaf_trunk_extrapolated() << t_leaf_trunk_steam;
  for (uint i = 0; i < velocity.rows(); ++i) {
    status_msg.mutable_velocity()->Add(velocity(i));
  }

  // Timing data
  status_msg.set_extrapolated_time_stamp(::asrl::common::timing::toUnix(vo_stamp));
  status_msg.set_vo_time_stamp(::asrl::common::timing::toUnix(steam_stamp));

  // Vertex in the privileged path we are localizing against.
  status_msg.set_trunk_id(trunk_vid);
  status_msg.set_vo_trunk_id(vo_trunk_vid);

  // Control data
  status_msg.set_omega_cmd(omega_cmd);
  status_msg.set_v_cmd(v_cmd);
  status_msg.set_cmd_time_stamp(::asrl::common::timing::toUnix(steam_stamp));

  // Uncertainty over prediction horizon
  status_msg.set_lateral_uncertainty(max_lateral_3_sig);
  status_msg.set_heading_uncertainty(max_head_3_sig);

  // GP info over prediction horizon
  status_msg.set_n_pts_gp(n_pts_gp);
  status_msg.add_gp_uncertainty(max_gp_x_stdev);
  status_msg.add_gp_uncertainty(max_gp_y_stdev);
  status_msg.add_gp_uncertainty(max_gp_theta_stdev);

  // Insert the message into the run
  auto stamp = ::asrl::common::timing::toRobochunk(::asrl::common::timing::clock::now());
  graph_->runs().at(rid)->insert<::asrl::path_tracker_msgs::PtStatus>(results_stream, status_msg, stamp);
}

#if 0
void RCExperienceManagement::logPredStatus(const Vid & log_vertex,
                                           const Stamp & t_leaf_trunk_stamp,
                                           const TfCov & t_leaf_trunk,
                                           const Vid & trunk_vid,
                                           const MpcNominalModel::model_trajectory_t & pred_traj) {

  // Establish the logging stream
  const RunId & rid = log_vertex.majorId();
  const std::string results_stream = RCExperienceManagement::ResultStream::prediction;
  if (!graph_->hasVertexStream(rid, results_stream)) {
    graph_->registerVertexStream(rid, results_stream, true);
    LOG(WARNING) << "Path tracker registered new stream at run " << rid << " on stream " << results_stream;
  }

  PredStatus pred_msg;

  // Set localization/timing info
  pred_msg.set_t_stamp(::asrl::common::timing::toUnix(t_leaf_trunk_stamp));
  pred_msg.set_trunk_vid(trunk_vid);
  *pred_msg.mutable_t_leaf_trunk() << t_leaf_trunk;

  // Fill out the prediction from the GP for each step in the look-ahead horizon
  MpcNominalModel mdl;
  for (const MpcNominalModel::model_state_t & s : pred_traj) {
    GpPred * gp_pred_ptr = pred_msg.add_gp_pred();
    for (uint i = 0; i < s.g_a_k_des_frame.rows(); ++i) {
      gp_pred_ptr->add_mean(s.g_a_k_des_frame(i));
      gp_pred_ptr->add_stdev(std::sqrt(s.var_g_a_k_des_frame(i,i)));
    }
    Eigen::VectorXf a;
    mdl.extract_disturbance_dependencies(s, a);
    for (uint i = 0; i < a.rows(); ++i) {
      gp_pred_ptr->add_test_point(a(i));
    }
  }

  // insert a message into the run
  robochunk::std_msgs::TimeStamp stamp;
  stamp.set_nanoseconds_since_epoch(::asrl::common::timing::toUnix(t_leaf_trunk_stamp));
  graph_->runs().at(rid)->insert<PredStatus>(results_stream, pred_msg, stamp);

  return;
}
#endif

#if 0
void RCExperienceManagement::logExperience(const Vid log_vertex, const MpcNominalModel::experience_t & experience)
{

  // Make sure the stream is registered
  RunId rid = log_vertex.majorId();
  std::string results_stream = RCExperienceManagement::ResultStream::experience;
  if (!graph_->hasVertexStream(rid, results_stream)) {
    graph_->registerVertexStream(rid, results_stream, true);
    LOG(WARNING) << "Path tracker registered new stream at run " << rid << " on stream " << results_stream;
    return;
  }

  // Convert the experience message to robochunk type
  RobochunkExperience experience_msg = experience_tToRobochunk(experience);
  experience_msg.set_store_time(ros::Time::now().toNSec());

  // insert a message into the run
  robochunk::std_msgs::TimeStamp stamp;
  stamp.set_nanoseconds_since_epoch(experience.transform_time.toNSec());
  graph_->runs().at(rid)->insert<RobochunkExperience>(results_stream, experience_msg, stamp);

}
#endif

#if 0
RobochunkExperience RCExperienceManagement::experience_tToRobochunk(MpcNominalModel::experience_t experience) {

  // Convert the experience message to robochunk type
  RobochunkExperience rc_experience;

  rc_experience.set_at_vid(experience.at_vertex_id);
  rc_experience.set_to_vid(experience.to_vertex_id);
  rc_experience.set_path_curvature(experience.path_curvature);
  rc_experience.mutable_command()->Add(experience.x_k.command_k[0]);
  rc_experience.mutable_command()->Add(experience.x_k.command_k[1]);
  rc_experience.set_store_time(experience.store_time.toNSec());
  rc_experience.set_transform_time(experience.transform_time.toNSec());

  // Check for sizes
  if (experience.gp_data.x_meas.rows() != DIST_DEP_SIZE) {
    LOG(ERROR) << "Tried to save features for the path tracker that are not the right size (9x1)!";
  }
  if (experience.gp_data.g_x_meas.rows() != STATE_SIZE) {
    LOG(ERROR) << "Tried to save output for the path tracker GP that is not the right size (3x1)!";
  }

  // add elements of the the GP input to the saved experience
  for (unsigned i = 0; i < experience.gp_data.x_meas.rows(); ++i) {
    rc_experience.mutable_gp_meas()->add_input(experience.gp_data.x_meas[i]);
  }
  // add elements of the gp output to the saved experience
  for (unsigned i = 0; i < experience.gp_data.g_x_meas.rows(); ++i) {
    rc_experience.mutable_gp_meas()->add_output(experience.gp_data.g_x_meas[i]);
  }

  return rc_experience;
}
#endif

void RCExperienceManagement::computeVelocitiesForExperienceKm1() {

  // Transform the robot poses
  tf::Transform T_km1_k = experience_km1_.T_0_v.inverse() * experience_k_.T_0_v;
  tf::Point p_km1_k_km1 = T_km1_k.getOrigin();
  tf::Transform C_km1_k(T_km1_k.getRotation());
  tf::Point xhat(1, 0, 0);
  tf::Point th_vec = C_km1_k * xhat;
  float th_k = atan2(th_vec.getY(), th_vec.getX());

  // Arrange the change in pose
  Eigen::VectorXf x_km1;
  x_km1 = Eigen::VectorXf::Zero(STATE_SIZE);

  Eigen::VectorXf x_k(STATE_SIZE);
  x_k << p_km1_k_km1.getX(), p_km1_k_km1.getY(), th_k;

  // Compute the change in time
  ros::Duration dt_ros = experience_k_.transform_time - experience_km1_.transform_time;
  auto d_t = (float) dt_ros.toSec();

  // Compute velocities
  if (d_t > 0.01) {
    computeVelocitiesFromState(experience_km1_.velocity_k, x_km1, x_k, d_t);
    experience_k_.x_k.velocity_km1 = experience_km1_.velocity_k;
    experience_km1_.velocity_is_valid = true;
  } else {
    // Pose estimate is not new, copy v and w from previous time
    experience_km1_.velocity_k = experience_km2_.velocity_k;
    experience_km1_.velocity_is_valid = false;
  }
}

void RCExperienceManagement::computeVelocitiesFromState(Eigen::VectorXf &velocity,
                                                        const Eigen::VectorXf &state_km1,
                                                        const Eigen::VectorXf &state_k,
                                                        const float d_t) {
  Eigen::VectorXf d_x = state_k - state_km1;

  velocity = Eigen::VectorXf::Zero(VELOCITY_SIZE);

  float v_pos = d_x.head<2>().norm() / d_t;

  velocity(0) = d_x(0) > 0 ? v_pos : -v_pos;
  velocity(1) = d_x(2) / d_t;
}

bool RCExperienceManagement::computeDisturbancesForExperienceKm2() {
  return nominal_model_.computeDisturbancesForExperienceKm2(experience_km2_, experience_km1_);
}

std::vector<Vid> RCExperienceManagement::getExperienceVertexList(const int &current_poseNum,
                                                                 const int &mpcSize,
                                                                 const std::vector<Vid> &vertexIds) {
  int numPoses = vertexIds.size();
  uint basisRegionStart = std::max(0, (int) current_poseNum - 5);
  uint basisRegionEnd = std::min((int) numPoses, (int) current_poseNum + mpcSize + 5 + 1);
  std::vector<Vid> vertex_list;
  vertex_list.reserve(basisRegionEnd - basisRegionStart);
  for (uint i = basisRegionStart; i < basisRegionEnd; i++) {
    vertex_list.push_back(vertexIds[i]);
  }
  return vertex_list;
}

std::vector<float> RCExperienceManagement::getExperienceSpeedList(const int &current_poseNum,
                                                                  const int &mpcSize,
                                                                  const std::vector<double> &scheduled_speed) {
  int numPoses = scheduled_speed.size();
  uint basisRegionStart = std::max(0, (int) current_poseNum - 5);
  uint basisRegionEnd = std::min((int) numPoses, (int) current_poseNum + mpcSize + 5 + 1);
  std::vector<float> speed_list;
  speed_list.reserve(basisRegionEnd - basisRegionStart);
  for (uint i = basisRegionStart; i < basisRegionEnd; i++) {
    speed_list.push_back(scheduled_speed[i]);
  }
  return speed_list;
}

#if 0
std::vector<MpcNominalModel::gp_data_t> RCExperienceManagement::getGpDataFromRCExperience(const int & current_pose_num,
                                                                                          const int & mpc_size,
                                                                                          const std::vector<Vid> & vertex_ids,
                                                                                          const std::vector<double> & speed_schedule,
                                                                                          const boost::unordered_map<Vid, double> &pathLengthByVertex,
                                                                                          const float & v_km1) {
  std::vector<Vid> vertex_list = getExperienceVertexList(current_pose_num,
                                                         mpc_size,
                                                         vertex_ids);

  std::vector<float> speed_list = getExperienceSpeedList(current_pose_num,
                                                         mpc_size,
                                                         speed_schedule);

  return getGpDataFromRCExperience(vertex_list,
                                   speed_list,
                                   pathLengthByVertex,
                                   v_km1);
}
#endif
}
} // vtr::path_tracker
