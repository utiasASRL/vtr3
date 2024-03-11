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
 * \file stereo_windowed_recall_module.cpp
 * \brief StereoWindowedRecallModule class method definitions
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
//#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_vision/modules/bundle_adjustment/stereo_windowed_recall_module.hpp>

namespace vtr {

namespace vision {

using namespace tactic;

auto StereoWindowedRecallModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix) -> ConstPtr {
  auto config = std::make_shared<Config>();
  // clang-format off
  config->window_size = node->declare_parameter<int>(param_prefix + ".window_size", config->window_size);
  // clang-format on

  return config;
}

void StereoWindowedRecallModule::run_(QueryCache &qdata0, OutputCache &output, const Graph::Ptr &graph,
                const std::shared_ptr<TaskExecutor> &executor) {
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);
  CLOG(DEBUG, "stereo.windowed_recall") << "Start running StereoWindowedRecallModule";
  vertex_landmarks_.clear();
  // Inputs: Graph, Window of Vertices
  // Outputs: Map of landmarkIds -> landmarks + observations
  //          Map of vertexIds -> poses
  //          Map of vertexIds -> T_s_v
  auto &lm_map = *qdata.landmark_map.clear().emplace(5000);
  auto &poses = *qdata.pose_map.clear().emplace();
  auto &transforms = *qdata.T_sensor_vehicle_map.clear().emplace();
  auto &rig_names = *qdata.rig_names;
  auto vertex = graph->at(*qdata.vid_odo);


  // set up a search for the previous keyframes in the graph
  const auto tempeval = std::make_shared<TemporalEvaluator<GraphBase>>(*graph);

  // only search backwards from the start_vid (which needs to be > the
  // landmark_vid)
  using DirectionEvaluator = pose_graph::eval::mask::direction_from_vertex::Eval;

  auto direval = std::make_shared<DirectionEvaluator>(*qdata.vid_odo, true);
  // combine the temporal and backwards mask
  auto evaluator = pose_graph::eval::And(tempeval, direval);

  // const auto frozen_graph = graph->getSubgraph(vertex->id(), config_->window_size, evaluator);
  const auto frozen_graph = graph->getSubgraph(vertex->id(), vertex->id().minorId(), evaluator);
  CLOG(DEBUG, "stereo.windowed_recall") << "Created a frozen graph";


  auto search_itr =
      frozen_graph->beginDfs(vertex->id(), config_->window_size - 1, evaluator);

  // add the latest vertex to the pose window
  poses[search_itr->v()->id()] = SteamPose(lgmath::se3::Transformation(), true);
  // Iterate through each vertex in the window, but stop if there are no more
  // backwards matches. No more backwards matches before we get to the end of
  // the graph search means the map was re-initialised This causes singularities
  // in optimisation where there are no factors between two poses
  for (; search_itr != frozen_graph->end() &&
         (poses.find(search_itr->v()->id()) != poses.end());
       ++search_itr) {
    // get current vertex.
    auto current_vertex = search_itr->v();
    CLOG(DEBUG, "stereo.windowed_recall") << "\ncurrent vertex is:" << current_vertex->id();
    // iterate through each rig
    // TODO: Stored in graph perhaps??
    for (auto &rig_name : rig_names)
      loadVertexData(lm_map, poses, transforms, current_vertex, rig_name,
                     frozen_graph);

  }  // end for search_itr

  CLOG(DEBUG, "stereo.windowed_recall") << "Loaded vertex data";

  // Compute all of the poses in a single coordinate frame (First vertex in the
  // chain is identity)
  computePoses(poses, frozen_graph);

  CLOG(DEBUG, "stereo.windowed_recall") << "Finished computing poses";

  // Load all the stored velocities
  getTimesandVelocities(poses, frozen_graph);

  CLOG(DEBUG, "stereo.windowed_recall") << "Finished getting time and vels";
}

void StereoWindowedRecallModule::loadVertexData(
    LandmarkMap &lm_map, SteamPoseMap &poses,
    SensorVehicleTransformMap &transforms,
    const pose_graph::RCVertex::Ptr &current_vertex,
    const std::string &rig_name, const GraphBase::ConstPtr &graph) {
  // unlock the pose associated with this vertex.
  poses[current_vertex->id()] = SteamPose(lgmath::se3::Transformation(), false);

  // load the T_s_v for this vertex
  loadSensorTransform(current_vertex->id(), transforms, rig_name, graph);

  // grab all of the observations associated with this vertex.
  auto locked_obs_msgs =
      current_vertex->retrieve<vtr_vision_msgs::msg::RigObservations>(
          rig_name + "_observations", "vtr_vision_msgs/msg/RigObservations");
  if (locked_obs_msgs == nullptr){
    std::stringstream err;
    err << "Observations at " << current_vertex->id() << " for " << rig_name
        << " could not be loaded!";
    CLOG(ERROR, "stereo.windowed_recall") << err.str();
    throw std::runtime_error{err.str()};
  }
  auto locked_msg = locked_obs_msgs->sharedLocked();
  auto observations = locked_msg.get().getDataPtr();

  // for each channel
  for (uint32_t channel_idx = 0; channel_idx < observations->channels.size();
       ++channel_idx) {
    // Grab the observations for this channel.
    const auto &channel_obs = observations->channels[channel_idx];
    // make sure we actually have observations
    if (channel_obs.cameras.size() > 0)
      loadLandmarksAndObs(lm_map, poses, transforms, current_vertex,
                          channel_obs, rig_name, graph);
  }  // endfor channel
}

void StereoWindowedRecallModule::loadLandmarksAndObs(
    LandmarkMap &lm_map, SteamPoseMap &poses,
    SensorVehicleTransformMap &transforms,
    const pose_graph::RCVertex::Ptr &current_vertex,
    const vtr_vision_msgs::msg::ChannelObservations &channel_obs,
    const std::string &rig_name, const GraphBase::ConstPtr &graph) {
  // grab the observations from the first camera.
  const auto &camera_obs = channel_obs.cameras[0];

  // iterate through the landmarks associated with each observation.
  for (uint32_t lm_idx = 0; lm_idx < camera_obs.landmarks.size(); ++lm_idx) {
    // Get the rlandmark ID TODO: for multi-experience we need to go through
    // each reference and see if the run is actually in the graph, if not we
    // have to skip this landmark.
    const auto &lm_match = camera_obs.landmarks[lm_idx].to_id[0];

    // wrap it in a simple struct so it can be used as a key.
    // TODO: Write wrapper
    vision::LandmarkId id = messages::copyLandmarkId(lm_match);
    auto vid = VertexId(lm_match.vid);

    if (vid.majorId() != current_vertex->id().majorId()) {
      CLOG(ERROR, "stereo.windowed_recall")
          << "Bad VO match "
          << "from vid: " << current_vertex->id() << " "
          << "to vid: " << vid << " "
          << "from obs: " << camera_obs.landmarks[lm_idx].from_id.idx << " "
          << "to lm: " << lm_match.idx << " ";
    }

    // Check to see if the pose associated with this landmark is already
    // accounted for.
    if (poses.find(vid) == poses.end()) {
      // if it is not, then add it and set it to be locked.(if it is in the
      // window, it will be unlocked later.)
      poses[vid] = SteamPose(lgmath::se3::Transformation(), true);
    }

    // load the T_s_v for this vertex
    loadSensorTransform(vid, transforms, rig_name, graph);

    // if this is a new landmark, then add it to the map and load it up.
    if (lm_map.find(id) == lm_map.end()) {
      lm_map.insert({id, LandmarkInfo()});
      auto landmark_vertex = graph->at(vid);

      // if we havent loaded up this vertex yet, then do so.
      if (vertex_landmarks_.find(landmark_vertex->id()) ==
          vertex_landmarks_.end()) {
        auto locked_landmark_msg = landmark_vertex->retrieve<vtr_vision_msgs::msg::RigLandmarks>(
            rig_name + "_landmarks", "vtr_vision_msgs/msg/RigLandmarks");
        if (locked_landmark_msg == nullptr) {
          std::stringstream err;
          err << "Couldn't retrieve landmarks from vertex data! Is the "
                 "vertex still unloaded?";
          CLOG(ERROR, "stereo.windowed_recall") << err.str();
          throw std::runtime_error{err.str()};
        }
        auto locked_msg = locked_landmark_msg->sharedLocked();
        vertex_landmarks_[vid] = locked_msg.get().getDataPtr();
      }

      // Get the pointer to the 3D point of this landmark.
      auto landmarks = vertex_landmarks_[landmark_vertex->id()];
      lm_map[id].point = landmarks->channels[id.channel].points[id.index];

      // Get the pointer to the Descriptor of this landmark
      const auto &step_size =
          landmarks->channels[id.channel].desc_type.bytes_per_desc;
      const auto &descriptor_string =
          landmarks->channels[id.channel].descriptors;
      lm_map[id].descriptor =
          (uint8_t)descriptor_string.data()[step_size * id.index];

      // Get the pointers to the other data elements
      lm_map[id].covariance = landmarks->channels[id.channel].covariance;
      lm_map[id].num_vo_observations =
          landmarks->channels[id.channel].num_vo_observations[id.index];
      lm_map[id].valid = landmarks->channels[id.channel].valid[id.index];
    }

    // add the observation of this landmark to the map.
    lm_map[id].observations.emplace_back(LandmarkObs());
    for (unsigned cam_idx = 0; cam_idx < channel_obs.cameras.size();
         ++cam_idx) {
      auto &kp = channel_obs.cameras[cam_idx].keypoints[lm_idx];
      lm_map[id].observations.back().keypoints.push_back(kp);
      auto &ref = channel_obs.cameras[cam_idx].landmarks[lm_idx];
      lm_map[id].observations.back().origin_ref = ref;
      auto &precision = channel_obs.cameras[cam_idx].precisions[lm_idx];
      lm_map[id].observations.back().precisions.push_back(precision);
      // \todo yuchen Double check here! The original code uses pointer tricks.
      /// auto &covariance = channel_obs.cameras[cam_idx].covariances[lm_idx *
      /// 4];  // old code for ref.
      std::vector<float> covariance;
      for (int i = 0; i < 4; i++)
        covariance.push_back(
            channel_obs.cameras[cam_idx].covariances[lm_idx * 4 + i]);
      lm_map[id].observations.back().covariances.push_back(covariance);
    }  // end for cameras
  }    // and for landmark index.
}

void StereoWindowedRecallModule::computePoses(
    SteamPoseMap &poses, const GraphBase::ConstPtr &graph) {
  // add the poses if they exist
  if (poses.empty() == false) {
    auto chain_start = poses.begin()->first;
    auto chain_end = poses.rbegin()->first;
    if (chain_start.majorId() != chain_end.majorId()) {
      CLOG(ERROR, "stereo.windowed_recall")
          << chain_start << " and " << chain_end
          << " are not in the same run!!";
    }
    // always lock the first pose
    poses.begin()->second.setLock(true);
    if (chain_start != chain_end) {
      lgmath::se3::Transformation curr_pose;
      const auto tempeval = std::make_shared<TemporalEvaluator<GraphBase>>(*graph);
      
      using DirectionEvaluator = pose_graph::eval::mask::direction_from_vertex::Eval;

      auto direval = std::make_shared<DirectionEvaluator>(chain_start);
      auto evaluator = pose_graph::eval::And(tempeval, direval);
      auto itr = graph->begin(chain_start, 0, evaluator);
      itr++;
      for (; itr != graph->end(); ++itr) {
        curr_pose = itr->e()->T() * curr_pose;
        if (poses.find(itr->e()->to()) != poses.end()) {
          poses[itr->e()->to()].setTransform(curr_pose);
        } else {
          // we have missed one,
          poses[itr->e()->to()] = SteamPose(curr_pose, true);
        }
      }
    }
  }
}

void StereoWindowedRecallModule::getTimesandVelocities(
    SteamPoseMap &poses, const GraphBase::ConstPtr &graph) {
#if 0
  Eigen::Matrix<double, 6, 1> initVelocity;
  initVelocity.setZero();
  auto begin_vertex = graph->at(poses.begin()->first);
  auto begin_stamp = begin_vertex->vertexTime();
#endif
  for (auto &pose : poses) {
    // get the time for this vertex.
    auto vertex = graph->at(pose.first);
    auto stamp = vertex->vertexTime();

    auto locked_vel_msg = vertex->retrieve<VelocityMsg>(
            "velocities", "vtr_common_msgs/msg/Velocity");
    if (locked_vel_msg == nullptr) {
      std::stringstream err;
      err << "Couldn't retrieve velocity from vertex data!";
      CLOG(ERROR, "stereo.windowed_recall") << err.str();
      throw std::runtime_error{err.str()};
    }
    auto locked_msg = locked_vel_msg->sharedLocked();
    auto proto_velocity = locked_msg.get().getDataPtr();

    Eigen::Matrix<double, 6, 1> velocity;
    velocity(0, 0) = proto_velocity->translational.x;
    velocity(1, 0) = proto_velocity->translational.y;
    velocity(2, 0) = proto_velocity->translational.z;
    velocity(3, 0) = proto_velocity->rotational.x;
    velocity(4, 0) = proto_velocity->rotational.y;
    velocity(5, 0) = proto_velocity->rotational.z;

    pose.second.proto_velocity = proto_velocity;
    pose.second.time = steam::traj::Time(stamp);
    pose.second.setVelocity(velocity);
  }
}

void StereoWindowedRecallModule::loadSensorTransform(
    const VertexId &vid, SensorVehicleTransformMap &transforms,
    const std::string &rig_name, const GraphBase::ConstPtr &graph) {
  // Check to see if the transform associated with this landmark is already
  // accounted for.
  if (transforms.find(vid) != transforms.end()) return;

  // if not, we should try and load it
  // extract the T_s_v transform for this vertex
  auto map_vertex = graph->at(vid);

  auto locked_tf_msg = map_vertex->retrieve<vtr_common_msgs::msg::LieGroupTransform>(
            rig_name + "_T_sensor_vehicle", "vtr_common_msgs/msg/LieGroupTransform");
  if (locked_tf_msg != nullptr) {
    auto locked_msg = locked_tf_msg->sharedLocked();
    auto rc_transforms = locked_msg.get().getDataPtr();
    common::conversions::fromROSMsg(*rc_transforms, transforms[vid]);
    transforms[vid].setZeroCovariance();
  }
}


}  // namespace vision
}  // namespace vtr
