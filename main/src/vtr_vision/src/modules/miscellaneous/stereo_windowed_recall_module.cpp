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
#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_vision/modules/miscellaneous/stereo_windowed_recall_module.hpp>

namespace vtr {

namespace vision {

using namespace tactic;

void StereoWindowedRecallModule::configFromROS(
    const rclcpp::Node::SharedPtr &node, const std::string param_prefix) {
  config_ = std::make_shared<Config>();
  // clang-format off
  config_->window_size = node->declare_parameter<int>(param_prefix + ".window_size", config_->window_size);
  // clang-format on
}

void StereoWindowedRecallModule::runImpl(QueryCache &qdata0,
                                         const Graph::ConstPtr &graph) {
  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);

  vertex_landmarks_.clear();
  // Inputs: Graph, Window of Vertices
  // Outputs: Map of landmarkIds -> landmarks + observations
  //          Map of vertexIds -> poses
  //          Map of vertexIds -> T_s_v
  auto &lm_map = *qdata.landmark_map.fallback(5000);
  auto &poses = *qdata.pose_map.fallback();
  auto &transforms = *qdata.T_sensor_vehicle_map.fallback();
  auto &rig_names = *qdata.rig_names;

  // create a frozen graph for traversal, since the graph may change during
  // traversal
  /// \todo runtime of copy scales linearly wrt to the size of the graph, better
  /// to get the required sub graph first here. Also needs to use the graph lock
  /// when getting the subgraph since graph may be changed during traversal.
  /// see map memory manager for an example. go back and change this when this
  /// module runs too slow.
  graph->lock();
  const auto frozen_graph = std::make_shared<GraphBase>(*graph);
  graph->unlock();

  CLOG(DEBUG, "stereo.windowed_recall") << "Created a frozen graph";

  // set up a search for the previous keyframes in the graph
  TemporalEvaluator::Ptr tempeval(new TemporalEvaluator());
  tempeval->setGraph((void *)frozen_graph.get());
  // only search backwards from the start_vid (which needs to be > the
  // landmark_vid)
  using DirectionEvaluator =
      pose_graph::eval::Mask::DirectionFromVertexDirect<GraphBase>;
  auto direval = std::make_shared<DirectionEvaluator>(*qdata.live_id, true);
  direval->setGraph((void *)frozen_graph.get());
  // combine the temporal and backwards mask
  auto evaluator = pose_graph::eval::And(tempeval, direval);
  // grab the latest vertex
  auto vertex = frozen_graph->at(*qdata.live_id);
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
    // iterate through each rig
    // TODO: We should probably get the rig name from somewhere else other than
    // the query features.
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
  auto observations =
      current_vertex->retrieveKeyframeData<vtr_messages::msg::RigObservations>(
          rig_name + "_observations");
  if (observations == nullptr) {
    std::stringstream err;
    err << "Observations at " << current_vertex->id() << " for " << rig_name
        << " could not be loaded!";
    CLOG(ERROR, "stereo.windowed_recall") << err.str();
    throw std::runtime_error{err.str()};
  }

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
    const vtr_messages::msg::ChannelObservations &channel_obs,
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
    auto vid = graph->fromPersistent(lm_match.persistent);

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
        graph->run(vid.majorId())
            ->registerVertexStream<vtr_messages::msg::RigLandmarks>(
                rig_name + "_landmarks", true,
                pose_graph::RegisterMode::Existing);
        vertex_landmarks_[vid] =
            landmark_vertex
                ->retrieveKeyframeData<vtr_messages::msg::RigLandmarks>(
                    rig_name + "_landmarks");
        if (vertex_landmarks_[vid] == nullptr) {
          std::stringstream err;
          err << "Couldn't retrieve landmarks from vertex data! Is the "
                 "vertex still unloaded?";
          CLOG(ERROR, "stereo.windowed_recall") << err.str();
          throw std::runtime_error{err.str()};
        }
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
      TemporalEvaluator::Ptr tempeval(new TemporalEvaluator());
      tempeval->setGraph((void *)graph.get());
      using DirectionEvaluator =
          pose_graph::eval::Mask::DirectionFromVertexDirect<Graph>;
      auto direval = std::make_shared<DirectionEvaluator>(chain_start);
      direval->setGraph((void *)graph.get());
      auto evaluator = pose_graph::eval::And(tempeval, direval);
      evaluator->setGraph((void *)graph.get());
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
  auto begin_stamp = begin_vertex->keyFrameTime();
#endif
  for (auto &pose : poses) {
    // get the time for this vertex.
    auto vertex = graph->at(pose.first);
    auto stamp = vertex->keyFrameTime();
    auto proto_velocity =
        vertex->retrieveKeyframeData<vtr_messages::msg::Velocity>(
            "_velocities");

    if (proto_velocity == nullptr) {
      std::stringstream err;
      err << "Couldn't retrieve velocities from vertex data! Is the "
             "vertex still unloaded?";
      CLOG(ERROR, "stereo.windowed_recall") << err.str();
      throw std::runtime_error{err.str()};
    }

    Eigen::Matrix<double, 6, 1> velocity;
    velocity(0, 0) = proto_velocity->translational.x;
    velocity(1, 0) = proto_velocity->translational.y;
    velocity(2, 0) = proto_velocity->translational.z;
    velocity(3, 0) = proto_velocity->rotational.x;
    velocity(4, 0) = proto_velocity->rotational.y;
    velocity(5, 0) = proto_velocity->rotational.z;

    pose.second.proto_velocity = proto_velocity;
    pose.second.time =
        steam::Time(static_cast<int64_t>(stamp.nanoseconds_since_epoch));
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
  auto rc_transforms =
      map_vertex->retrieveKeyframeData<vtr_messages::msg::Transform>(
          rig_name + "_T_sensor_vehicle");

  // check if we have the data. Some older datasets may not have this saved
  if (rc_transforms != nullptr) {
    Eigen::Matrix<double, 6, 1> tmp;
    auto mt = rc_transforms->translation;
    auto mr = rc_transforms->orientation;
    tmp << mt.x, mt.y, mt.z, mr.x, mr.y, mr.z;
    transforms[vid] = lgmath::se3::TransformationWithCovariance(tmp);
  }
}

void StereoWindowedRecallModule::updateGraphImpl(QueryCache &,
                                                 const Graph::Ptr &, VertexId) {
  /// steam::Time
  /// curr_time(static_cast<int64_t>(curr_stamp.nanoseconds_since_epoch()));
  /// auto curr_eval = trajectory_->getInterpPoseEval(curr_time);
  /// LOG(INFO) << curr_eval->evaluate();
  /// auto future_eval = trajectory_->getInterpPoseEval(curr_time + 1e9);
  /// LOG(INFO) << "Pose + 1 second:" << "\n" << future_eval->evaluate() <<
  /// "\n";
}

}  // namespace vision
}  // namespace vtr
