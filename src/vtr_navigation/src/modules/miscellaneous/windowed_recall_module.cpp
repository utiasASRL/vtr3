#include <vtr/navigation/modules/miscellaneous/windowed_recall_module.h>

#include <asrl/common/timing/SimpleTimer.hpp>
#include <asrl/vision/messages/bridge.hpp>

namespace vtr {
namespace navigation {

void WindowedRecallModule::run(QueryCache &qdata, MapCache &mdata,
                               const std::shared_ptr<const Graph> &graph) {
  vertex_landmarks_.clear();
  // Inputs: Graph, Window of Vertices
  // Outputs: Map of landmarkIds -> landmarks + observations
  //          Map of vertexIds -> poses
  //          Map of vertexIds -> T_s_v
  auto &lm_map = *mdata.landmark_map.fallback(5000);
  auto &poses = *mdata.pose_map.fallback();
  auto &transforms = *mdata.T_sensor_vehicle_map.fallback();
  auto &rig_names = *qdata.rig_names;

  // grab the latest vertex
  auto vertex = graph->at(*qdata.live_id);

  // set up a search for the previous keyframes in the graph
  TemporalEvaluatorPtr tempeval(new TemporalEvaluator());
  tempeval->setGraph((void *)graph.get());
  // only search backwards from the start_vid (which needs to be > the
  // landmark_vid)
  typedef asrl::pose_graph::Eval::Mask::DirectionFromVertexDirect<Graph>
      DirectionEvaluator;
  auto direval = std::make_shared<DirectionEvaluator>(*qdata.live_id, true);
  direval->setGraph((void *)graph.get());
  // combine the temporal and backwards mask
  auto evaluator = asrl::pose_graph::Eval::And(tempeval, direval);
  auto search_itr =
      graph->beginDfs(vertex->id(), config_->window_size - 1, evaluator);

  // add the latest vertex to the pose window
  poses[search_itr->v()->id()] = SteamPose(lgmath::se3::Transformation(), true);

  // Iterate through each vertex in the window, but stop if there are no more
  // backwards matches. No more backwards matches before we get to the end of
  // the graph search means the map was re-initialised This causes singularities
  // in optimisation where there are no factors between two poses
  for (; search_itr != graph->end() &&
         (poses.find(search_itr->v()->id()) != poses.end());
       ++search_itr) {
    // get current vertex.
    auto current_vertex = search_itr->v();
    // iterate through each rig
    // TODO: We should probably get the rig name from somewhere else other than
    // the query features.
    // TODO: Stored in graph perhaps??
    for (auto &rig_name : rig_names) {
      loadVertexData(lm_map, poses, transforms, current_vertex, rig_name,
                     graph);
    }
  }  // end for search_itr

  // Compute all of the poses in a single coordinate frame (First vertex in the
  // chain is identity)
  computePoses(poses, graph);

  // Load all the stored velocities
  getTimesandVelocities(poses, graph);
}

void WindowedRecallModule::loadVertexData(
    LandmarkMap &lm_map, SteamPoseMap &poses,
    SensorVehicleTransformMap &transforms,
    const asrl::pose_graph::RCVertex::Ptr &current_vertex,
    const std::string &rig_name, const std::shared_ptr<const Graph> &graph) {
  // unlock the pose associated with this vertex.
  poses[current_vertex->id()] = SteamPose(lgmath::se3::Transformation(), false);

  // load the T_s_v for this vertex
  loadSensorTransform(current_vertex->id(), transforms, rig_name, graph);

  // grab all of the observations associated with this vertex.
  auto observations =
      current_vertex->retrieveKeyframeData<asrl::vision_msgs::RigObservations>(
          "/" + rig_name + "/observations");
  if (observations == nullptr) {
    LOG(ERROR) << "Observations at " << current_vertex->id() << " for "
               << rig_name << " could not be loaded!";
  }
  // for each channel
  for (int channel_idx = 0; channel_idx < observations->channels().size();
       ++channel_idx) {
    // Grab the observations for this channel.
    auto *channel_obs = observations->mutable_channels()->Mutable(channel_idx);
    // make sure we actually have observations
    if (channel_obs->cameras().size() > 0) {
      loadLandmarksAndObs(lm_map, poses, transforms, current_vertex,
                          channel_obs, rig_name, graph);
    }  // end for cameras.size > 0
  }    // endfor channel
}

void WindowedRecallModule::loadLandmarksAndObs(
    LandmarkMap &lm_map, SteamPoseMap &poses,
    SensorVehicleTransformMap &transforms,
    const asrl::pose_graph::RCVertex::Ptr &current_vertex,
    asrl::vision_msgs::ChannelObservations *channel_obs,
    const std::string &rig_name, const std::shared_ptr<const Graph> &graph) {
  // grab the observations from the first camera.
  auto *camera_obs = channel_obs->mutable_cameras()->Mutable(0);

  // iterate through the landmarks associated with each observation.
  for (int lm_idx = 0; lm_idx < camera_obs->landmarks().size(); ++lm_idx) {
    // Get the rlandmark ID TODO: for multi-experience we need to go through
    // each reference and see if the run is actually in the graph, if not we
    // have to skip this landmark.
    const auto &lm_match = camera_obs->landmarks().Get(lm_idx).to().Get(0);

    // wrap it in a simple struct so it can be used as a key.
    // TODO: Write wrapper
    asrl::vision::LandmarkId id = asrl::messages::copyLandmarkId(lm_match);
    auto vid = graph->fromPersistent(lm_match.persistent());

    if (vid.majorId() != current_vertex->id().majorId()) {
      LOG(ERROR) << "Bad VO match "
                 << "from vid: " << current_vertex->id() << " "
                 << "to vid: " << vid << " "
                 << "from obs: "
                 << camera_obs->landmarks().Get(lm_idx).from().DebugString()
                 << " "
                 << "to lm: " << lm_match.DebugString() << " ";
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
        vertex_landmarks_[vid] =
            landmark_vertex
                ->retrieveKeyframeData<asrl::vision_msgs::RigLandmarks>(
                    "/" + rig_name + "/landmarks");
        if (vertex_landmarks_[vid] == nullptr) {
          LOG(ERROR) << "Couldn't retrieve landmarks from vertex data! Is the "
                        "vertex still unloaded?";
        }
      }

      // Get the pointer to the 3D point of this landmark.
      auto landmarks = vertex_landmarks_[landmark_vertex->id()];
      lm_map[id].point = landmarks->mutable_channels()
                             ->Mutable(id.channel)
                             ->mutable_points()
                             ->Mutable(id.index);

      // Get the pointer to the Descriptor of this landmark
      const auto &step_size = landmarks->mutable_channels()
                                  ->Mutable(id.channel)
                                  ->desc_type()
                                  .bytes_per_desc();
      const auto &descriptor_string =
          landmarks->mutable_channels()->Mutable(id.channel)->descriptors();
      lm_map[id].descriptor =
          (uint8_t *)&descriptor_string.data()[step_size * id.index];

      // Get the pointers to the other data elements
      lm_map[id].covariance = landmarks->mutable_channels()
                                  ->Mutable(id.channel)
                                  ->mutable_covariance();
      lm_map[id].num_vo_observations = landmarks->mutable_channels()
                                           ->Mutable(id.channel)
                                           ->mutable_num_vo_observations()
                                           ->Mutable(id.index);
      lm_map[id].valid = landmarks->mutable_channels()
                             ->Mutable(id.channel)
                             ->mutable_valid()
                             ->Mutable(id.index);
    }

    // add the observation of this landmark to the map.
    lm_map[id].observations.emplace_back(LandmarkObs());
    for (int cam_idx = 0; cam_idx < channel_obs->cameras().size(); ++cam_idx) {
      auto *kp = channel_obs->mutable_cameras()
                     ->Mutable(cam_idx)
                     ->mutable_keypoints()
                     ->Mutable(lm_idx);
      lm_map[id].observations.back().keypoints.push_back(kp);
      auto *ref = channel_obs->mutable_cameras()
                      ->Mutable(cam_idx)
                      ->mutable_landmarks()
                      ->Mutable(lm_idx);
      lm_map[id].observations.back().origin_ref = ref;
      auto *precision = channel_obs->mutable_cameras()
                            ->Mutable(cam_idx)
                            ->mutable_precisions()
                            ->Mutable(lm_idx);
      lm_map[id].observations.back().precisions.push_back(precision);
      auto *covariance = channel_obs->mutable_cameras()
                             ->Mutable(cam_idx)
                             ->mutable_covariances()
                             ->Mutable(lm_idx * 4);
      lm_map[id].observations.back().covariances.push_back(covariance);
    }  // end for cameras
  }    // and for landmark index.
}

void WindowedRecallModule::computePoses(
    SteamPoseMap &poses, const std::shared_ptr<const Graph> &graph) {
  // add the poses if they exist
  if (poses.empty() == false) {
    auto chain_start = poses.begin()->first;
    auto chain_end = poses.rbegin()->first;
    if (chain_start.majorId() != chain_end.majorId()) {
      LOG(WARNING) << __func__ << chain_start << " and " << chain_end
                   << " are not in the same run!!";
    }
    // always lock the first pose
    poses.begin()->second.setLock(true);
    if (chain_start != chain_end) {
      lgmath::se3::Transformation curr_pose;
      TemporalEvaluatorPtr tempeval(new TemporalEvaluator());
      tempeval->setGraph((void *)graph.get());
      typedef asrl::pose_graph::Eval::Mask::DirectionFromVertexDirect<Graph>
          DirectionEvaluator;
      auto direval = std::make_shared<DirectionEvaluator>(chain_start);
      direval->setGraph((void *)graph.get());
      auto evaluator = asrl::pose_graph::Eval::And(tempeval, direval);
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

void WindowedRecallModule::getTimesandVelocities(
    SteamPoseMap &poses, const std::shared_ptr<const Graph> &graph) {
  Eigen::Matrix<double, 6, 1> initVelocity;
  initVelocity.setZero();
  auto begin_vertex = graph->at(poses.begin()->first);
  auto begin_stamp = begin_vertex->keyFrameTime();
  for (auto &pose : poses) {
    // get the time for this vertex.
    auto vertex = graph->at(pose.first);
    auto stamp = vertex->keyFrameTime();
    auto proto_velocity =
        vertex->retrieveKeyframeData<robochunk::kinematic_msgs::Velocity>(
            "/velocities");

    if (proto_velocity == nullptr) {
      LOG(ERROR) << "Couldn't retrieve velocities from vertex data! Is the "
                    "vertex still unloaded?";
      return;
    }

    Eigen::Matrix<double, 6, 1> velocity;
    velocity(0, 0) = proto_velocity->translational().x();
    velocity(1, 0) = proto_velocity->translational().y();
    velocity(2, 0) = proto_velocity->translational().z();
    velocity(3, 0) = proto_velocity->rotational().x();
    velocity(4, 0) = proto_velocity->rotational().y();
    velocity(5, 0) = proto_velocity->rotational().z();

    pose.second.proto_velocity = proto_velocity;
    pose.second.time =
        steam::Time(static_cast<int64_t>(stamp.nanoseconds_since_epoch()));
    pose.second.setVelocity(velocity);
  }
}

void WindowedRecallModule::loadSensorTransform(
    const VertexId &vid, SensorVehicleTransformMap &transforms,
    const std::string &rig_name, const Graph::ConstPtr &graph) {
  // Check to see if the transform associated with this landmark is already
  // accounted for.
  if (transforms.find(vid) == transforms.end()) {
    // if not, we should try and load it
    // extract the T_s_v transform for this vertex
    auto map_vertex = graph->at(vid);
    auto rc_transforms =
        map_vertex->retrieveKeyframeData<robochunk::kinematic_msgs::Transform>(
            "/" + rig_name + "/T_sensor_vehicle");
    if (rc_transforms != nullptr) {  // check if we have the data. Some older
                                     // datasets may not have this saved
      Eigen::Matrix<double, 6, 1> tmp;
      auto mt = rc_transforms->mutable_translation();
      auto mr = rc_transforms->mutable_orientation();
      tmp << mt->x(), mt->y(), mt->z(), mr->x(), mr->y(), mr->z();
      transforms[vid] = lgmath::se3::TransformationWithCovariance(tmp);
    }
  }
  return;
}

void WindowedRecallModule::updateGraph(QueryCache &, MapCache &,
                                       const std::shared_ptr<Graph> &,
                                       VertexId) {
  /*steam::Time
  curr_time(static_cast<int64_t>(curr_stamp.nanoseconds_since_epoch())); auto
  curr_eval = trajectory_->getInterpPoseEval(curr_time); LOG(INFO) <<
  curr_eval->evaluate(); auto future_eval =
  trajectory_->getInterpPoseEval(curr_time + 1e9); LOG(INFO) << "Pose + 1
  second:" << "\n" << future_eval->evaluate() << "\n";*/
}

}  // namespace navigation
}  // namespace vtr
