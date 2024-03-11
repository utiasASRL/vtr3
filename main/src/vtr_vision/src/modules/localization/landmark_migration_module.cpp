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
 * \file landmark_migration_module.cpp
 * \brief LandmarkMigrationModule class method definition
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <cmath>
#include <iomanip>
#include <iostream>

#include <vtr_common/timing/stopwatch.hpp>
#include <vtr_pose_graph/evaluator/evaluators.hpp>
#include <vtr_pose_graph/path/pose_cache.hpp>
#include <vtr_vision/types.hpp>
#include <vtr_vision/modules/localization/landmark_migration_module.hpp>

namespace vtr {
namespace vision {

using namespace tactic;

auto LandmarkMigrationModule::Config::fromROS(
    const rclcpp::Node::SharedPtr &node, const std::string &param_prefix) 
    -> ConstPtr {
      return std::make_shared<LandmarkMigrationModule::Config>();
    }

void LandmarkMigrationModule::run_(tactic::QueryCache &qdata0, tactic::OutputCache &output, const tactic::Graph::Ptr &graph,
                const std::shared_ptr<tactic::TaskExecutor> &) {

  auto &qdata = dynamic_cast<CameraQueryCache &>(qdata0);
  // check if the required data is in the cache
  if (!qdata.rig_features.valid()) {
    CLOG(ERROR, "stereo.migration") << "rig_features not present";
  }

  // sanity check
  if (!qdata.localization_map.valid()) {
    CLOG(ERROR,"stereo.migration") << "localization_map not present";
    return;
  } else if (!qdata.T_s_r.valid()) {
    CLOG(ERROR, "stereo.migration") << "T_s_r not present";
    return;
  } else if (!qdata.T_sensor_vehicle_map.valid()) {
    CLOG(ERROR, "stereo.migration") << "T_sensor_vehicle_map not present";
    return;
  } else if (!qdata.vid_loc.valid()) {
    CLOG(ERROR, "stereo.migration") << "map_id (vid_loc) not present";
    return;
  }

  // get the sub map.
  auto &sub_map = *qdata.localization_map;

  // TODO: Handle multiple rigs, where should this info come from??
  auto &rig_names = *qdata.rig_names;
  auto T_s_v_q = *qdata.T_s_r;
  auto &T_s_v_map = *qdata.T_sensor_vehicle_map;
  VertexId root_vid(*qdata.vid_loc);
  std::string rig_name = rig_names.at(0);
  int rig_idx = 0;
  initializeMapData(qdata);

  double load_time = 0;
  double migrate_time = 0;
  double search_time = 0;

  // get the T_s_v of the root vertex as this may be different, but default to
  // T_s_v_q
  lgmath::se3::Transformation T_s_v_r = T_s_v_q;

  // load the T_s_v for the root vertex ID
  loadSensorTransform(root_vid, T_s_v_map, rig_name, graph);

  // if successfully loaded, get it
  auto it = T_s_v_map.find(root_vid);
  if (it != T_s_v_map.end()) {
    T_s_v_r = it->second;
  } else {
    LOG_N_TIMES(1, WARNING)
        << "Couldn't find saved T_s_v for map vertex " << root_vid << std::endl
        << "This is likely OK if you are using a dataset from 2018 or earlier."
        << std::endl
        << "Support for non-static transforms was introduced in Github PR #469."
        << std::endl
        << "This message won't print again.";
  }

  // cache all the transforms so we only calculate them once
  pose_graph::PoseCache<pose_graph::RCGraph> pose_cache(graph, root_vid);

  // iterate through each vertex in the sub map.
  for (VertexId curr_vid : sub_map->subgraph().getNodeIds()) {
    // keep a time record
    common::timing::Stopwatch timer;

    // 1. get transform between the root and the current vertex, in the sensor
    // frame.

    // get the T_s_v of the map vertex as this may be different, but default to
    // T_s_v_q
    lgmath::se3::Transformation T_s_v_m = T_s_v_q;

    // load the T_s_v for the target vertex ID
    loadSensorTransform(curr_vid, T_s_v_map, rig_name, graph);

    // if successfully loaded, get it
    auto it = T_s_v_map.find(curr_vid);
    if (it != T_s_v_map.end()) {
      T_s_v_m = it->second;
    } else {
      CLOG(WARNING, "stereo.migration") << "Couldn't find T_s_v for vertex " << curr_vid;
    }

    EdgeTransform T_root_curr;
    try {
      T_root_curr =
          T_s_v_r * pose_cache.T_root_query(curr_vid) * T_s_v_m.inverse();
    } catch (std::exception &e) {
      /// \todo when would this happen?? Is this due to threading issue?
      CLOG(ERROR, "stereo.migration") << "Error migrating landmarks at " << curr_vid << ": "
                 << e.what();
      search_time += timer.count();
      continue;
    }

    if (!T_root_curr.covarianceSet()) {
      T_root_curr.setZeroCovariance();
    }

    search_time += timer.count();
    timer.reset();

    // 2. get landmarks
    std::string lm_stream_name = rig_name + "_landmarks";
    // for (const auto &r : graph->runs())
    //   r.second->registerVertexStream<vtr_vision_msgs::msg::RigLandmarks>(
    //       lm_stream_name, true, pose_graph::RegisterMode::Existing);

    auto curr_vertex = graph->at(curr_vid);

    //curr_vertex->load(lm_stream_name);
    auto locked_landmark_msg = curr_vertex->retrieve<vtr_vision_msgs::msg::RigLandmarks>(
            lm_stream_name, "vtr_vision_msgs/msg/RigLandmarks");
    auto locked_msg = locked_landmark_msg->sharedLocked();
    auto landmarks = locked_msg.get().getDataPtr();
    if (landmarks == nullptr) {
      std::stringstream err;
      err << "Landmarks at " << curr_vertex->id() << " for " << rig_name
          << " could not be loaded!";
      CLOG(ERROR, "stereo.migration") << err.str();
      throw std::runtime_error{err.str()};
    } else {
      CLOG(DEBUG, "stereo.migration") << "Landmarks loaded " << curr_vertex->id() << " for " << rig_name
          << "!";
    }
    load_time += timer.count();
    timer.reset();

    // 3. migrate the landmarks
    // auto persist_id = curr_vertex->persistentId();
    auto persist_id = curr_vid;

    migrate(rig_idx, persist_id, T_root_curr, qdata, landmarks);
    migrate_time += timer.count();
  }

  if (load_time + migrate_time + search_time >= 200) {
    auto num_vertices = sub_map->numberOfVertices();

    CLOG(WARNING, "stereo.migration") << std::setprecision(5) << " search time: " << search_time
                 << " ms ";
    CLOG(WARNING, "stereo.migration") << " load time: " << load_time << " ms ";
    CLOG(WARNING, "stereo.migration") << " migration time: " << migrate_time << " ms ";
    CLOG(WARNING, "stereo.migration") << " temporal depth: "
                 << (*qdata.localization_status).window_temporal_depth
                 << " total vertices: "
                 << (*qdata.localization_status).window_num_vertices;
    CLOG(WARNING, "stereo.migration") << " Avg load time " << load_time / num_vertices;
    CLOG(WARNING, "stereo.migration") << " Avg migrate time " << migrate_time / num_vertices;
  }

  // Get hnormalized migrated points.
  auto &migrated_points = *qdata.migrated_points;
  qdata.migrated_points_3d.emplace(migrated_points.colwise().hnormalized());

  // Get the motion prior, in the sensor frame.
  auto T_q_m_prior = T_s_v_q * (*qdata.T_r_m_prior) * T_s_v_r.inverse();
  auto &calibrations = *qdata.rig_calibrations;

  // Project the map points in the query camera frame using the prior
  vision::CameraIntrinsic &K = calibrations.front().intrinsics.at(0);
  qdata.projected_map_points.emplace(
      (K * T_q_m_prior.matrix().topLeftCorner(3, 4) * migrated_points)
          .colwise()
          .hnormalized());
}

void LandmarkMigrationModule::initializeMapData(CameraQueryCache &qdata) {
  // Outputs: migrated points, landmark<->point map.
  // How many landmarks do we think we'll have
  unsigned map_size = (*qdata.localization_map)->numberOfVertices();
  unsigned num_landmarks_est = std::min(20000u, 300u * map_size);
  // Pre-allocate points and covariance
  auto &migrated_points = *qdata.migrated_points.emplace(4, num_landmarks_est);
  auto &migrated_covariance =
      *qdata.migrated_covariance.emplace(9, num_landmarks_est);
  migrated_points.conservativeResize(Eigen::NoChange, 0);
  migrated_covariance.conservativeResize(Eigen::NoChange, 0);
  // Pre-allocate ids and map
  qdata.landmark_offset_map.emplace(num_landmarks_est);
  auto &migrated_landmark_ids = *qdata.migrated_landmark_ids.emplace();
  migrated_landmark_ids.reserve(num_landmarks_est);
  qdata.migrated_validity.emplace();
  qdata.migrated_validity->reserve(num_landmarks_est);
}

void LandmarkMigrationModule::migrate(
    const int &rig_idx, const VertexId &persist_id,
    const EdgeTransform &T_root_curr, CameraQueryCache &qdata,
    std::shared_ptr<vtr_vision_msgs::msg::RigLandmarks> &landmarks) {
  if (landmarks == nullptr) {
    CLOG(ERROR, "stereo.migration") << "Retrieved landmark is not valid";
    return;
  }
  // Outputs: migrated points, landmark<->point map.
  auto &migrated_points = *qdata.migrated_points;
  auto &migrated_validity = *qdata.migrated_validity;
  auto &migrated_covariance = *qdata.migrated_covariance;
  auto &landmark_offset_map = *qdata.landmark_offset_map;
  auto &migrated_landmark_ids = *qdata.migrated_landmark_ids;

  // 3. Iterate through each set of landmarks and transform the points.
  for (unsigned channel_idx = 0; channel_idx < landmarks->channels.size();
       ++channel_idx) {
    auto channel_landmarks = landmarks->channels[channel_idx];

    int matrix_offset = migrated_points.cols();
    // resize the matrix of migrated points to accomidate this batch of
    // landmarks.
    migrated_points.conservativeResize(
        Eigen::NoChange, matrix_offset + channel_landmarks.points.size());
    migrated_covariance.conservativeResize(
        Eigen::NoChange, matrix_offset + channel_landmarks.points.size());

    for (unsigned lm_idx = 0; lm_idx < channel_landmarks.points.size();
         ++lm_idx) {
      bool validity = true;
      if (channel_landmarks.valid.size() > 0) {
        // get the validity
        validity = channel_landmarks.valid[lm_idx];
      }

      // placeholder for migrated point
      Eigen::Vector4d migrated_point = Eigen::Vector3d::Zero().homogeneous();
      if (validity) {
        // Transform the point
        const auto &point_msg = channel_landmarks.points[lm_idx];
        Eigen::Vector3d point(point_msg.x, point_msg.y, point_msg.z);
        migrated_point = T_root_curr * point.homogeneous();
      } else {
        CLOG(WARNING, "stereo.migration") << "Point: " << lm_idx << " in " << channel_landmarks.name << " is invalid.";
      }

      // insert the migrated point
      migrated_points.col(lm_idx + matrix_offset) = migrated_point;

      // record the ID and validity
      migrated_landmark_ids.emplace_back(channel_landmarks.matches[lm_idx]);
      migrated_validity.push_back(validity);

      // TODO: (old) Move this into keyframe opt.
      // Transform the covariance
      namespace lgr3 = lgmath::r3;
      Eigen::Map<lgr3::CovarianceMatrix> migrated_cov(
          migrated_covariance.col(lm_idx + matrix_offset).data());
      if (validity && lm_idx * 9 < channel_landmarks.covariance.size()) {
        Eigen::Map<const Eigen::Matrix3f> covariance(
            &channel_landmarks.covariance[lm_idx * 9]);
        migrated_cov = lgr3::transformCovariance(
            T_root_curr, covariance.cast<double>(), migrated_point);
      } else {
        // note: this is only happening for vertex <0,0>. potential bug in VO.
        migrated_cov = Eigen::Matrix<double, 3, 3>::Identity();
      }
    }

    // Store off the channel offset in the map.
    vision::LandmarkId id;
    // id.persistent = messages::copyPersistentId(persist_id);
    // id.persistent.robot = persist_id.robot;
    // id.persistent.stamp = persist_id.stamp;
    id.vid = persist_id;
    id.rig = rig_idx;
    id.channel = channel_idx;
    landmark_offset_map[id] = matrix_offset;
  }
}

// recall the T_s_v for this vertex ID
void LandmarkMigrationModule::loadSensorTransform(
    const VertexId &vid, SensorVehicleTransformMap &transforms,
    const std::string &rig_name, const Graph::ConstPtr &graph) {
  // Check to see if the transform associated with this landmark is already
  // accounted for.
  if (transforms.find(vid) == transforms.end()) {
    // if not, we should try and load it
    // extract the T_s_v transform for this vertex
    std::string stream_name = rig_name + "_T_sensor_vehicle";


    auto map_vertex = graph->at(vid);

    auto locked_tf_msg = map_vertex->retrieve<vtr_common_msgs::msg::LieGroupTransform>(
            stream_name, "vtr_common_msgs/msg/LieGroupTransform");
    if (locked_tf_msg != nullptr) {
      auto locked_msg = locked_tf_msg->sharedLocked();
      auto rc_transforms = locked_msg.get().getDataPtr();
      common::conversions::fromROSMsg(*rc_transforms, transforms[vid]);
      transforms[vid].setZeroCovariance();
    }
  }
}

}  // namespace vision
}  // namespace vtr