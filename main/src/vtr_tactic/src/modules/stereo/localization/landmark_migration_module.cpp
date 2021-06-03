#include <cmath>
#include <iomanip>
#include <iostream>

#include <vtr_tactic/modules/stereo/localization/landmark_migration_module.hpp>

#include <vtr_common/timing/simple_timer.hpp>
#include <vtr_messages/msg/localization_status.hpp>
#include <vtr_messages/msg/transform.hpp>
#include <vtr_pose_graph/evaluator/accumulators.hpp>
#include <vtr_pose_graph/path/pose_cache.hpp>
#include <vtr_vision/messages/bridge.hpp>

namespace vtr {
namespace tactic {

void LandmarkMigrationModule::runImpl(QueryCache &qdata, MapCache &mdata,
                                      const Graph::ConstPtr &graph) {
  // check if the required data is in the cache
  if (!qdata.rig_features.is_valid()) {
    LOG(ERROR) << "rig_features not present";
  }

  // sanity check
  if (!qdata.localization_map.is_valid()) {
    LOG(ERROR) << "localization_map not present";
    return;
  } else if (!qdata.T_sensor_vehicle.is_valid()) {
    LOG(ERROR) << "T_sensor_vehicle not present";
    return;
  } else if (!qdata.T_sensor_vehicle_map.is_valid()) {
    LOG(ERROR) << "T_sensor_vehicle_map not present";
    return;
  } else if (!qdata.map_id.is_valid()) {
    LOG(ERROR) << "map_id not present";
    return;
  }

  // get the sub map.
  auto &sub_map = *qdata.localization_map;

  // TODO: Handle multiple rigs, where should this info come from??
  auto &rig_names = *qdata.rig_names;
  auto T_s_v_q = *qdata.T_sensor_vehicle;
  auto &T_s_v_map = *qdata.T_sensor_vehicle_map;
  VertexId root_vid(*qdata.map_id);
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
    common::timing::SimpleTimer timer;

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
      LOG(WARNING) << "Couldn't find T_s_v for vertex " << curr_vid;
    }

    // get the cached pose, in the coordinate frame of the sensor
    using namespace pose_graph::eval;
    typedef Mask::Typed<Graph>::Lambda LambdaEval;
    LambdaEval::VertexFunction veval = [&](const Vertex::Ptr &vp) {
      return vp->id().majorId() == curr_vid.majorId() ||
             graph->run(vp->id().majorId())->isManual();
    };
    Mask::Ptr mask = std::make_shared<Mask::Typed<Graph>::Lambda>(
        veval, const_cast<Graph *>(graph.get()));
    EdgeTransform T_root_curr;
    try {
      T_root_curr =
          T_s_v_r * pose_cache.T_root_query(curr_vid, mask) * T_s_v_m.inverse();
    } catch (std::exception &e) {
      LOG(ERROR) << "Error migrating landmarks at " << curr_vid << ": "
                 << e.what();
      search_time += timer.elapsedMs();
      continue;
    }

    if (!T_root_curr.covarianceSet()) {
      T_root_curr.setZeroCovariance();
    }

    search_time += timer.elapsedMs();
    timer.reset();

    // 2. get landmarks
    std::string lm_stream_name = rig_name + "_landmarks";
    for (const auto &r : graph->runs())
      r.second->registerVertexStream<vtr_messages::msg::RigLandmarks>(
          lm_stream_name, true, pose_graph::RegisterMode::Existing);

    auto curr_vertex = graph->at(curr_vid);

    curr_vertex->load(lm_stream_name);
    auto landmarks =
        curr_vertex->retrieveKeyframeData<vtr_messages::msg::RigLandmarks>(
            lm_stream_name);
    if (landmarks == nullptr) {
      std::stringstream err;
      err << "Landmarks at " << curr_vertex->id() << " for " << rig_name
          << " could not be loaded!";
      LOG(ERROR) << err.str();
      throw std::runtime_error{err.str()};
    }
    load_time += timer.elapsedMs();
    timer.reset();

    // 3. migrate the landmarks
    auto persist_id = curr_vertex->persistentId();
    migrate(rig_idx, persist_id, T_root_curr, qdata, landmarks);
    migrate_time += timer.elapsedMs();
  }

  if (load_time + migrate_time + search_time >= 200) {
    auto num_vertices = sub_map->numberOfVertices();

    LOG(WARNING) << std::setprecision(5) << " search time: " << search_time
                 << " ms ";
    LOG(WARNING) << " load time: " << load_time << " ms ";
    LOG(WARNING) << " migration time: " << migrate_time << " ms ";
    LOG(WARNING) << " temporal depth: "
                 << (*qdata.localization_status).window_temporal_depth
                 << " total vertices: "
                 << (*qdata.localization_status).window_num_vertices;
    LOG(WARNING) << " Avg load time " << load_time / num_vertices;
    LOG(WARNING) << " Avg migrate time " << migrate_time / num_vertices;
  }

  // Get hnormalized migrated points.
  auto &migrated_points = *qdata.migrated_points;
  qdata.migrated_points_3d.fallback(migrated_points.colwise().hnormalized());

  // Get the motion prior, in the sensor frame.
  auto T_q_m_prior = T_s_v_q * (*qdata.T_r_m_prior) * T_s_v_r.inverse();
  auto &calibrations = *qdata.rig_calibrations;

  // Project the map points in the query camera frame using the prior
  vision::CameraIntrinsic &K = calibrations.front().intrinsics.at(0);
  qdata.projected_map_points.fallback(
      (K * T_q_m_prior.matrix().topLeftCorner(3, 4) * migrated_points)
          .colwise()
          .hnormalized());
}

void LandmarkMigrationModule::initializeMapData(QueryCache &qdata) {
  // Outputs: migrated points, landmark<->point map.
  // How many landmarks do we think we'll have
  unsigned map_size = (*qdata.localization_map)->numberOfVertices();
  unsigned num_landmarks_est = std::min(20000u, 300u * map_size);
  // Pre-allocate points and covariance
  auto &migrated_points = *qdata.migrated_points.fallback(4, num_landmarks_est);
  auto &migrated_covariance =
      *qdata.migrated_covariance.fallback(9, num_landmarks_est);
  migrated_points.conservativeResize(Eigen::NoChange, 0);
  migrated_covariance.conservativeResize(Eigen::NoChange, 0);
  // Pre-allocate ids and map
  qdata.landmark_offset_map.fallback(num_landmarks_est);
  auto &migrated_landmark_ids = *qdata.migrated_landmark_ids.fallback();
  migrated_landmark_ids.reserve(num_landmarks_est);
  qdata.migrated_validity.fallback();
  qdata.migrated_validity->reserve(num_landmarks_est);
}

void LandmarkMigrationModule::migrate(
    const int &rig_idx, const vtr_messages::msg::GraphPersistentId &persist_id,
    const EdgeTransform &T_root_curr, QueryCache &qdata,
    std::shared_ptr<vtr_messages::msg::RigLandmarks> &landmarks) {
  if (landmarks == nullptr) {
    LOG(ERROR) << "Retrieved landmark is not valid";
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

    for (int lm_idx = 0; lm_idx < channel_landmarks.points.size(); ++lm_idx) {
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
    id.persistent = messages::copyPersistentId(persist_id);
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

    for (const auto &r : graph->runs())
      r.second->registerVertexStream<vtr_messages::msg::Transform>(
          stream_name, true, pose_graph::RegisterMode::Existing);

    auto map_vertex = graph->at(vid);
    map_vertex->load(stream_name);
    auto rc_transforms =
        map_vertex->retrieveKeyframeData<vtr_messages::msg::Transform>(
            stream_name);
    if (rc_transforms != nullptr) {
      Eigen::Matrix<double, 6, 1> tmp;
      auto mt = rc_transforms->translation;
      auto mr = rc_transforms->orientation;
      tmp << mt.x, mt.y, mt.z, mr.x, mr.y, mr.z;
      transforms[vid] = lgmath::se3::TransformationWithCovariance(tmp);
      transforms[vid]
          .setZeroCovariance();  // todo: add covariance field to message (?)
    }
  }
}

}  // namespace tactic
}  // namespace vtr
