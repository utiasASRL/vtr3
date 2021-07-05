#include <vtr_navigation/map_projector.hpp>

namespace vtr {
namespace navigation {

const std::string MapProjector::pj_str_ =
    "+proj=utm +ellps=WGS84 +datum=WGS84 +units=m +no_defs +zone=";

MapProjector::MapProjector(const GraphPtr& graph,
                           const rclcpp::Node::SharedPtr node)
    : graph_(graph), node_(node), pool_(1, 1) {
  // clang-format off
  /// Publishers and services
  edge_updates_ = node_->create_publisher<EdgeMsg>("edge_updates", 5000);
  graph_updates_ = node_->create_publisher<UpdateMsg>("graph_updates", 5000);
  relaxation_service_ = node_->create_service<GraphRelaxSrv>("relaxed_graph", std::bind(&MapProjector::relaxGraphCallback, this, std::placeholders::_1, std::placeholders::_2));
  calibration_service_ = node_->create_service<GraphCalibSrv>("update_calib", std::bind(&MapProjector::updateCalibCallback, this, std::placeholders::_1, std::placeholders::_2));
  /// Parameters: default to UTIAS campus
  auto lat = node_->declare_parameter<double>("map_projection.origin_lat", 43.782207);
  auto lng = node_->declare_parameter<double>("map_projection.origin_lng", -79.466092);
  auto theta = node_->declare_parameter<double>("map_projection.origin_theta", 0.);
  auto scale = node_->declare_parameter<double>("map_projection.scale", 1.);
  // clang-format on

  /// Initialize the default map
  default_map_.root_vertex = root_;  // map root defaults to <0,0>
  default_map_.scale = scale;
  default_map_.utm_zone = uint32_t((lng + 180.) / 6.) + 1;
  // build projection
  PJ* pj_utm;
  auto pstr = pj_str_ + std::to_string(default_map_.utm_zone);
  if (!(pj_utm = proj_create(PJ_DEFAULT_CTX, pstr.c_str()))) {
    LOG(ERROR) << "[MapProjector] Could not build UTM projection";
  } else {
    PJ_COORD src, res;
    src.uv.u = proj_torad(lng);
    src.uv.v = proj_torad(lat);
    res = proj_trans(pj_utm, PJ_FWD, src);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<2, 2>() << std::cos(theta), std::sin(theta),
        -std::sin(theta), std::cos(theta);
    T.topRightCorner<2, 1>() << res.uv.u, res.uv.v;

    lgmath::se3::Transformation T_root_world(T);

    auto vec = T_root_world.vec();
    for (int i = 0; i < vec.size(); ++i)
      default_map_.t_map_vtr.entries.push_back(vec(i));
  }
  proj_destroy(pj_utm);

  /// Initialize cached response
  cached_response_.stamp = node_->now();
  cached_response_.seq = nextSeq();
  cached_response_.center.clear();
  cached_response_.center.push_back(lat);
  cached_response_.center.push_back(lng);

  initPoses();
  buildProjection();

  // If there is no graph, then no relaxation is a valid relaxation
  if (graph->numberOfVertices() == 0) {
    relaxation_valid_ = true;
    projection_valid_ = true;
  }
}

void MapProjector::runAdded(const RunPtr&) {
  LOG(DEBUG) << "[MapProjector] New run added.";
}

void MapProjector::vertexAdded(const VertexPtr& v) {
  LOG(DEBUG) << "[MapProjector] New vertex added.";

  /// The first vertex is added
  if (graph_.lock()->numberOfVertices() == 1) {
    buildProjection();

    VertexMsg msg;
    msg.id = v->id();
    msg.t_vertex_world = common::rosutils::toPoseMessage(TransformType());
    {
      std::lock_guard<std::mutex> lock(project_mutex_);
      project_(msg);
    }
    msg_map_.insert({v->id(), msg});

    tf_map_.insert({v->id(), TransformType()});

    // The first vertex is set to root, which should always be <0,0>
    root_ = v->id();

    updateProjection();

    /// Update cached response
    cached_response_.stamp = node_->now();
    cached_response_.seq = nextSeq();
    cached_response_.root_id = root_;
    cached_response_.active_branch.push_back(v->id());
  }
}

void MapProjector::edgeAdded(const EdgePtr& e) {
  LOG(DEBUG) << "[MapProjector] New edge added.";

  EdgeMsg new_edge;
  new_edge.from_id = e->from();
  new_edge.to_id = e->to();
  new_edge.type = e->idx();
  new_edge.manual = e->isManual();
  new_edge.t_to_from = common::rosutils::toTransformMessage(e->T());
  edge_updates_->publish(new_edge);

  if (incrementalRelax(e)) {
    projection_valid_ = false;
    LOG(INFO) << "[MapProjector] Addition of edge " << e->id()
              << " triggering relaxation.";

    change_lock_.lock();

    updateRelaxation();

    if (pool_.pending() > 1) {
      LOG(INFO) << "[MapProjector] Relaxation queued.";
    } else {
      LOG(INFO) << "[MapProjector] No active task; running relaxation now.";
    }

    change_lock_.unlock();
  }
}

void MapProjector::projectRobot(const tactic::Localization& persistent_loc,
                                const tactic::Localization& target_loc,
                                RobotStatusMsg& msg) {
  if (!project_robot_) {
    LOG(WARNING) << "Robot projector not initialized. Return {0,0,0}.";
    msg.lng_lat_theta = std::vector<double>({0, 0, 0});
    if (target_loc.localized) {
      msg.target_lng_lat_theta = std::vector<double>({0, 0, 0});
    }
  } else {
    std::lock_guard<std::mutex> lock(project_mutex_);
    msg.lng_lat_theta = project_robot_(persistent_loc.v, persistent_loc.T);
    if (target_loc.localized && target_loc.successes > 5) {
      msg.target_lng_lat_theta = project_robot_(target_loc.v, target_loc.T);
    }
  }
  /// create a copy so that we can reposition the robot after graph calibration
  /// and relaxation.
  cached_persistent_loc_ = persistent_loc;
  cached_target_loc_ = target_loc;
}

void MapProjector::updateProjection() {
  buildProjection();
  std::lock_guard<std::mutex> lock(project_mutex_);
  for (auto&& it : msg_map_) project_(it.second);
  projection_valid_ = true;
}

void MapProjector::updateRelaxation(const MutexPtr& mutex) {
  change_lock_.lock();

  auto shared_graph = getGraph();
  working_graph_ = shared_graph->getManualSubgraph();

  cached_response_.active_branch.clear();
  cached_response_.junctions.clear();
  cached_response_.components.clear();

  if (working_graph_->numberOfVertices() == 0) {
    change_lock_.unlock();
    LOG(INFO) << "[updateRelaxation] Empty graph; nothing to relax here...";
    return;
  }

  LOG(INFO) << "[updateRelaxation] Building decomposition...";

  /// Find and add junctions, paths, cycles to the cached_response.
  typename RCGraph::ComponentList paths, cycles;
  auto junctions = working_graph_->pathDecomposition(&paths, &cycles);
  for (auto&& it : junctions) cached_response_.junctions.push_back(it);
  for (auto&& it : paths) {
    ComponentMsg c;
    c.type = ComponentMsg::PATH;
    for (auto&& jt : it.elements()) c.vertices.push_back(jt);
    cached_response_.components.push_back(c);
  }
  for (auto&& it : cycles) {
    ComponentMsg c;
    c.type = ComponentMsg::CYCLE;
    for (auto&& jt : it.elements()) c.vertices.push_back(jt);
    cached_response_.components.push_back(c);
  }

  LOG(DEBUG) << "[updateRelaxation] Launching relaxation.";

  (void)pool_.try_dispatch([this, mutex]() {
    LOG(DEBUG) << "[updateRelaxation] In Relaxation Thread";

    // Typedef'd for now; eventually we will pull this from the graph
    Eigen::Matrix<double, 6, 6> cov(Eigen::Matrix<double, 6, 6>::Identity());
    cov.topLeftCorner<3, 3>() *= LINEAR_NOISE * LINEAR_NOISE;
    cov.bottomRightCorner<3, 3>() *= ANGLE_NOISE * ANGLE_NOISE;
    cov /= 10;

    auto graph = getGraph();

    // We use standard lock to acquire both locks in one operation, as the
    // program might otherwise deadlock.
    LOG(DEBUG) << "[Graph+Change Lock Requested] <relaxGraph>";
    std::lock(change_lock_, graph->mutex());
    LOG(DEBUG) << "[Graph+Change Lock Acquired] <relaxGraph>";

    // Take a static copy of the working graph once we begin executing
    auto frozenGraph(*working_graph_);

    // If we were supplied a mutex, lock it until we are done relaxation
    /// \todo this is no longer needed since steam has been made thread safe.
    /// For now, just in case the thread-safe version of steam is buggy.
    std::unique_lock<std::mutex> lck;
    if (mutex != nullptr) {
      LOG(DEBUG) << "[Steam Lock Requested] <relaxGraph>";
      lck = std::unique_lock<std::mutex>(*mutex);
      LOG(DEBUG) << "[Steam Lock Acquired] <relaxGraph>";
    }

    {
      // Build the relaxation problem while locked, so that the problem is
      // consistent with the frozen graph
      using EvalType = pose_graph::eval::Mask::Privileged<RCGraph>::Caching;
      pose_graph::GraphOptimizationProblem<RCGraph> relaxer(
          graph, root_, tf_map_, EvalType::MakeShared());

      relaxer.setLock(root_, true);

      relaxer.registerComponent(
          pose_graph::PoseGraphRelaxation<RCGraph>::MakeShared(cov));

      graph->unlock();
      LOG(DEBUG) << "[Graph Lock Released] <relaxGraph>";

      change_lock_.unlock();
      LOG(DEBUG) << "[Callback Lock Released] <relaxGraph>";

      // Solve things
      SolverType::Params params;
      params.verbose = true;
      // This is low, but just needs to stop us from trying to relax a perfect
      // chain
      params.absoluteCostThreshold = 1;
      relaxer.template optimize<SolverType>(params);

      LOG(DEBUG) << "[Callback Lock Requested] <relaxGraph>";
      change_lock_.lock();
      LOG(DEBUG) << "[Callback Lock Acquired] <relaxGraph>";

      LOG(INFO) << "[updateRelaxation] Rebuilding message map...";
      for (auto it = frozenGraph.beginVertex(), ite = frozenGraph.endVertex();
           it != ite; ++it) {
        tf_map_[it->id()] = relaxer.at(it->id());
        msg_map_[it->id()].t_vertex_world =
            common::rosutils::toPoseMessage(relaxer.at(it->id()));
        it->setTransform(relaxer.at(it->id()));
      }
    }

    // Update the active branch as it's root will have moved
    LOG(INFO) << "[updateRelaxation] Rebuilding active branch...";

    for (size_t ix = 1; ix < cached_response_.active_branch.size(); ++ix) {
      VertexId last = cached_response_.active_branch[ix - 1];
      VertexId current = cached_response_.active_branch[ix];

      auto e = graph->at(
          pose_graph::simple::SimpleEdge({uint64_t(last), uint64_t(current)}));
      TransformType T_new, T_edge = e->T();
      if (e->from() != last) {
        T_edge = T_edge.inverse();
      }

      T_new = T_edge * tf_map_[last];
      tf_map_[current] = T_new;
      msg_map_[current].t_vertex_world = common::rosutils::toPoseMessage(T_new);
      graph->at(current)->setTransform(T_new);
    }

    UpdateMsg msg;
    msg.invalidate = true;
    msg.seq = nextSeq();
    msg.stamp = node_->now();
    msg.vertices.clear();

    cached_response_.seq = nextSeq();
    cached_response_.stamp = node_->now();

    change_lock_.unlock();
    LOG(DEBUG) << "[Callback Lock Released] <relaxGraph>";

    relaxation_valid_ = true;
    LOG(INFO) << "[updateRelaxation] Done Relaxation.";

    graph_updates_->publish(msg);

    LOG(DEBUG) << "[Steam Lock Released] relaxGraph";
  });
  pool_.start();

  change_lock_.unlock();
  LOG(DEBUG) << "[Callback Lock Released] relaxGraph";
}

void MapProjector::initPoses() {
  auto shared_graph = getGraph();
  auto priv = shared_graph->getManualSubgraph();

  /// Populate map of messages for manual vertices
  for (auto it = priv->beginVertex(), ite = priv->endVertex(); it != ite;
       ++it) {
    VertexMsg msg;
    msg.id = it->id();
    msg.neighbours.clear();
    for (auto&& jt : priv->neighbors(it->id()))
      msg.neighbours.push_back(jt->id());

    msg_map_.insert({it->id(), msg});
  }

  if (shared_graph->contains(VertexId(0, 0))) {
    /// \todo note that root_ should always be <0, 0> so we should not have this
    /// assignment here. Need to enfore this somewhere. (e.g., in vertexAdded,
    /// when the first vertex is added, check that it is <0,0>).
    root_ = VertexId(0, 0);

    pose_graph::PrivilegedFrame<RCGraph> pf(shared_graph->begin(root_),
                                            shared_graph->end(), false);

    // Populate all transforms in case we relax again later
    for (auto it = shared_graph->beginVertex(); it != shared_graph->endVertex();
         ++it) {
      try {
        tf_map_[it->id()] = pf[it->id()];
      } catch (const std::runtime_error& error) {
        LOG_N_TIMES(1, WARNING) << "ignoring disconnected vertex: " << it->id();
      }
    }

    // Only populate mesages for manual vertices
    for (auto it = priv->beginVertex(); it != priv->endVertex(); ++it) {
      msg_map_[it->id()].t_vertex_world =
          common::rosutils::toPoseMessage(pf[it->id()]);
    }
  }
}

void MapProjector::buildProjection() {
  auto shared_graph = getGraph();

  if (!shared_graph->hasMap()) {
    LOG(INFO) << "[MapProjector] Initializing map info of the pose graph.";
    shared_graph->setMapInfo(default_map_);
    shared_graph->saveIndex();
  }

  TransformType T_root_world = Eigen::Matrix<double, 6, 1>(
      shared_graph->mapInfo().t_map_vtr.entries.data());
  /// \todo root is in fact always set to <0,0>. ensure this!
  root_ = VertexId(shared_graph->mapInfo().root_vertex);

  if (shared_graph->mapInfo().utm_zone > 0) {  // utm is 1-60, 0 means not set
    auto pstr = pj_str_ + std::to_string(default_map_.utm_zone);
    // delete existing projection (PJ) object
    if (pj_utm_ != nullptr) proj_destroy(pj_utm_);
    // build the new projection
    if (!(pj_utm_ = proj_create(PJ_DEFAULT_CTX, pstr.c_str()))) {
      std::lock_guard<std::mutex> lock(project_mutex_);
      project_ = [](VertexMsg&) {};
      project_robot_ = [](const VertexId&, const TransformType&) {
        return std::vector<double>({0, 0, 0});
      };
      projection_valid_ = false;
      std::string err = "[MapProjector] Could not build UTM projection";
      LOG(ERROR) << err;
      throw std::runtime_error{err};
    } else {
      auto scale = shared_graph->mapInfo().scale;
      std::lock_guard<std::mutex> lock(project_mutex_);
      project_ = [this, scale, T_root_world](VertexMsg& v) {
        Eigen::Matrix4d T_world_vertex(
            common::rosutils::fromPoseMessage(v.t_vertex_world).inverse());
        T_world_vertex.block<3, 1>(0, 3) =
            scale * T_world_vertex.block<3, 1>(0, 3);

        Eigen::Matrix4d T_global_vertex =
            T_root_world.matrix() * T_world_vertex;

        PJ_COORD src, res;
        src.uv.u = T_global_vertex(0, 3);
        src.uv.v = T_global_vertex(1, 3);
        res = proj_trans(pj_utm_, PJ_INV, src);

        v.t_projected.x = proj_todeg(res.uv.u);
        v.t_projected.y = proj_todeg(res.uv.v);
        v.t_projected.theta =
            std::atan2(T_global_vertex(1, 0), T_global_vertex(0, 0));

        LOG(DEBUG) << "[project_] vertex id: " << v.id
                   << ", x: " << v.t_projected.x << ", y: " << v.t_projected.y
                   << ", theta: " << v.t_projected.theta;
      };
      project_robot_ = [this, scale, T_root_world](
                           const VertexId& vid,
                           const TransformType& T_robot_vertex) {
        if (tf_map_.count(vid) == 0) {
          std::string err =
              "[MapProjector] Cannot find localization vertex id in tf map.";
          LOG(ERROR) << err;
          throw std::runtime_error{err};
        }

        Eigen::Matrix4d T_world_robot = tf_map_.at(vid).inverse().matrix() *
                                        T_robot_vertex.inverse().matrix();
        T_world_robot.block<3, 1>(0, 3) =
            scale * T_world_robot.block<3, 1>(0, 3);
        Eigen::Matrix4d T_global_robot = T_root_world.matrix() * T_world_robot;

        PJ_COORD src, res;
        src.uv.u = T_global_robot(0, 3);
        src.uv.v = T_global_robot(1, 3);
        res = proj_trans(pj_utm_, PJ_INV, src);

        auto lng = proj_todeg(res.uv.u);
        auto lat = proj_todeg(res.uv.v);
        auto theta = std::atan2(T_global_robot(1, 0), T_global_robot(0, 0));

        LOG(DEBUG) << "[project_robot_] robot live vertex: " << vid
                   << ", x: " << std::setprecision(12) << lng << ", y: " << lat
                   << ", theta: " << theta;

        return std::vector<double>({lng, lat, theta});
      };
    }
  } else {
    auto scale = shared_graph->mapInfo().scale;
    project_ = [scale, T_root_world](VertexMsg& v) {
      auto T_global_vertex =
          T_root_world.matrix() *
          common::rosutils::fromPoseMessage(v.t_vertex_world).inverse();

      v.t_projected.x = scale * T_global_vertex(0, 3);
      v.t_projected.y = scale * T_global_vertex(1, 3);
      v.t_projected.theta =
          std::atan2(T_global_vertex(1, 0), T_global_vertex(0, 0));
    };
    project_robot_ = [this, scale, T_root_world](
                         const VertexId& vid,
                         const TransformType& T_robot_vertex) {
      if (tf_map_.count(vid) == 0) {
        std::string err =
            "[MapProjector] Cannot find localization vertex id in tf map.";
        LOG(ERROR) << err;
        throw std::runtime_error{err};
      }
      auto T_vertex_world = tf_map_.at(vid);
      auto T_global_robot = T_root_world.matrix() *
                            T_vertex_world.inverse().matrix() *
                            T_robot_vertex.inverse().matrix();
      auto lng = scale * T_global_robot(0, 3);
      auto lat = scale * T_global_robot(1, 3);
      auto theta = std::atan2(T_global_robot(1, 0), T_global_robot(0, 0));

      return std::vector<double>({lng, lat, theta});
    };
  }
  /// Update cached response
  cached_response_.root_id = root_;
  cached_response_.projected = true;
}

bool MapProjector::incrementalRelax(const EdgePtr& e) {
  std::lock_guard<std::recursive_mutex> guard(change_lock_);

  if (e->isManual()) {
    VertexId from = e->from(), to = e->to();
    TransformType T = e->T();

    // Spatial edges are "backwards", in that the new vertex is e->from()
    if (e->type() == EdgeIdType::Type::Spatial) {
      from = e->to();
      to = e->from();
      T = e->T().inverse();
    }

    VertexMsg& v = msg_map_[to];
    v.id = to;
    VertexMsg& f = msg_map_[from];
    f.neighbours.push_back(v.id);

    auto shared_graph = getGraph();
    auto adj = shared_graph->at(v.id)->neighbours();
    v.neighbours = std::vector<uint64_t>(adj.begin(), adj.end());

    if (relaxation_valid_ || !pool_.isIdle()) {
      if (std::abs(int(uint64_t(e->to()) - uint64_t(e->from()))) == 1 ||
          (e->from().minorId() == 0 &&
           e->type() == EdgeIdType::Type::Spatial)) {
        if (cached_response_.active_branch.size() == 0) {
          cached_response_.active_branch.clear();
          cached_response_.active_branch.push_back(uint64_t(from));
        }

        if (uint64_t(from) == cached_response_.active_branch.back()) {
          v.t_vertex_world = common::rosutils::toPoseMessage(T * tf_map_[from]);
          {
            std::lock_guard<std::mutex> lock(project_mutex_);
            project_(v);
          }
          tf_map_[v.id] = T * tf_map_[from];

          UpdateMsg msg;
          msg.stamp = node_->now();  // ros::Time::now();
          msg.seq = nextSeq();
          msg.vertices.clear();
          msg.vertices.push_back(f);
          msg.vertices.push_back(v);
          graph_updates_->publish(msg);

          cached_response_.seq = msg.seq;
          cached_response_.active_branch.push_back(v.id);

          // Appended to chain; things are still valid
          return false;
        } else {
          // We didn't append to the active chain, and the chain wasnt empty; we
          // need to relax
          relaxation_valid_ = false;
          return true;
        }
      } else {
        // This edge was not between two vertices in the same run, or a branch
        //        relaxation_valid_ = false;
        //        return true;
        // TODO: triggering this relaxation breaks things right now because
        // steam isn't thread safe and we don't have a reference to the steam
        // mutex when this is triggered from a pose graph update...
        projection_valid_ = false;
        return false;
      }
    } else {
      // We don't have a valid relaxation, so we cannot append to it
      return true;
    }
  } else {
    // Autonomous edges do not trigger relaxation
    return false;
  }
}

void MapProjector::relaxGraphCallback(
    GraphRelaxSrv::Request::SharedPtr request,
    GraphRelaxSrv::Response::SharedPtr response) {
  LOG(INFO) << "Relaxation service called!";
  pool_.wait();

  if (relaxation_valid_) {
    if (projection_valid_ || !request->project) {
      LOG(INFO) << "[relaxGraph] Using cached graph.";
    } else {
      LOG(INFO) << "[relaxGraph] Reprojecting graph...";
      updateProjection();
    }
  } else {
    LOG(INFO) << "[relaxGraph] Relaxing graph...";
    updateRelaxation();
    pool_.wait();

    if (request->project) {
      LOG(INFO) << "[relaxGraph] Projecting graph...";
      updateProjection();
    }
  }

  std::lock_guard<std::recursive_mutex> lck(change_lock_);

  // If the last sequence the requester saw is the same or later than what we
  // have, don't replace anything
  if (cached_response_.seq && request->seq >= cached_response_.seq) {
    response->seq = -1;
    response->stamp = node_->now();
    response->root_id = cached_response_.root_id;
    return;
  }

  (*response) = cached_response_;

  response->stamp = node_->now();
  response->projected = request->project && projection_valid_;
  for (auto&& it : msg_map_) response->vertices.push_back(it.second);

  // Since we have updated the projection, also need to update the robot
  // location projected on graph.
  if (publisher_) {
    LOG(INFO)
        << "Update robot persistent and target loc after graph reprojection.";
    /// \note we never use the path_seq arg anywhere in the code currently, so
    /// just pass zero.
    /// \note we also assume that target loc is not needed after graph
    /// relaxation, so simply publish an invalid target localization.
    publisher_->publishRobot(cached_persistent_loc_);
  }
}

void MapProjector::updateCalibCallback(
    GraphCalibSrv::Request::SharedPtr request,
    GraphCalibSrv::Response::SharedPtr) {
  auto shared_graph = graph_.lock();
  if (!shared_graph)
    throw std::runtime_error(
        "[updateCalib] Attempted to update map info on an expired graph!");

  if (!shared_graph->hasMap()) {
    throw std::runtime_error(
        "[updateCalib] The pose graph does not have a map info set!");
#if false
    MapInfoMsg mapInfo;
    mapInfo.scale = 1.0;
    for (int i = 0; i < 6; ++i) mapInfo.t_map_vtr.entries.push_back(0.0);
    shared_graph->setMapInfo(mapInfo);
#endif
  }

  MapInfoMsg newInfo = shared_graph->mapInfo();
  TransformType T(
      Eigen::Matrix<double, 6, 1>(newInfo.t_map_vtr.entries.data()));

  // If theta > 0, modify the rotation
  if (std::abs(request->t_delta.theta) > 0) {
    Eigen::Matrix3d T_tmp = common::rosutils::fromPoseMessage(request->t_delta);
    Eigen::Matrix3d C = Eigen::Matrix3d::Identity();
    C.topLeftCorner<2, 2>() = T_tmp.topLeftCorner<2, 2>();

    // We do this by building the matrix directly so that we don't rotate the
    // rather large displacement vector twice
    Eigen::Matrix4d T_new = T.matrix();
    T_new.topLeftCorner<3, 3>() = C * T_new.topLeftCorner<3, 3>();
    T = TransformType(T_new);

    projection_valid_ = false;
    newInfo.t_map_vtr.entries.clear();
    auto vec = T.vec();

    for (int i = 0; i < 6; ++i) newInfo.t_map_vtr.entries.push_back(vec(i));
  }

  // Update offset if
  if (std::abs(request->t_delta.x) > 0 || std::abs(request->t_delta.y) > 0) {
    Eigen::Matrix4d T_new = T.matrix();

    // If we have a GPS zone, the update is assumed to be a lat/lon offset
    if (shared_graph->mapInfo().utm_zone > 0) {
      auto pstr = pj_str_ + std::to_string(default_map_.utm_zone);

      PJ* pj_utm;

      if (!(pj_utm = proj_create(PJ_DEFAULT_CTX, pstr.c_str()))) {
        LOG(ERROR) << "[MapProjector] Could not build UTM projection";
      } else {
        PJ_COORD src, res;  // projUV src, res;

        auto r_ab = T.r_ab_inb();

        // \todo (old) will this ever be used over large distances?  Projecting
        // through UTM and back may be overkill... Project original transform
        // offset from UTM --> GPS
        src.uv.u = r_ab(0);
        src.uv.v = r_ab(1);
        res = proj_trans(pj_utm, PJ_INV, src);

        // Add offset in GPS coordinates
        res.uv.u += proj_torad(request->t_delta.x);
        res.uv.v += proj_torad(request->t_delta.y);

        // Project back into UTM
        src = proj_trans(pj_utm, PJ_FWD, res);
        r_ab(0) = src.uv.u;
        r_ab(1) = src.uv.v;

        T_new.topRightCorner<3, 1>() = r_ab;
      }

      proj_destroy(pj_utm);
    }
    // Otherwise it is assumed to be a direct offset
    else {
      T_new(0, 3) += request->t_delta.x;
      T_new(1, 3) += request->t_delta.y;
    }

    T = TransformType(T_new);

    projection_valid_ = false;
    newInfo.t_map_vtr.entries.clear();
    auto vec = T.vec();

    for (int i = 0; i < 6; ++i) newInfo.t_map_vtr.entries.push_back(vec(i));
  }

  // Update map scaling
  if (std::abs(request->scale_delta) > 0) {
    newInfo.scale = newInfo.scale * request->scale_delta;
    projection_valid_ = false;
  }

  if (!projection_valid_) {
    shared_graph->setMapInfo(newInfo);
    shared_graph->saveIndex();
    buildProjection();

    /// Send a graph invalid message to UI so that the UI will request a full
    /// map update (i.e. call the relax graph service).
    UpdateMsg msg;
    msg.invalidate = true;
    msg.stamp = node_->now();
    msg.vertices.clear();
    msg.seq = nextSeq();

    cached_response_.seq = nextSeq();
    cached_response_.stamp = node_->now();

    graph_updates_->publish(msg);
  }
}

}  // namespace navigation
}  // namespace vtr