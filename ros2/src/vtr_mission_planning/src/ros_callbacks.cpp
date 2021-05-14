#include <vtr_common/rosutils/transformations.hpp>
#include <vtr_mission_planning/ros_callbacks.hpp>
#include <vtr_pose_graph/evaluator/common.hpp>
#include <vtr_pose_graph/relaxation/privileged_frame.hpp>

#define ANGLE_NOISE -M_PI / 16.0 / 6.0
#define LINEAR_NOISE 0.2 / 6.0

namespace vtr {
namespace mission_planning {

RosCallbacks::RosCallbacks(const GraphPtr& graph,
                           const std::shared_ptr<rclcpp::Node> node)
    : graph_(graph), node_(node), pool_(1, 1) {
  // cachedResponse_.stamp = ros::Time(0);
  cachedResponse_.stamp = node_->now();  // \todo yuchen Ok to use now?
  cachedResponse_.seq = _nextSeq();

  edgeUpdates_ = node_->create_publisher<EdgeMsg>("edge_updates", 5000);
  graphUpdates_ = node_->create_publisher<UpdateMsg>("graph_updates", 5000);
  // clang-format off
  relaxation_service_ = node_->create_service<GraphSrv>("relaxed_graph", std::bind(&RosCallbacks::_relaxGraphCallback, this, std::placeholders::_1, std::placeholders::_2));
  calibration_service_ = node_->create_service<GraphCalibSrv>("update_calib", std::bind(&RosCallbacks::_updateCalibCallback, this, std::placeholders::_1, std::placeholders::_2));
  // clang-format on

  // Default: UTIAS
  // double lat, lng, theta, scale;
  auto lat = node_->declare_parameter<double>("map.default.lat", 43.782207);
  auto lng = node_->declare_parameter<double>("map.default.lng", -79.466092);
  auto theta = node_->declare_parameter<double>("map.default.theta", 0.);
  auto scale = node_->declare_parameter<double>("map.default.scale", 1.);

  defaultMap_.root_vertex = root_;
  defaultMap_.scale = scale;
  defaultMap_.utm_zone = uint32_t((lng + 180.) / 6.) + 1;

  std::string pstr(
      "+proj=utm +ellps=WGS84 +datum=WGS84 +units=m +no_defs +zone=");
  pstr += std::to_string(defaultMap_.utm_zone);

  PJ* pj_utm;  // projPJ pj_utm;

  if (!(pj_utm = proj_create(PJ_DEFAULT_CTX, pstr.c_str()))) {
    LOG(ERROR) << "[RosCallbacks] Could not build UTM projection";
  } else {
    PJ_COORD src, res;  // projUV src, res;
    src.uv.u = proj_torad(lng);
    src.uv.v = proj_torad(lat);
    res = proj_trans(pj_utm, PJ_FWD, src);  // res = pj_fwd(src, pj_utm);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<2, 2>() << std::cos(theta), std::sin(theta),
        -std::sin(theta), std::cos(theta);
    T.topRightCorner<2, 1>() << res.uv.u, res.uv.v;

    lgmath::se3::Transformation T_root_world(T);

    auto vec = T_root_world.vec();
    for (int i = 0; i < vec.size(); ++i)
      defaultMap_.t_map_vtr.entries.push_back(vec(i));
  }

  proj_destroy(pj_utm);

  cachedResponse_.center.clear();
  cachedResponse_.center.push_back(lat);
  cachedResponse_.center.push_back(lng);

  _initPoses();
  _buildProjection();

  // If there is no graph, then no relaxation is a valid relaxation
  if (graph->numberOfVertices() == 0) {
    relaxationValid_ = true;
    projectionValid_ = true;
  }
}

void RosCallbacks::runAdded(const RunPtr& run) {
  LOG(DEBUG) << "New run added.";
}

void RosCallbacks::vertexAdded(const VertexPtr& v) {
  LOG(DEBUG) << "New vertex added.";

  if (graph_.lock()->numberOfVertices() == 1) {
    _buildProjection();

    VertexMsg msg;
    msg.id = v->id();
    msg.t_vertex_world = common::rosutils::toPoseMessage(TransformType());
    project_(msg);

    msgMap_.insert({msg.id, msg});

    if (root_ == VertexId::Invalid()) root_ = v->id();

    tfMap_.insert({v->id(), TransformType()});

    relaxationValid_ = true;
    updateProjection();

    cachedResponse_.stamp = node_->now();  // ros::Time::now();
    cachedResponse_.seq = _nextSeq();
    cachedResponse_.active_branch.push_back(v->id());
    cachedResponse_.root_id = root_;
  }
}

void RosCallbacks::edgeAdded(const EdgePtr& e) {
  LOG(DEBUG) << "New edge added.";

  EdgeMsg newEdge;
  newEdge.from_id = e->from();
  newEdge.to_id = e->to();
  newEdge.type = e->idx();
  newEdge.manual = e->isManual();
  newEdge.t_to_from = common::rosutils::toTransformMessage(e->T());
  edgeUpdates_->publish(newEdge);

  if (_incrementalRelax(e)) {
    projectionValid_ = false;
    LOG(INFO) << "[edgeAdded] Addition of edge " << e->id()
              << " triggering relaxation.";

    changeLock_.lock();

    updateRelaxation();

    if (pool_.pending() > 1) {
      LOG(INFO) << "[edgeAdded] Relaxation queued.";
    } else {
      LOG(INFO) << "[edgeAdded] No active task; running relaxation now.";
    }

    changeLock_.unlock();
  }
}

bool RosCallbacks::_incrementalRelax(const EdgePtr& e) {
  std::lock_guard<std::recursive_mutex> guard(changeLock_);

  if (e->isManual()) {
    VertexId from = e->from(), to = e->to();
    TransformType T = e->T();

    // Spatial edges are "backwards", in that the new vertex is e->from()
    if (e->type() == EdgeIdType::Type::Spatial) {
      from = e->to();
      to = e->from();
      T = e->T().inverse();
    }

    VertexMsg& v = msgMap_[to];
    v.id = to;
    VertexMsg& f = msgMap_[from];
    f.neighbours.push_back(v.id);

    auto sharedGraph = _getGraph();
    auto adj = sharedGraph->at(v.id)->neighbours();
    v.neighbours = std::vector<uint64_t>(adj.begin(), adj.end());

    if (relaxationValid_ || !pool_.isIdle()) {
      if (std::abs(int(uint64_t(e->to()) - uint64_t(e->from()))) ==
              1  // vtr3 change : add int() explicit type converson
          || (e->from().minorId() == 0 &&
              e->type() == EdgeIdType::Type::Spatial)) {
        if (cachedResponse_.active_branch.size() == 0) {
          cachedResponse_.active_branch.clear();
          cachedResponse_.active_branch.push_back(uint64_t(from));
        }

        if (uint64_t(from) == cachedResponse_.active_branch.back()) {
          v.t_vertex_world = common::rosutils::toPoseMessage(T * tfMap_[from]);
          project_(v);
          tfMap_[v.id] = T * tfMap_[from];

          UpdateMsg msg;
          msg.stamp = node_->now();  // ros::Time::now();
          msg.seq = _nextSeq();
          msg.vertices.clear();
          msg.vertices.push_back(f);
          msg.vertices.push_back(v);
          graphUpdates_->publish(msg);

          cachedResponse_.active_branch.push_back(v.id);
          cachedResponse_.seq = msg.seq;

          // Appended to chain; things are still valid
          return false;
        } else {
          // We didn't append to the active chain, and the chain wasnt empty; we
          // need to relax
          relaxationValid_ = false;
          return true;
        }
      } else {
        // This edge was not between two vertices in the same run, or a branch
        //        relaxationValid_ = false;
        //        return true;
        // TODO: triggering this relaxation breaks things right now because
        // steam isn't thread safe and we don't have a reference to the steam
        // mutex when this is triggered from a pose graph update...
        projectionValid_ = false;
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

void RosCallbacks::updateRelaxation(const MutexPtr& mutex) {
  changeLock_.lock();

  auto sharedGraph = _getGraph();
  workingGraph_ = sharedGraph->getManualSubgraph();

  cachedResponse_.active_branch.clear();
  cachedResponse_.junctions.clear();
  cachedResponse_.components.clear();

  if (workingGraph_->numberOfVertices() == 0) {
    changeLock_.unlock();
    LOG(INFO) << "[updateRelaxation] Empty graph; nothing to relax here...";
    return;
  }

  LOG(INFO) << "[updateRelaxation] Building decomposition...";

  typename RCGraph::ComponentList paths, cycles;
  auto junctions = workingGraph_->pathDecomposition(&paths, &cycles);

  for (auto&& it : junctions) cachedResponse_.junctions.push_back(it);

  for (auto&& it : paths) {
    ComponentMsg c;
    c.type = ComponentMsg::PATH;
    for (auto&& jt : it.elements()) c.vertices.push_back(jt);
    cachedResponse_.components.push_back(c);
  }

  for (auto&& it : cycles) {
    ComponentMsg c;
    c.type = ComponentMsg::CYCLE;
    for (auto&& jt : it.elements()) c.vertices.push_back(jt);
    cachedResponse_.components.push_back(c);
  }

  LOG(DEBUG) << "[updateRelaxation] Launching relaxation.";

  (void)pool_.try_dispatch([this, mutex]() {
    LOG(DEBUG) << "\t<updateRelaxation> In Relaxation Thread";

    // Typedef'd for now; eventually we will pull this from the graph
    Eigen::Matrix<double, 6, 6> cov(Eigen::Matrix<double, 6, 6>::Identity());
    cov.topLeftCorner<3, 3>() *= LINEAR_NOISE * LINEAR_NOISE;
    cov.bottomRightCorner<3, 3>() *= ANGLE_NOISE * ANGLE_NOISE;

    cov /= 10;  // \todo yuchen what is this magic number?

    auto graph = _getGraph();

    // We use standard lock to acquire both locks in one operation, as the
    // program might otherwise deadlock.
    LOG(DEBUG) << "[Graph+Change Lock Requested] <relaxGraph>";
    std::lock(changeLock_, graph->mutex());
    LOG(DEBUG) << "[Graph+Change Lock Acquired] <relaxGraph>";

    // Take a static copy of the working graph once we begin executing
    auto frozenGraph(*workingGraph_);

    // If we were supplied a mutex, lock it until we are done relaxation
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
          graph, root_, tfMap_, EvalType::MakeShared());

      relaxer.setLock(root_, true);

      relaxer.registerComponent(
          pose_graph::PoseGraphRelaxation<RCGraph>::MakeShared(cov));

      graph->unlock();
      LOG(DEBUG) << "[Graph Lock Released] <relaxGraph>";

      changeLock_.unlock();
      LOG(DEBUG) << "[Callback Lock Released] <relaxGraph>";

      // Solve things
      SolverType::Params params;
      params.verbose = true;
      params.absoluteCostThreshold =
          1;  // This is low, but just needs to stop us from trying to relax a
              // perfect chain
      relaxer.template optimize<SolverType>(params);

      LOG(DEBUG) << "[Callback Lock Requested] <relaxGraph>";
      changeLock_.lock();
      LOG(DEBUG) << "[Callback Lock Acquired] <relaxGraph>";

      LOG(INFO) << "\t<updateRelaxation> Rebuilding message map...";
      for (auto it = frozenGraph.beginVertex(), ite = frozenGraph.endVertex();
           it != ite; ++it) {
        tfMap_[it->id()] = relaxer.at(it->id());
        msgMap_[it->id()].t_vertex_world =
            common::rosutils::toPoseMessage(relaxer.at(it->id()));
        it->setTransform(relaxer.at(it->id()));
      }
    }

    // Update the active branch as it's root will have moved
    LOG(INFO) << "\t<updateRelaxation> Rebuilding active branch...";

    for (size_t ix = 1; ix < cachedResponse_.active_branch.size(); ++ix) {
      VertexId last = cachedResponse_.active_branch[ix - 1];
      VertexId current = cachedResponse_.active_branch[ix];

      auto e = graph->at(
          pose_graph::simple::SimpleEdge({uint64_t(last), uint64_t(current)}));
      TransformType T_new, T_edge = e->T();
      if (e->from() != last) {
        T_edge = T_edge.inverse();
      }

      T_new = T_edge * tfMap_[last];
      tfMap_[current] = T_new;
      msgMap_[current].t_vertex_world = common::rosutils::toPoseMessage(T_new);

      graph->at(current)->setTransform(T_new);
    }

    UpdateMsg msg;
    msg.invalidate = true;
    msg.stamp = node_->now();  // ros::Time::now();
    msg.vertices.clear();
    msg.seq = _nextSeq();

    cachedResponse_.seq = _nextSeq();
    cachedResponse_.stamp = node_->now();  // ros::Time::now();

    changeLock_.unlock();
    LOG(DEBUG) << "[Callback Lock Released] <relaxGraph>";

    relaxationValid_ = true;
    LOG(INFO) << "\t<updateRelaxation> Done Relaxation.";

    graphUpdates_->publish(msg);

    LOG(DEBUG) << "[Steam Lock Released] relaxGraph";
  });
  pool_.start();

  changeLock_.unlock();
  LOG(DEBUG) << "[Callback Lock Released] relaxGraph";
}

void RosCallbacks::_buildProjection() {
  auto sharedGraph = _getGraph();

  if (!sharedGraph->hasMap()) {
    LOG(WARNING) << "[_buildProjection] Falling back to default map "
                    "calibration since none was provided";
    sharedGraph->setMapInfo(defaultMap_);
    sharedGraph->saveIndex();
  }

  TransformType T_root_world = Eigen::Matrix<double, 6, 1>(
      sharedGraph->mapInfo().t_map_vtr.entries.data());
  root_ = VertexId(sharedGraph->mapInfo().root_vertex);
  cachedResponse_.root_id = root_;
  double scale = sharedGraph->mapInfo().scale;

  cachedResponse_.projected = true;

  if (sharedGraph->mapInfo().utm_zone > 0) {  // utm is 1-60, 0 means not set
    std::string pstr(
        "+proj=utm +ellps=WGS84 +datum=WGS84 +units=m +no_defs +zone=");
    pstr += std::to_string(sharedGraph->mapInfo().utm_zone);

    PJ* pj_utm;  // projPJ pj_utm;

    if (!(pj_utm = proj_create(PJ_DEFAULT_CTX, pstr.c_str()))) {
      project_ = [](VertexMsg&) {};
      projectionValid_ = false;
      LOG(ERROR) << "[_buildProjection] Could not build UTM projection";
    } else {
      project_ = [pj_utm, T_root_world](VertexMsg& v) {
        PJ_COORD src, res;  // projUV src, res;

        Eigen::Matrix4d T_global_vertex =
            T_root_world.matrix() *
            common::rosutils::fromPoseMessage(v.t_vertex_world).inverse();

        src.uv.u = T_global_vertex(0, 3);
        src.uv.v = T_global_vertex(1, 3);
        res = proj_trans(pj_utm, PJ_INV, src);

        v.t_projected.x = proj_todeg(res.uv.u);
        v.t_projected.y = proj_todeg(res.uv.v);

        // TODO: find a representative angle to use here
        v.t_projected.theta =
            std::atan2(T_global_vertex(1, 0), T_global_vertex(0, 0));
      };
    }
  } else {
    project_ = [scale, T_root_world](VertexMsg& v) {
      auto T_global_vertex =
          T_root_world.matrix() *
          common::rosutils::fromPoseMessage(v.t_vertex_world).inverse();

      v.t_projected.x = scale * T_global_vertex(0, 3);
      v.t_projected.y = scale * T_global_vertex(1, 3);

      // TODO: find a representative angle to use here
      v.t_projected.theta =
          std::atan2(T_global_vertex(1, 0), T_global_vertex(0, 0));
    };
  }
}

void RosCallbacks::_initPoses() {
  auto sharedGraph = _getGraph();
  auto priv = sharedGraph->getManualSubgraph();

  // Populate map of messages for manual vertices
  for (auto it = priv->beginVertex(), ite = priv->endVertex(); it != ite;
       ++it) {
    VertexMsg msg;
    msg.id = it->id();
    msg.neighbours.clear();

    for (auto&& jt : priv->neighbors(it->id()))
      msg.neighbours.push_back(jt->id());

    msgMap_.insert({it->id(), msg});
  }

  if (!root_.isSet() && sharedGraph->contains(VertexId(0, 0)))
    root_ = VertexId(0, 0);

  pose_graph::PrivilegedFrame<RCGraph> pf(sharedGraph->begin(root_),
                                          sharedGraph->end(), false);

  // Populate all transforms in case we relax again later
  for (auto it = sharedGraph->beginVertex(); it != sharedGraph->endVertex();
       ++it) {
    try {
      tfMap_[it->id()] = pf[it->id()];
    } catch (const std::runtime_error& error) {
      LOG_N_TIMES(1, WARNING) << "ignoring disconnected vertex: " << it->id();
    }
  }

  // Only populate mesages for manual vertices
  for (auto it = priv->beginVertex(); it != priv->endVertex(); ++it) {
    msgMap_[it->id()].t_vertex_world =
        common::rosutils::toPoseMessage(pf[it->id()]);
  }
}

void RosCallbacks::updateProjection() {
  _buildProjection();
  for (auto&& it : msgMap_) project_(it.second);
  projectionValid_ = true;
}

void RosCallbacks::_relaxGraphCallback(
    std::shared_ptr<GraphSrv::Request> request,
    std::shared_ptr<GraphSrv::Response> response) {
  LOG(INFO) << "Relaxation service called!";
  pool_.wait();

  if (relaxationValid_) {
    if (projectionValid_ || !request->project) {
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

  std::lock_guard<std::recursive_mutex> lck(changeLock_);

  // If the last sequence the requester saw is the same or later than what we
  // have, don't replace anything
  if (cachedResponse_.seq && request->seq >= cachedResponse_.seq) {
    response->seq = -1;

    response->stamp = node_->now();  // ros::Time::now();
    response->root_id = cachedResponse_.root_id;
    return;
  }

  (*response) = cachedResponse_;

  //  response.seq = seq_;  // yuchen: used to be commented out
  response->stamp = node_->now();  // ros::Time::now();
  response->projected = request->project && projectionValid_;
  for (auto&& it : msgMap_) response->vertices.push_back(it.second);
}

void RosCallbacks::_updateCalibCallback(
    std::shared_ptr<GraphCalibSrv::Request> request,
    std::shared_ptr<GraphCalibSrv::Response>) {
  auto sharedGraph = graph_.lock();
  if (!sharedGraph)
    throw std::runtime_error(
        "[updateCalib] Attempted to update map info on an expired graph!");

  if (!sharedGraph->hasMap()) {
    MapInfoMsg mapInfo;
    mapInfo.scale = 1.0;
    for (int i = 0; i < 6; ++i) mapInfo.t_map_vtr.entries.push_back(0.0);

    sharedGraph->setMapInfo(mapInfo);
  }

  MapInfoMsg newInfo = sharedGraph->mapInfo();
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

    projectionValid_ = false;
    newInfo.t_map_vtr.entries.clear();
    auto vec = T.vec();

    for (int i = 0; i < 6; ++i) newInfo.t_map_vtr.entries.push_back(vec(i));
  }

  // Update offset if
  if (std::abs(request->t_delta.x) > 0 || std::abs(request->t_delta.y) > 0) {
    Eigen::Matrix4d T_new = T.matrix();

    // If we have a GPS zone, the update is assumed to be a lat/lon offset
    if (sharedGraph->mapInfo().utm_zone > 0) {
      std::string pstr(
          "+proj=utm +ellps=WGS84 +datum=WGS84 +units=m +no_defs +zone=");
      pstr += std::to_string(sharedGraph->mapInfo().utm_zone);

      PJ* pj_utm;  // projPJ pj_utm;

      if (!(pj_utm = proj_create(PJ_DEFAULT_CTX, pstr.c_str()))) {
        LOG(ERROR) << "[RosCallbacks] Could not build UTM projection";
      } else {
        PJ_COORD src, res;  // projUV src, res;

        auto r_ab = T.r_ab_inb();

        // \todo (old) will this ever be used over large distances?  Projecting
        // through UTM and back may be overkill... Project original transform
        // offset from UTM --> GPS
        src.uv.u = r_ab(0);
        src.uv.v = r_ab(1);
        res = proj_trans(pj_utm, PJ_INV, src);  // res = pj_fwd(src, pj_utm);

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

    projectionValid_ = false;
    newInfo.t_map_vtr.entries.clear();
    auto vec = T.vec();

    for (int i = 0; i < 6; ++i) newInfo.t_map_vtr.entries.push_back(vec(i));
  }

  // Update map scaling
  if (std::abs(request->scale_delta) > 0) {
    newInfo.scale = newInfo.scale * request->scale_delta;
    projectionValid_ = false;
  }

  if (!projectionValid_) {
    sharedGraph->setMapInfo(newInfo);
    sharedGraph->saveIndex();
    _buildProjection();

    UpdateMsg msg;
    msg.invalidate = true;
    msg.stamp = node_->now();
    msg.vertices.clear();
    msg.seq = _nextSeq();

    cachedResponse_.seq = _nextSeq();
    cachedResponse_.stamp = node_->now();

    graphUpdates_->publish(msg);
  }
}

}  // namespace mission_planning
}  // namespace vtr
