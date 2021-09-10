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
 * \file map_projector.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
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
  calibration_service_ = node_->create_service<GraphCalibSrv>("update_calib", std::bind(&MapProjector::updateCalibCallback, this, std::placeholders::_1, std::placeholders::_2));
  relaxation_service_ = node_->create_service<GraphRelaxSrv>("relaxed_graph", std::bind(&MapProjector::relaxGraphCallback, this, std::placeholders::_1, std::placeholders::_2));
  graph_pinning_service_ = node_->create_service<GraphPinningSrv>("pin_graph", std::bind(&MapProjector::pinGraphCallback, this, std::placeholders::_1, std::placeholders::_2));
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
  // pin of the first root vertex (always set), lat lng may change
  GraphPinMsg root_pin;
  root_pin.id = root_;
  root_pin.lat = lat;
  root_pin.lng = lng;
  root_pin.weight = -1;
  default_map_.pins.clear();
  default_map_.pins.push_back(root_pin);
  // build projection to set t_map_vtr
  PJ* pj_utm;
  auto pstr = pj_str_ + std::to_string(default_map_.utm_zone);
  if (!(pj_utm = proj_create(PJ_DEFAULT_CTX, pstr.c_str()))) {
    std::string err{"[MapProjector] Could not build UTM projection"};
    CLOG(ERROR, "map_projector") << err;
    throw std::runtime_error{err};
  } else {
    PJ_COORD src, res;
    src.uv.u = proj_torad(lng);
    src.uv.v = proj_torad(lat);
    res = proj_trans(pj_utm, PJ_FWD, src);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<2, 2>() << std::cos(theta), std::sin(theta),
        -std::sin(theta), std::cos(theta);
    T.topRightCorner<2, 1>() << res.uv.u, res.uv.v;

    lgmath::se3::Transformation T_map_root(T);
    const auto vec = T_map_root.vec();
    for (int i = 0; i < vec.size(); ++i)
      default_map_.t_map_vtr.entries.push_back(vec(i));
  }
  proj_destroy(pj_utm);

  auto shared_graph = getGraph();
  if (!shared_graph->hasMap()) {
    CLOG(INFO, "map_projector")
        << "[MapProjector] Initializing map info of the pose graph.";
    shared_graph->setMapInfo(default_map_);
    shared_graph->saveIndex();
  }

  /// Initialize cached response
  cached_response_.stamp = node_->now();
  cached_response_.seq = nextSeq();
  cached_response_.center.clear();
  cached_response_.center.push_back(lat);
  cached_response_.center.push_back(lng);
  cached_response_.pins = default_map_.pins;  // might not be the best
                                              // initialization but this ensures
                                              // that the UI always shows at
                                              // least the root pin

  initPoses();
  buildProjection();

  // If there is no graph, then no relaxation is a valid relaxation
  if (graph->numberOfVertices() == 0) {
    relaxation_valid_ = true;
    projection_valid_ = true;
  }
}

void MapProjector::runAdded(const RunPtr&) {
  CLOG(DEBUG, "map_projector") << "[MapProjector] New run added.";
}

void MapProjector::vertexAdded(const VertexPtr& v) {
  CLOG(DEBUG, "map_projector") << "[MapProjector] New vertex added " << v->id();

  /// The first vertex is added
  if (graph_.lock()->numberOfVertices() == 1) {
    buildProjection();

    // The first vertex is set to root, which should always be <0,0>
    root_ = v->id();

    VertexMsg msg;
    msg.id = v->id();
    msg.t_vertex_world = common::rosutils::toPoseMessage(TransformType());
    {
      std::lock_guard<std::mutex> lock(project_mutex_);
      project_(msg);
    }
    msg_map_.insert({v->id(), msg});

    tf_map_.insert({v->id(), TransformType()});

    updateProjection();

    /// Update cached response
    cached_response_.stamp = node_->now();
    cached_response_.seq = nextSeq();
    cached_response_.root_id = root_;
    cached_response_.active_branch.push_back(v->id());
  }
}

void MapProjector::edgeAdded(const EdgePtr& e) {
  CLOG(DEBUG, "map_projector") << "[MapProjector] New edge added from "
                               << e->from() << " to " << e->to();

  EdgeMsg new_edge;
  new_edge.from_id = e->from();
  new_edge.to_id = e->to();
  new_edge.type = e->idx();
  new_edge.manual = e->isManual();
  new_edge.t_to_from = common::rosutils::toTransformMessage(e->T());
  edge_updates_->publish(new_edge);

  if (incrementalRelax(e)) {
    projection_valid_ = false;
    CLOG(INFO, "map_projector") << "[MapProjector] Addition of edge " << e->id()
                                << " triggering relaxation.";

    change_lock_.lock();

    updateRelaxation();

    if (pool_.pending() > 1) {
      CLOG(INFO, "map_projector") << "[MapProjector] Relaxation queued.";
    } else {
      CLOG(INFO, "map_projector")
          << "[MapProjector] No active task; running relaxation now.";
    }

    change_lock_.unlock();
  }
}

void MapProjector::projectRobot(const tactic::Localization& persistent_loc,
                                const tactic::Localization& target_loc,
                                RobotStatusMsg& msg) {
  if (!project_robot_) {
    CLOG(WARNING, "map_projector")
        << "Robot projector not initialized. Return {0,0,0}.";
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

void MapProjector::updateRelaxation() {
  change_lock_.lock();

  auto shared_graph = getGraph();
  working_graph_ = shared_graph->getManualSubgraph();

  /// Clear current cached response entries
  cached_response_.active_branch.clear();
  cached_response_.junctions.clear();
  cached_response_.components.clear();

  /// \note Currently getManualSubgraph returns empty when there only one vertex
  /// in the graph.
  if (working_graph_->numberOfVertices() == 0) {
    change_lock_.unlock();
    CLOG(INFO, "map_projector")
        << "[updateRelaxation] Empty graph; nothing to relax here...";
    relaxation_valid_ = true;
    return;
  }

  CLOG(INFO, "map_projector") << "[updateRelaxation] Building decomposition...";

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

  /// Also update the list of pins (mostly for the initial call right after
  /// launching vtr)
  cached_response_.pins = shared_graph->mapInfo().pins;

  CLOG(DEBUG, "map_projector") << "[updateRelaxation] Launching relaxation.";

  (void)pool_.try_dispatch([this]() {
    CLOG(DEBUG, "map_projector") << "[updateRelaxation] In Relaxation Thread";

    /// \todo Typedef'd for now; eventually we will pull this from the graph
    Eigen::Matrix<double, 6, 6> cov(Eigen::Matrix<double, 6, 6>::Identity());
    cov.topLeftCorner<3, 3>() *= LINEAR_NOISE * LINEAR_NOISE;
    cov.bottomRightCorner<3, 3>() *= ANGLE_NOISE * ANGLE_NOISE;
    cov /= 10;

    auto graph = getGraph();

    // We use standard lock to acquire both locks in one operation, as the
    // program might otherwise deadlock.
    CLOG(DEBUG, "map_projector")
        << "[Graph+Change Lock Requested] <relaxGraph>";
    std::lock(change_lock_, graph->mutex());
    CLOG(DEBUG, "map_projector") << "[Graph+Change Lock Acquired] <relaxGraph>";

    // Take a static copy of the working graph once we begin executing
    auto frozenGraph(*working_graph_);

    {
      // Build the relaxation problem while locked, so that the problem is
      // consistent with the frozen graph
      using EvalType = pose_graph::eval::Mask::Privileged<RCGraph>::Caching;
      pose_graph::GraphOptimizationProblem<RCGraph> relaxer(
          graph, root_, tf_map_, EvalType::MakeShared());

      relaxer.setLock(root_, true);

      relaxer.registerComponent(
          pose_graph::PoseGraphRelaxation<RCGraph>::MakeShared(cov));

      /// Add pins to the pose graph optimization problem
      // If we have a GPS zone, the update is assumed to be a lat/lon offset
      if (graph->mapInfo().utm_zone > 0) {
        auto pstr = pj_str_ + std::to_string(default_map_.utm_zone);

        PJ* pj_utm;

        if (!(pj_utm = proj_create(PJ_DEFAULT_CTX, pstr.c_str()))) {
          std::string err = "[MapProjector] Could not build UTM projection";
          CLOG(ERROR, "map_projector") << err;
          throw std::runtime_error{err};
        } else {
          PJ_COORD src, res;  // projUV src, res;

          TransformType T_map_root(Eigen::Matrix<double, 6, 1>(
              graph->mapInfo().t_map_vtr.entries.data()));
          TransformType T_root_map = T_map_root.inverse();

          auto scale = graph->mapInfo().scale;

          for (const auto& pin : graph->mapInfo().pins) {
            src.uv.u = proj_torad(pin.lng);
            src.uv.v = proj_torad(pin.lat);
            res = proj_trans(pj_utm, PJ_FWD, src);

            // Transform into vtr frame (i.e. frame of root vertex)
            Eigen::Vector4d r_vertex_map_in_map;
            r_vertex_map_in_map << res.uv.u, res.uv.v, 0, 1;
            Eigen::Vector4d r_vertex_root_in_root =
                T_root_map * r_vertex_map_in_map;
            // apply scale
            r_vertex_root_in_root = r_vertex_root_in_root / scale;

            // Add the corresponding cost term
            /// \note weight < 0 means user did not set weight for this pin, so
            /// we use a sufficiently large weight instead
            relaxer.registerComponent(
                pose_graph::PGRVertexPinPrior<RCGraph>::MakeShared(
                    VertexId(pin.id), r_vertex_root_in_root.block<2, 1>(0, 0),
                    pin.weight > 0 ? pin.weight : 1000000));
          }
        }
        proj_destroy(pj_utm);
      } else {
        std::string err =
            "[MapProjector] Map info does not have a valid UTM zone, meaning "
            " we are not using the satellite map. Currently not supported.";
        CLOG(ERROR, "map_projector") << err;
        throw std::runtime_error{err};
      }

      graph->unlock();
      CLOG(DEBUG, "map_projector") << "[Graph Lock Released] <relaxGraph>";

      change_lock_.unlock();
      CLOG(DEBUG, "map_projector") << "[Callback Lock Released] <relaxGraph>";

      // Solve things
      SolverType::Params params;
      params.verbose = true;
      // This is low, but just needs to stop us from trying to relax a perfect
      // chain
      params.absoluteCostThreshold = 1;
      relaxer.template optimize<SolverType>(params);

      CLOG(DEBUG, "map_projector") << "[Callback Lock Requested] <relaxGraph>";
      change_lock_.lock();
      CLOG(DEBUG, "map_projector") << "[Callback Lock Acquired] <relaxGraph>";

      CLOG(INFO, "map_projector")
          << "[updateRelaxation] Rebuilding message map...";
      for (auto it = frozenGraph.beginVertex(), ite = frozenGraph.endVertex();
           it != ite; ++it) {
        tf_map_[it->id()] = relaxer.at(it->id());
        msg_map_[it->id()].t_vertex_world =
            common::rosutils::toPoseMessage(relaxer.at(it->id()));
        it->setTransform(relaxer.at(it->id()));
      }
    }

    // Update the active branch as it's root will have moved
    CLOG(INFO, "map_projector")
        << "[updateRelaxation] Rebuilding active branch...";

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
    CLOG(DEBUG, "map_projector") << "[Callback Lock Released] <relaxGraph>";

    relaxation_valid_ = true;
    CLOG(INFO, "map_projector") << "[updateRelaxation] Done Relaxation.";

    graph_updates_->publish(msg);
  });
  pool_.start();

  change_lock_.unlock();
  CLOG(DEBUG, "map_projector") << "[Callback Lock Released] relaxGraph";
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
        CLOG(WARNING, "map_projector")
            << "ignoring disconnected vertex: " << it->id();
      }
    }

    // Only populate mesages for manual vertices
    for (auto it = priv->beginVertex(); it != priv->endVertex(); ++it) {
      msg_map_[it->id()].t_vertex_world =
          common::rosutils::toPoseMessage(pf[it->id()]);
    }

    /// Special case where the pose graph has only 1 vertex (0, 0)
    /// In this case getManualSubGraph returns nothing -> likely a bug?
    /// For now, just add this manually, assuming the first is always (0, 0)
    if ((shared_graph->numberOfVertices() == 1) && (msg_map_.size() == 0)) {
      CLOG(WARNING, "map_projector")
          << "Pose graph has only 1 vertex, assumed to be (0, 0) and treated "
             "specially. This could be error-prone.";
      VertexMsg msg;
      msg.id = VertexId(0, 0);
      msg.neighbours.clear();  // no neighbors
      msg.t_vertex_world = common::rosutils::toPoseMessage(pf[VertexId(0, 0)]);
      msg_map_.insert({VertexId(0, 0), msg});
    }
  }
}

void MapProjector::buildProjection() {
  auto shared_graph = getGraph();
  if (!shared_graph->hasMap())
    throw std::runtime_error(
        "[MapProjector] The shared map does not have a valid map info msg!");

  TransformType T_map_root = Eigen::Matrix<double, 6, 1>(
      shared_graph->mapInfo().t_map_vtr.entries.data());
  /// \todo root is in fact always set to <0,0>. ensure this!
  root_ = VertexId(shared_graph->mapInfo().root_vertex);

  if (shared_graph->mapInfo().utm_zone > 0) {  // utm is 1-60, 0 means not set
    auto pstr = pj_str_ + std::to_string(default_map_.utm_zone);
    // delete existing projection (PJ) object
    if (pj_utm_ != nullptr) proj_destroy(pj_utm_);
    // build the new projection
    if (!(pj_utm_ = proj_create(PJ_DEFAULT_CTX, pstr.c_str()))) {
      std::string err = "[MapProjector] Could not build UTM projection";
      CLOG(ERROR, "map_projector") << err;
      throw std::runtime_error{err};
#if false
      std::lock_guard<std::mutex> lock(project_mutex_);
      project_ = [](VertexMsg&) {};
      project_robot_ = [](const VertexId&, const TransformType&) {
        return std::vector<double>({0, 0, 0});
      };
      projection_valid_ = false;
#endif
    } else {
      auto scale = shared_graph->mapInfo().scale;
      std::lock_guard<std::mutex> lock(project_mutex_);
      project_ = [this, scale, T_map_root](VertexMsg& v) {
        Eigen::Matrix4d T_root_vertex(
            common::rosutils::fromPoseMessage(v.t_vertex_world).inverse());
        T_root_vertex.block<3, 1>(0, 3) =
            scale * T_root_vertex.block<3, 1>(0, 3);

        Eigen::Matrix4d T_map_vertex = T_map_root.matrix() * T_root_vertex;

        PJ_COORD src, res;
        src.uv.u = T_map_vertex(0, 3);
        src.uv.v = T_map_vertex(1, 3);
        res = proj_trans(pj_utm_, PJ_INV, src);

        v.t_projected.x = proj_todeg(res.uv.u);
        v.t_projected.y = proj_todeg(res.uv.v);
        v.t_projected.theta =
            std::atan2(T_map_vertex(1, 0), T_map_vertex(0, 0));

        CLOG(DEBUG, "map_projector")
            << "[project_] vertex id: " << v.id << ", x: " << v.t_projected.x
            << ", y: " << v.t_projected.y << ", theta: " << v.t_projected.theta;
      };
      project_robot_ = [this, scale, T_map_root](
                           const VertexId& vid,
                           const TransformType& T_robot_vertex) {
        if (tf_map_.count(vid) == 0) {
          std::stringstream err;
          err << "Cannot find localization vertex id " << vid << " in tf map.";
          CLOG(ERROR, "map_projector") << err.str();
          throw std::runtime_error{err.str()};
        }

        Eigen::Matrix4d T_root_robot = tf_map_.at(vid).inverse().matrix() *
                                       T_robot_vertex.inverse().matrix();
        T_root_robot.block<3, 1>(0, 3) = scale * T_root_robot.block<3, 1>(0, 3);
        Eigen::Matrix4d T_map_robot = T_map_root.matrix() * T_root_robot;

        PJ_COORD src, res;
        src.uv.u = T_map_robot(0, 3);
        src.uv.v = T_map_robot(1, 3);
        res = proj_trans(pj_utm_, PJ_INV, src);

        auto lng = proj_todeg(res.uv.u);
        auto lat = proj_todeg(res.uv.v);
        auto theta = std::atan2(T_map_robot(1, 0), T_map_robot(0, 0));

        CLOG(DEBUG, "map_projector")
            << "[project_robot_] robot live vertex: " << vid
            << ", x: " << std::setprecision(12) << lng << ", y: " << lat
            << ", theta: " << theta;

        return std::vector<double>({lng, lat, theta});
      };
    }
  } else {
    auto scale = shared_graph->mapInfo().scale;
    project_ = [scale, T_map_root](VertexMsg& v) {
      auto T_global_vertex =
          T_map_root.matrix() *
          common::rosutils::fromPoseMessage(v.t_vertex_world).inverse();

      v.t_projected.x = scale * T_global_vertex(0, 3);
      v.t_projected.y = scale * T_global_vertex(1, 3);
      v.t_projected.theta =
          std::atan2(T_global_vertex(1, 0), T_global_vertex(0, 0));
    };
    project_robot_ = [this, scale, T_map_root](
                         const VertexId& vid,
                         const TransformType& T_robot_vertex) {
      if (tf_map_.count(vid) == 0) {
        std::string err =
            "[MapProjector] Cannot find localization vertex id in tf map.";
        CLOG(ERROR, "map_projector") << err;
        throw std::runtime_error{err};
      }
      auto T_vertex_world = tf_map_.at(vid);
      auto T_global_robot = T_map_root.matrix() *
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

    // First add the new vertex to the msg_map_ and set it neighbors
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
  CLOG(INFO, "map_projector") << "Relaxation service called!";
  pool_.wait();

  if (relaxation_valid_) {
    if (projection_valid_ || !request->project) {
      CLOG(INFO, "map_projector") << "[relaxGraph] Using cached graph.";
    } else {
      CLOG(INFO, "map_projector") << "[relaxGraph] Reprojecting graph...";
      updateProjection();
    }
  } else {
    CLOG(INFO, "map_projector") << "[relaxGraph] Relaxing graph...";
    updateRelaxation();
    pool_.wait();

    if (request->project) {
      CLOG(INFO, "map_projector") << "[relaxGraph] Projecting graph...";
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
    CLOG(INFO, "map_projector")
        << "Update robot persistent and target loc after graph reprojection.";
    /// \note we never use the path_seq arg anywhere in the code currently, so
    /// just pass zero.
    /// \note we also assume that target loc is not needed after graph
    /// relaxation, so simply publish an invalid target localization.
    publisher_->publishRobot(cached_persistent_loc_);
  }
}

void MapProjector::pinGraphCallback(GraphPinningSrv::Request::SharedPtr request,
                                    GraphPinningSrv::Response::SharedPtr) {
  CLOG(INFO, "map_projector") << "Graph pinning service called!";
  auto shared_graph = graph_.lock();
  if (!shared_graph)
    throw std::runtime_error(
        "[updateCalib] Attempted to update map info on an expired graph!");

  if (!shared_graph->hasMap())
    throw std::runtime_error(
        "[updateCalib] The pose graph does not have a map info set!");

  MapInfoMsg new_map_info = shared_graph->mapInfo();
  new_map_info.pins = request->pins;
  shared_graph->setMapInfo(new_map_info);
  shared_graph->saveIndex();

  // also update the vector of pins
  cached_response_.pins = shared_graph->mapInfo().pins;

  // No longer having a valid relaxation since we add/remove pins
  relaxation_valid_ = false;

  /// Send a graph invalid message to UI so that the UI will request a full
  /// map update (i.e. call the relax graph service).
  UpdateMsg msg;
  msg.invalidate = true;
  msg.stamp = node_->now();
  msg.vertices.clear();
  msg.seq = nextSeq();
  // update cached_response_ seq id whenever a update is sent
  cached_response_.seq = nextSeq();
  cached_response_.stamp = node_->now();

  graph_updates_->publish(msg);
}

void MapProjector::updateCalibCallback(
    GraphCalibSrv::Request::SharedPtr request,
    GraphCalibSrv::Response::SharedPtr) {
  auto shared_graph = graph_.lock();
  if (!shared_graph)
    throw std::runtime_error(
        "[updateCalib] Attempted to update map info on an expired graph!");

  if (!shared_graph->hasMap())
    throw std::runtime_error(
        "[updateCalib] The pose graph does not have a map info set!");

  MapInfoMsg new_map_info = shared_graph->mapInfo();
  TransformType T(
      Eigen::Matrix<double, 6, 1>(new_map_info.t_map_vtr.entries.data()));

  bool graph_moved = false;

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

    new_map_info.t_map_vtr.entries.clear();
    auto vec = T.vec();

    for (int i = 0; i < 6; ++i)
      new_map_info.t_map_vtr.entries.push_back(vec(i));

    projection_valid_ = false;
    graph_moved = true;
  }

  // Update offset if x or y is not zero
  if (std::abs(request->t_delta.x) > 0 || std::abs(request->t_delta.y) > 0) {
    Eigen::Matrix4d T_new = T.matrix();

    if (new_map_info.pins.size() == 0)
      throw std::runtime_error{
          "[MapProjector] size of graph pins in map info is 0."};
    double lng = new_map_info.pins[0].lng;
    double lat = new_map_info.pins[0].lat;

    // If we have a GPS zone, the update is assumed to be a lat/lon offset
    if (shared_graph->mapInfo().utm_zone > 0) {
      auto pstr = pj_str_ + std::to_string(default_map_.utm_zone);

      PJ* pj_utm;

      if (!(pj_utm = proj_create(PJ_DEFAULT_CTX, pstr.c_str()))) {
        std::string err = "[MapProjector] Could not build UTM projection";
        CLOG(ERROR, "map_projector") << err;
        throw std::runtime_error{err};
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

        // Record the new lat lng of origin to update the pin info
        lng += request->t_delta.x;
        lat += request->t_delta.y;

        // Project back into UTM
        src = proj_trans(pj_utm, PJ_FWD, res);
        r_ab(0) = src.uv.u;
        r_ab(1) = src.uv.v;

        T_new.topRightCorner<3, 1>() = r_ab;
      }

      proj_destroy(pj_utm);
    }
    // Otherwise it is assumed to be a direct offset and no need to update lat
    // lng
    else {
      T_new(0, 3) += request->t_delta.x;
      T_new(1, 3) += request->t_delta.y;
    }

    T = TransformType(T_new);

    new_map_info.t_map_vtr.entries.clear();
    auto vec = T.vec();

    for (int i = 0; i < 6; ++i)
      new_map_info.t_map_vtr.entries.push_back(vec(i));

    new_map_info.pins[0].lat = lat;
    new_map_info.pins[0].lng = lng;

    projection_valid_ = false;
    graph_moved = true;
  }

  // Update map scaling
  if (std::abs(request->scale_delta) > 0) {
    new_map_info.scale = new_map_info.scale * request->scale_delta;
    projection_valid_ = false;
    graph_moved = true;
  }

  if (graph_moved) {
    // Moving graph invalidates all graph pins
    if (new_map_info.pins.size() > 1) {
      new_map_info.pins.erase(new_map_info.pins.begin() + 1,
                              new_map_info.pins.end());
    }
    // also update the vector of pins
    cached_response_.pins = new_map_info.pins;
  }

  if ((!projection_valid_) || graph_moved) {
    shared_graph->setMapInfo(new_map_info);
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