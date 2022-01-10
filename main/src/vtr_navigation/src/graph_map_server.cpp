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
 * \file graph_map_server.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_navigation/graph_map_server.hpp"

#include "vtr_pose_graph/optimization/pose_graph_optimizer.hpp"
#include "vtr_pose_graph/optimization/pose_graph_relaxation.hpp"

#define ANGLE_NOISE M_PI / 16.0 / 6.0
#define LINEAR_NOISE 0.2 / 6.0

/** \brief The PROJ string defining what projection is required */
static const std::string PJ_STR =
    "+proj=utm +ellps=WGS84 +datum=WGS84 +units=m +no_defs +zone=";

namespace vtr {
namespace navigation {

namespace {
Eigen::Matrix4d fromLngLatTheta(const double lng, const double lat,
                                const double theta) {
  const auto utm_zone = uint32_t((lng + 180.) / 6.) + 1;
  const auto pstr = PJ_STR + std::to_string(utm_zone);
  PJ* pj_utm = proj_create(PJ_DEFAULT_CTX, pstr.c_str());
  if (pj_utm == nullptr) {
    std::string err{"Failed to build UTM projection"};
    CLOG(ERROR, "navigation.graph_map_server") << err;
    throw std::runtime_error{err};
  }
  PJ_COORD src, res;
  src.uv.u = proj_torad(lng);
  src.uv.v = proj_torad(lat);
  res = proj_trans(pj_utm, PJ_FWD, src);
  proj_destroy(pj_utm);

  Eigen::Matrix4d T_map_root = Eigen::Matrix4d::Identity();
  T_map_root.topLeftCorner<2, 2>() << std::cos(theta), -std::sin(theta),
      std::sin(theta), std::cos(theta);
  T_map_root.topRightCorner<2, 1>() << res.uv.u, res.uv.v;
  return T_map_root;
}
}  // namespace

void GraphMapServer::start(const rclcpp::Node::SharedPtr& node,
                           const GraphPtr& graph) {
  graph_ = graph;

  // clang-format off
  /// Parameters: default to UTIAS campus, only for initialization
  const auto lat = node->declare_parameter<double>("graph_projection.origin_lat", 43.782207);
  const auto lng = node->declare_parameter<double>("graph_projection.origin_lng", -79.466092);
  const auto theta = node->declare_parameter<double>("graph_projection.origin_theta", 0.);
  const auto scale = node->declare_parameter<double>("graph_projection.scale", 1.);

  /// Publishers and services
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // graph state
  graph_update_pub_ = node->create_publisher<GraphUpdate>("graph_update", 10);
  graph_state_pub_ = node->create_publisher<GraphState>("graph_state", 10);
  graph_state_srv_ = node->create_service<GraphStateSrv>("graph_state_srv", std::bind(&GraphMapServer::graphStateSrvCallback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);
  // robot state
  robot_state_pub_ = node->create_publisher<RobotState>("robot_state", 10);
  robot_state_srv_ = node->create_service<RobotStateSrv>("robot_state_srv", std::bind(&GraphMapServer::robotStateSrvCallback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);
  // route being followed
  following_route_pub_ = node->create_publisher<FollowingRoute>("following_route", 10);
  following_route_srv_ = node->create_service<FollowingRouteSrv>("following_route_srv", std::bind(&GraphMapServer::followingRouteSrvCallback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);

  // graph manipulation
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  annotate_route_sub_ = node->create_subscription<AnnotateRouteMsg>("annotate_route", rclcpp::QoS(10), std::bind(&GraphMapServer::annotateRouteCallback, this, std::placeholders::_1), sub_opt);
  move_graph_sub_ = node->create_subscription<MoveGraphMsg>("move_graph", rclcpp::QoS(10), std::bind(&GraphMapServer::moveGraphCallback, this, std::placeholders::_1), sub_opt);
  // clang-format on

  // initialize graph mapinfo if working on a new map
  auto map_info = graph->getMapInfo();
  if (!map_info.set) {
    CLOG(INFO, "navigation.graph_map_server")
        << "Initializing pose graph mapinfo";
    map_info.root_vid = 0;
    map_info.lng = lng;
    map_info.lat = lat;
    map_info.theta = theta;
    map_info.scale = scale;
    map_info.set = true;
    graph->setMapInfo(map_info);
  }
  if (graph->numberOfVertices() == 0) return;

  auto graph_lock = graph->guard();  // lock graph then internal lock
  UniqueLock lock(mutex_);
  const auto priv_graph = getPrivilegedGraph();
  optimizeGraph(priv_graph);
  updateVertexProjection();
  updateVertexType();
  computeRoutes(priv_graph);
}

void GraphMapServer::graphStateSrvCallback(
    const std::shared_ptr<GraphStateSrv::Request>,
    std::shared_ptr<GraphStateSrv::Response> response) const {
  CLOG(DEBUG, "navigation.graph_map_server") << "Received graph state request";
  SharedLock lock(mutex_);
  response->graph_state = graph_state_;
}

void GraphMapServer::robotStateSrvCallback(
    const std::shared_ptr<RobotStateSrv::Request>,
    std::shared_ptr<RobotStateSrv::Response> response) const {
  CLOG(DEBUG, "navigation.graph_map_server") << "Received robot state request";
  SharedLock lock(mutex_);
  response->robot_state = robot_state_;
}

void GraphMapServer::followingRouteSrvCallback(
    const std::shared_ptr<FollowingRouteSrv::Request>,
    std::shared_ptr<FollowingRouteSrv::Response> response) const {
  CLOG(DEBUG, "navigation.graph_map_server")
      << "Received following route request";
  SharedLock lock(mutex_);
  response->following_route = following_route_;
}

void GraphMapServer::annotateRouteCallback(
    const AnnotateRouteMsg::ConstSharedPtr msg) {
  CLOG(DEBUG, "navigation.graph_map_server")
      << "Received annotate graph request: ids: " << msg->ids
      << ", type: " << (int)msg->type;
  const auto graph = getGraph();
  for (const auto& id : msg->ids) {
    const auto env_info_msg =
        graph->at(VertexId(id))
            ->retrieve<tactic::EnvInfo>("env_info",
                                        "vtr_tactic_msgs/msg/EnvInfo");
    if (env_info_msg == nullptr) {
      CLOG(ERROR, "navigation.graph_map_server")
          << "Failed to retrieve env_info for vertex " << id;
      throw std::runtime_error{"Failed to retrieve env_info for vertex"};
    }
    auto locked_env_info_msg_ref = env_info_msg->locked();  // lock the msg
    auto& locked_env_info_msg = locked_env_info_msg_ref.get();
    auto env_info = locked_env_info_msg.getData();
    env_info.terrain_type = msg->type;
    locked_env_info_msg.setData(env_info);
  }
  //
  auto graph_lock = graph->guard();  // lock graph then internal lock
  UniqueLock lock(mutex_);
  const auto priv_graph = getPrivilegedGraph();
  updateVertexType();
  computeRoutes(priv_graph);
  //
  graph_state_pub_->publish(graph_state_);
}

void GraphMapServer::moveGraphCallback(const MoveGraphMsg::ConstSharedPtr msg) {
  CLOG(DEBUG, "navigation.graph_map_server")
      << "Received move graph request: <" << msg->lng << ", " << msg->lat
      << ", " << msg->theta << ", " << msg->scale << ">";
  //
  const auto graph = getGraph();
  auto map_info = graph->getMapInfo();
  map_info.lng += msg->lng;
  map_info.lat += msg->lat;
  map_info.theta += msg->theta;
  map_info.scale *= msg->scale;
  getGraph()->setMapInfo(map_info);
  CLOG(DEBUG, "navigation.graph_map_server")
      << "Updated graph map info: <" << map_info.lng << ", " << map_info.lat
      << ", " << map_info.theta << ", " << map_info.scale << ">";

  UniqueLock lock(mutex_);
  updateVertexProjection();
  updateRobotProjection();
  //
  graph_state_pub_->publish(graph_state_);
}

void GraphMapServer::vertexAdded(const VertexPtr& v) {
  if (getGraph()->numberOfVertices() > 1) return;
  /// The first vertex is added
  if ((uint64_t)v->id() != 0) {
    std::string err{"First vertex added is not the root vertex"};
    CLOG(ERROR, "navigation.graph_map_server") << err;
    throw std::runtime_error{err};
  };
  UniqueLock lock(mutex_);
  /// \note \todo currently privileged graph is extracted based on edges
  /// (manual/autonomous), at this moment we do not have any edge in the graph
  /// so privileged graph returns an empty graph, which is wrong. Solution is to
  /// add manual/autonomous info to each vertex.
  /// When above is done, we can simply call:
  ///  const auto priv_graph = getPrivilegedGraph();
  ///  optimizeGraph(priv_graph);
  vid2tf_map_[v->id()] = Transform(true);
  auto& vertex = graph_state_.vertices.emplace_back();
  vertex.id = v->id();
  vid2idx_map_[v->id()] = graph_state_.vertices.size() - 1;
  //
  updateVertexProjection();
}

void GraphMapServer::edgeAdded(const EdgePtr& e) {
  UniqueLock lock(mutex_);
  if (updateIncrementally(e)) return;
  //
  const auto priv_graph = getPrivilegedGraph();
  optimizeGraph(priv_graph);
  updateVertexProjection();
  updateVertexType();
  computeRoutes(priv_graph);
  //
  graph_state_pub_->publish(graph_state_);
}

void GraphMapServer::endRun() {
  auto graph_lock = getGraph()->guard();  // lock graph then internal lock
  UniqueLock lock(mutex_);
  if (getGraph()->numberOfVertices() <= 1) return;

  const auto priv_graph = getPrivilegedGraph();
  optimizeGraph(priv_graph);
  updateVertexProjection();
  updateVertexType();
  computeRoutes(priv_graph);
  //
  graph_state_pub_->publish(graph_state_);
}

void GraphMapServer::robotStateUpdated(const tactic::Localization& persistent,
                                       const tactic::Localization& target) {
  UniqueLock lock(mutex_);
  // cache these in case we update projection
  robot_persistent_loc_ = persistent;
  robot_target_loc_ = target;
  //
  updateRobotProjection();
}

void GraphMapServer::updateRobotProjection() {
  //
  if (project_robot_ == nullptr) return;
  //
  if (robot_persistent_loc_.v.isValid() &&
      vid2tf_map_.count(robot_persistent_loc_.v) != 0) {
    const auto [lng, lat, theta] =
        project_robot_(robot_persistent_loc_.v, robot_persistent_loc_.T);
    robot_state_.valid = true;
    robot_state_.lng = lng;
    robot_state_.lat = lat;
    robot_state_.theta = theta;
    robot_state_.localized = robot_persistent_loc_.localized;
  } else {
    robot_state_.valid = false;
  }
  if (robot_target_loc_.v.isValid() &&
      vid2tf_map_.count(robot_target_loc_.v) != 0) {
    const auto [lng, lat, theta] =
        project_robot_(robot_target_loc_.v, robot_target_loc_.T);
    robot_state_.target_valid = true;
    robot_state_.target_lng = lng;
    robot_state_.target_lat = lat;
    robot_state_.target_theta = theta;
    robot_state_.target_localized = robot_target_loc_.localized;
  } else {
    robot_state_.target_valid = false;
  }
  //
  robot_state_pub_->publish(robot_state_);
}

void GraphMapServer::pathUpdated(const VertexId::Vector& path) {
  UniqueLock lock(mutex_);
  // cache
  following_route_.ids.clear();
  for (const auto& vid : path) following_route_.ids.push_back(vid);
  //
  following_route_pub_->publish(following_route_);
}

auto GraphMapServer::getGraph() const -> GraphPtr {
  if (auto graph_acquired = graph_.lock())
    return graph_acquired;
  else {
    std::string err{"Graph has expired"};
    CLOG(ERROR, "navigation.graph_map_server") << err;
    throw std::runtime_error(err);
  }
  return nullptr;
}

auto GraphMapServer::getPrivilegedGraph() const -> GraphBasePtr {
  // get the current privileged graph
  const auto graph = getGraph();
  using PrivEval =
      typename pose_graph::eval::Mask::Privileged<tactic::GraphBase>::Caching;
  auto priv_eval = std::make_shared<PrivEval>();
  priv_eval->setGraph(graph.get());
  return graph->getSubgraph(priv_eval);
}

void GraphMapServer::optimizeGraph(const tactic::GraphBase::Ptr& priv_graph) {
  const auto map_info = getGraph()->getMapInfo();
  const auto root_vid = VertexId(map_info.root_vid);

  pose_graph::PoseGraphOptimizer<tactic::GraphBase> optimizer(
      priv_graph, root_vid, vid2tf_map_);

  // add pose graph relaxation factors
  // default covariance to use
  Eigen::Matrix<double, 6, 6> cov(Eigen::Matrix<double, 6, 6>::Identity());
  cov.topLeftCorner<3, 3>() *= LINEAR_NOISE * LINEAR_NOISE;
  cov.bottomRightCorner<3, 3>() *= ANGLE_NOISE * ANGLE_NOISE;
  auto relaxation_factor =
      std::make_shared<pose_graph::PoseGraphRelaxation<tactic::GraphBase>>(cov);
  optimizer.addFactor(relaxation_factor);

  // udpates the tf map
  using SolverType = steam::DoglegGaussNewtonSolver;
  optimizer.optimize<SolverType>();

  // update the graph state vertices and idx map
  auto& vertices = graph_state_.vertices;
  vertices.clear();
  vid2idx_map_.clear();
  for (auto it = priv_graph->beginVertex(), ite = priv_graph->endVertex();
       it != ite; ++it) {
    auto& vertex = vertices.emplace_back();
    vertex.id = it->id();
    for (auto&& jt : priv_graph->neighbors(it->id()))
      vertex.neighbors.push_back(jt->id());
    //
    vid2idx_map_[it->id()] = vertices.size() - 1;
  }
}

void GraphMapServer::updateVertexProjection() {
  const auto map_info = getGraph()->getMapInfo();

  // delete existing projection (PJ) object
  if (pj_utm_ != nullptr) proj_destroy(pj_utm_);
  const auto utm_zone = uint32_t((map_info.lng + 180.) / 6.) + 1;
  const auto pstr = PJ_STR + std::to_string(utm_zone);
  pj_utm_ = proj_create(PJ_DEFAULT_CTX, pstr.c_str());
  if (!pj_utm_) {
    std::string err{"Failed to build UTM projection"};
    CLOG(ERROR, "navigation.graph_map_server") << err;
    throw std::runtime_error{err};
  }
  //
  auto T_map_root = fromLngLatTheta(map_info.lng, map_info.lat, map_info.theta);
  const auto scale = map_info.scale;
  project_vertex_ = [this, T_map_root, scale](const VertexId& vid) {
    Eigen::Matrix4d T_root_vertex = vid2tf_map_.at(vid).inverse().matrix();
    T_root_vertex.block<3, 1>(0, 3) = scale * T_root_vertex.block<3, 1>(0, 3);

    Eigen::Matrix4d T_map_vertex = T_map_root * T_root_vertex;

    PJ_COORD src, res;
    src.uv.u = T_map_vertex(0, 3);
    src.uv.v = T_map_vertex(1, 3);
    res = proj_trans(pj_utm_, PJ_INV, src);

    auto lng = proj_todeg(res.uv.u);
    auto lat = proj_todeg(res.uv.v);
    auto theta = std::atan2(T_map_vertex(1, 0), T_map_vertex(0, 0));
    CLOG(DEBUG, "navigation.graph_map_server")
        << "Project - vertex id: " << vid << ", lng: " << std::setprecision(12)
        << lng << ", lat: " << lat << ", theta: " << theta;
    return std::make_tuple(lng, lat, theta);
  };

  project_robot_ = [this, T_map_root, scale](const VertexId& vid,
                                             const Transform& T_robot_vertex) {
    if (vid2tf_map_.count(vid) == 0) {
      std::stringstream err;
      err << "Cannot find localization vertex id " << vid << " in tf map.";
      CLOG(ERROR, "navigation.graph_map_server") << err.str();
      throw std::runtime_error{err.str()};
    }
    Eigen::Matrix4d T_root_vertex = vid2tf_map_.at(vid).inverse().matrix();
    Eigen::Matrix4d T_root_robot =
        T_root_vertex * T_robot_vertex.inverse().matrix();
    T_root_robot.block<3, 1>(0, 3) = scale * T_root_robot.block<3, 1>(0, 3);

    Eigen::Matrix4d T_map_robot = T_map_root * T_root_robot;

    PJ_COORD src, res;
    src.uv.u = T_map_robot(0, 3);
    src.uv.v = T_map_robot(1, 3);
    res = proj_trans(pj_utm_, PJ_INV, src);

    auto lng = proj_todeg(res.uv.u);
    auto lat = proj_todeg(res.uv.v);
    auto theta = std::atan2(T_map_robot(1, 0), T_map_robot(0, 0));
    CLOG(DEBUG, "navigation.graph_map_server")
        << "Project - robot live vertex: " << vid << std::setprecision(12)
        << ", lng: " << lng << ", lat: " << lat << ", theta: " << theta;
    return std::make_tuple(lng, lat, theta);
  };

  /// updateProjection
  // project the vertices
  auto& vertices = graph_state_.vertices;
  for (auto&& vertex : vertices) {
    const auto [lng, lat, theta] = project_vertex_(VertexId(vertex.id));
    vertex.lng = lng;
    vertex.lat = lat;
    vertex.theta = theta;
  }
  // project the robot
  if (robot_persistent_loc_.v.isValid()) {
    const auto [lng, lat, theta] =
        project_robot_(robot_persistent_loc_.v, robot_persistent_loc_.T);
    robot_state_.lng = lng;
    robot_state_.lat = lat;
    robot_state_.theta = theta;
  }
  if (robot_target_loc_.v.isValid()) {
    const auto [lng, lat, theta] =
        project_robot_(robot_target_loc_.v, robot_target_loc_.T);
    robot_state_.target_lng = lng;
    robot_state_.target_lat = lat;
    robot_state_.target_theta = theta;
  }
}

void GraphMapServer::updateVertexType() {
  const auto graph = getGraph();
  auto& vertices = graph_state_.vertices;
  for (auto&& vertex : vertices) {
    const auto env_info_msg =
        graph->at(VertexId(vertex.id))
            ->retrieve<tactic::EnvInfo>("env_info",
                                        "vtr_tactic_msgs/msg/EnvInfo");
    vertex.type = env_info_msg->sharedLocked().get().getData().terrain_type;
  }
}

void GraphMapServer::computeRoutes(const tactic::GraphBase::Ptr& priv_graph) {
  /// \note for now we do not use junctions in the GUI, which is the return
  /// value from this function, we also do note distinguis between path and
  /// cycles, so just call them routes - a single name
  typename tactic::GraphBase::ComponentList routes;
  priv_graph->pathDecomposition(routes, routes);
  //
  auto& fixed_routes = graph_state_.fixed_routes;
  fixed_routes.clear();
  for (auto&& route : routes) {
    int curr_route_type = -1;
    for (auto&& id : route.elements()) {
      const auto type = graph_state_.vertices[vid2idx_map_.at(id)].type;
      // new route
      if (curr_route_type == -1) {
        auto& fixed_route = fixed_routes.emplace_back();
        fixed_route.type = type;
        fixed_route.ids.push_back(id);
        curr_route_type = type;
      }
      // new route with a different type
      else if (curr_route_type != type) {
        const auto prev_id = fixed_routes.back().ids.back();
        //
        auto& fixed_route = fixed_routes.emplace_back();
        fixed_route.type = type;
        fixed_route.ids.push_back(prev_id);
        fixed_route.ids.push_back(id);
        curr_route_type = type;
      }
      // same type
      else {
        fixed_routes.back().ids.push_back(id);
      }
    }
  }
  //
  graph_state_.active_routes.clear();
}

bool GraphMapServer::updateIncrementally(const EdgePtr& e) {
  // Autonomouse edges do not need to be considered
  if (e->isAutonomous()) return true;
  // Spatial edge always triggers a complete update
  if (e->isSpatial()) return false;
  // Simply ignore this edge if it is not connected to the main graph (trunk)
  if (vid2tf_map_.count(e->from()) == 0) {
    std::stringstream ss;
    ss << "Cannot find vertex " << e->from()
       << " in vid2tf_map_, haven't localized to a trunk vertex yet so not "
          "updating the map.";
    CLOG(DEBUG, "navigation.graph_map_server") << ss.str();
    return true;
  }

  //
  const auto from = e->from();
  const auto to = e->to();
  const auto T_to_from = e->T();

  // consistency check
  if (vid2tf_map_.count(to) != 0) {
    std::stringstream ss;
    ss << "Cannot connect to an existing vertex " << to
       << " via a temporal edge";
    CLOG(ERROR, "navigation.graph_map_server") << ss.str();
    throw std::runtime_error{ss.str()};
  }
  if (!(((uint64_t(to) - uint64_t(from)) == 1))) {
    std::stringstream ss;
    ss << "Temporal edge from " << from << " to " << to << " isn't continuous.";
    CLOG(ERROR, "navigation.graph_map_server") << ss.str();
    throw std::runtime_error{ss.str()};
  }

  // vid2tfmap update
  vid2tf_map_[to] = T_to_from * vid2tf_map_.at(from);

  // graph_state_.vertices.<id, neighbors>
  auto& vertices = graph_state_.vertices;
  // update from neighbors
  vertices[vid2idx_map_.at(from)].neighbors.push_back(to);
  // add to into the vertices
  auto& vertex = vertices.emplace_back();
  vertex.id = to;
  vertex.neighbors.push_back(from);
  vid2idx_map_[to] = vertices.size() - 1;

  // projection
  const auto [lng, lat, theta] = project_vertex_(to);
  vertex.lng = lng;
  vertex.lat = lat;
  vertex.theta = theta;

  // vertex type
  if (vertices[vid2idx_map_.at(from)].type == -1) {
    const auto env_info_msg = getGraph()->at(from)->retrieve<tactic::EnvInfo>(
        "env_info", "vtr_tactic_msgs/msg/EnvInfo");
    if (env_info_msg == nullptr) {
      std::stringstream ss;
      ss << "Cannot find env_info for vertex " << from
         << ", which is assumed added at this moment.";
      CLOG(ERROR, "navigation.graph_map_server") << ss.str();
      throw std::runtime_error{ss.str()};
    }
    vertices[vid2idx_map_.at(from)].type =
        env_info_msg->sharedLocked().get().getData().terrain_type;
  }
  const auto env_info_msg = getGraph()->at(to)->retrieve<tactic::EnvInfo>(
      "env_info", "vtr_tactic_msgs/msg/EnvInfo");
  if (env_info_msg == nullptr) {
    std::stringstream ss;
    ss << "Cannot find env_info for vertex " << to
       << ", which is assumed added at this moment.";
    CLOG(ERROR, "navigation.graph_map_server") << ss.str();
    throw std::runtime_error{ss.str()};
  }
  vertex.type = env_info_msg->sharedLocked().get().getData().terrain_type;

  // add to active route
  auto& active_routes = graph_state_.active_routes;
  if (active_routes.empty()) {
    active_routes.emplace_back();
    auto& active_route = active_routes.back();
    active_route.type = vertices[vid2idx_map_.at(from)].type;
    active_route.ids.emplace_back(from);
  }
  active_routes.back().ids.emplace_back(to);
  if (active_routes.back().type != vertex.type) {
    active_routes.emplace_back();
    auto& active_route = active_routes.back();
    active_route.type = vertex.type;
    active_route.ids.emplace_back(to);
  }

  // compute and publish the update message
  GraphUpdate graph_update;
  graph_update.vertex_from = vertices[vid2idx_map_.at(from)];
  graph_update.vertex_to = vertices[vid2idx_map_.at(to)];
  graph_update_pub_->publish(graph_update);

  CLOG(DEBUG, "navigation.graph_map_server") << "Incremental update succeeded";
  return true;
}

}  // namespace navigation
}  // namespace vtr