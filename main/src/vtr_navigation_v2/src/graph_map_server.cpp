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
#include "vtr_navigation_v2/graph_map_server.hpp"

#include "vtr_pose_graph/optimization/pose_graph_optimizer.hpp"
#include "vtr_pose_graph/optimization/pose_graph_relaxation.hpp"

#define ANGLE_NOISE M_PI / 16.0 / 6.0
#define LINEAR_NOISE 0.2 / 6.0

namespace vtr {
namespace navigation {

void GraphMapServer::start(const rclcpp::Node::SharedPtr& node,
                           const GraphPtr& graph) {
  LockGuard lock(mutex_);
  graph_ = graph;

  // clang-format off
  /// Parameters: default to UTIAS campus, only for initialization
  const auto lat = node->declare_parameter<double>("graph_projection.origin_lat", 43.782207);
  const auto lng = node->declare_parameter<double>("graph_projection.origin_lng", -79.466092);
  const auto theta = node->declare_parameter<double>("graph_projection.origin_theta", 0.);
  const auto scale = node->declare_parameter<double>("graph_projection.scale", 1.);

  /// Publishers and services
  callback_group_ = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
#if false
  // robot state
  robot_state_pub_ = node->create_publisher<RobotStateMsg>("robot_state", 10);
  robot_state_srv_ = node->create_service<RobotStateSrv>("robot_state_srv", std::bind(&GraphMapServer::robotStateSrvCallback, this, std::placeholders::_1, std::placeholders::_2));
  // graph state
  graph_update_pub_ = node->create_publisher<GraphUpdateMsg>("graph_update", 10);
  graph_state_pub_ = node->create_publisher<GraphStateMsg>("graph_state", 10);
#endif
  graph_state_srv_ = node->create_service<GraphStateSrv>("graph_state_srv", std::bind(&GraphMapServer::graphStateSrvCallback, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default, callback_group_);

  // graph manipulation
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = callback_group_;
  move_graph_sub_ = node->create_subscription<MoveGraphMsg>("move_graph", rclcpp::QoS(10), std::bind(&GraphMapServer::moveGraphCallback, this, std::placeholders::_1), sub_opt);
  // clang-format on

  // initialize graph mapinfo if working on a new map
  auto map_info = getGraph()->getMapInfo();
  if (!map_info.set) {
    CLOG(INFO, "navigator.graph_map_server")
        << "Initializing pose graph mapinfo";
    map_info.root_vid = 0;
    map_info.utm_zone = uint32_t((lng + 180.) / 6.) + 1;
    // build default projection
    const auto pstr = pj_str_ + std::to_string(map_info.utm_zone);
    PJ* pj_utm = proj_create(PJ_DEFAULT_CTX, pstr.c_str());
    if (pj_utm == nullptr) {
      std::string err{"Failed to build UTM projection"};
      CLOG(ERROR, "navigator.graph_map_server") << err;
      throw std::runtime_error{err};
    }
    PJ_COORD src, res;
    src.uv.u = proj_torad(lng);
    src.uv.v = proj_torad(lat);
    res = proj_trans(pj_utm, PJ_FWD, src);
    proj_destroy(pj_utm);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<2, 2>() << std::cos(theta), std::sin(theta),
        -std::sin(theta), std::cos(theta);
    T.topRightCorner<2, 1>() << res.uv.u, res.uv.v;
    lgmath::se3::Transformation T_map_root(T);
    const auto vec = T_map_root.vec();
    for (int i = 0; i < vec.size(); ++i)
      map_info.tf_map_root.entries.push_back(vec(i));

    map_info.scale = scale;
    map_info.set = true;
    //
    getGraph()->setMapInfo(map_info);
  }

  CLOG(INFO, "navigator.graph_map_server")
      << "Pose graph map info " << std::fixed
      << "- root_vid: " << map_info.root_vid
      << ", utm_zone: " << map_info.utm_zone
      << ", tf_map_root: " << map_info.tf_map_root.entries
      << ", scale: " << map_info.scale;

  computeGraphState();
}

void GraphMapServer::graphStateSrvCallback(
    const std::shared_ptr<GraphStateSrv::Request>,
    std::shared_ptr<GraphStateSrv::Response> response) const {
  CLOG(WARNING, "navigator.graph_map_server") << "Received graph state request";
  response->graph_state = graph_state_;
}

void GraphMapServer::moveGraphCallback(const MoveGraphMsg::ConstSharedPtr msg) {
  CLOG(WARNING, "navigator.graph_map_server")
      << "Received move graph request: <" << msg->lng << ", " << msg->lat
      << ", " << msg->theta << ", " << msg->scale << ">";
}

void GraphMapServer::computeGraphState() {
  // get the current privileged graph
  const auto graph = getGraph();

  using PrivEval =
      typename pose_graph::eval::Mask::Privileged<tactic::GraphBase>::Caching;
  auto priv_eval = std::make_shared<PrivEval>();
  priv_eval->setGraph(graph.get());
  const auto priv_subgraph = graph->getSubgraph(priv_eval);

  /// updateRelaxation
  // get the current map info
  const auto map_info = graph->getMapInfo();
  const auto root_vid = VertexId(map_info.root_vid);

  pose_graph::PoseGraphOptimizer<tactic::GraphBase> optimizer(
      priv_subgraph, root_vid, vid2tf_map_);

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

  /// build projection
  const auto utm_zone = map_info.utm_zone;
  const auto scale = map_info.scale;
  Transform T_map_root =
      Eigen::Matrix<double, 6, 1>(map_info.tf_map_root.entries.data());
  auto pstr = pj_str_ + std::to_string(utm_zone);
  // delete existing projection (PJ) object
  if (pj_utm_ != nullptr) proj_destroy(pj_utm_);
  pj_utm_ = proj_create(PJ_DEFAULT_CTX, pstr.c_str());
  if (!pj_utm_) {
    std::string err{"Failed to build UTM projection"};
    CLOG(ERROR, "navigator.graph_map_server") << err;
    throw std::runtime_error{err};
  }
  //
  project_vertex_ = [this, scale, T_map_root](const VertexId& vid) {
    Eigen::Matrix4d T_root_vertex = vid2tf_map_.at(vid).inverse().matrix();
    T_root_vertex.block<3, 1>(0, 3) = scale * T_root_vertex.block<3, 1>(0, 3);

    Eigen::Matrix4d T_map_vertex = T_map_root.matrix() * T_root_vertex;

    PJ_COORD src, res;
    src.uv.u = T_map_vertex(0, 3);
    src.uv.v = T_map_vertex(1, 3);
    res = proj_trans(pj_utm_, PJ_INV, src);

    CLOG(DEBUG, "navigator.graph_map_server")
        << "Project - vertex id: " << vid << ", x: " << proj_todeg(res.uv.u)
        << ", y: " << proj_todeg(res.uv.v)
        << ", theta: " << std::atan2(T_map_vertex(1, 0), T_map_vertex(0, 0));

    return std::make_tuple<double, double, double>(
        proj_todeg(res.uv.u), proj_todeg(res.uv.v),
        std::atan2(T_map_vertex(1, 0), T_map_vertex(0, 0)));
  };
#if false
  project_robot_ = [this, scale, T_map_root](const VertexId& vid,
                                             const Transform& T_robot_vertex) {
    if (tf_map_.count(vid) == 0) {
      std::stringstream err;
      err << "Cannot find localization vertex id " << vid << " in tf map.";
      CLOG(ERROR, "navigator.graph_map_server") << err.str();
      throw std::runtime_error{err.str()};
    }

    Eigen::Matrix4d T_root_robot =
        tf_map_.at(vid).inverse().matrix() * T_robot_vertex.inverse().matrix();
    T_root_robot.block<3, 1>(0, 3) = scale * T_root_robot.block<3, 1>(0, 3);
    Eigen::Matrix4d T_map_robot = T_map_root.matrix() * T_root_robot;

    PJ_COORD src, res;
    src.uv.u = T_map_robot(0, 3);
    src.uv.v = T_map_robot(1, 3);
    res = proj_trans(pj_utm_, PJ_INV, src);

    auto lng = proj_todeg(res.uv.u);
    auto lat = proj_todeg(res.uv.v);
    auto theta = std::atan2(T_map_robot(1, 0), T_map_robot(0, 0));

    CLOG(DEBUG, "navigator.graph_map_server")
        << "[project_robot_] robot live vertex: " << vid
        << ", x: " << std::setprecision(12) << lng << ", y: " << lat
        << ", theta: " << theta;

    return std::vector<double>({lng, lat, theta});
  };

  /// updateProjection
  // project the vertices
  for (auto&& it : msg_map_) project_(it.second);
  // project the robot
  msg.lng_lat_theta = project_robot_(persistent_loc.v, persistent_loc.T);
  if (target_loc.localized && target_loc.successes > 5) {
    msg.target_lng_lat_theta = project_robot_(target_loc.v, target_loc.T);
  }
#endif

  auto& vertices = graph_state_.vertices;
  vertices.clear();
  for (auto it = priv_subgraph->beginVertex(), ite = priv_subgraph->endVertex();
       it != ite; ++it) {
    auto& vertex = vertices.emplace_back();
    vertex.id = it->id();
    for (auto&& jt : priv_subgraph->neighbors(it->id()))
      vertex.neighbors.push_back(jt->id());
    const auto [lng, lat, theta] = project_vertex_(it->id());
    vertex.lng = lng;
    vertex.lat = lat;
    vertex.theta = theta;
    vertex.type = 0;  /// \todo get type from vertex
  }

  /// path decomposition
  auto& fixed_routes = graph_state_.fixed_routes;
  fixed_routes.clear();
  typename tactic::GraphBase::ComponentList routes;
  /// \note for now we do not use junctions in the GUI, which is the return
  /// value from this function, we also do note distinguis between path and
  /// cycles, so just call them routes - a single name
  priv_subgraph->pathDecomposition(routes, routes);
  for (auto&& route : routes) {
    auto& fixed_route = fixed_routes.emplace_back();
    fixed_route.type = 0;  /// \todo get type from vertex
    for (auto&& id : route.elements()) fixed_route.ids.push_back(id);
  }
  graph_state_.active_routes.clear();
  graph_state_.current_route = GraphRoute();
}

}  // namespace navigation
}  // namespace vtr