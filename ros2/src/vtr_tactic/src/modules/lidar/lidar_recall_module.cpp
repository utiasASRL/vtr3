#include <vtr_tactic/modules/lidar/lidar_recall_module.hpp>

namespace {
std::shared_ptr<PointMap> copyPointcloudMap(
    const float voxel_size, const PointCloudMapMsg::SharedPtr &map_msg) {
  auto N = map_msg->points.size();

  std::vector<PointXYZ> points;
  points.reserve(N);
  std::vector<PointXYZ> normals;
  normals.reserve(N);
  std::vector<float> scores;
  scores.reserve(N);

  for (int i = 0; i < N; i++) {
    const auto &point = map_msg->points[i];
    const auto &normal = map_msg->normals[i];
    const auto &score = map_msg->scores[i];
    // Add all points to the vector container
    points.push_back(PointXYZ(point.x, point.y, point.z));
    normals.push_back(PointXYZ(normal.x, normal.y, normal.z));
    scores.push_back(score);
  }
  auto map = std::make_shared<PointMap>(voxel_size, points, normals, scores);
  return map;
}

void retrievePointCloudMap(const PointCloudMapMsg::SharedPtr &map_msg,
                           std::vector<PointXYZ> &points,
                           std::vector<PointXYZ> &normals,
                           std::vector<float> &scores) {
  auto N = map_msg->points.size();
  points.reserve(N);
  normals.reserve(N);
  scores.reserve(N);

  for (int i = 0; i < N; i++) {
    const auto &point = map_msg->points[i];
    const auto &normal = map_msg->normals[i];
    const auto &score = map_msg->scores[i];
    // Add all points to the vector container
    points.push_back(PointXYZ(point.x, point.y, point.z));
    normals.push_back(PointXYZ(normal.x, normal.y, normal.z));
    scores.push_back(score);
  }
}

}  // namespace

namespace vtr {
namespace tactic {

void LidarRecallModule::runImpl(QueryCache &qdata, MapCache &mdata,
                                const Graph::ConstPtr &graph) {
  if (*qdata.first_frame) {
    LOG(INFO) << "First keyframe, simply return.";
    return;
  }

  // input
  auto &live_id = *qdata.live_id;

  LOG(INFO) << "Loading vertex id: " << live_id.minorId();

  //
  auto vertex = graph->at(live_id);
  const auto &map_msg =
      vertex->retrieveKeyframeData<PointCloudMapMsg>("pcl_map");
  std::vector<PointXYZ> points;
  std::vector<PointXYZ> normals;
  std::vector<float> scores;
  retrievePointCloudMap(map_msg, points, normals, scores);
  auto map = std::make_shared<PointMap>(config_->map_voxel_size);
  map->update(points, normals, scores);
  qdata.current_map_odo = map;
}

void LidarRecallModule::visualizeImpl(QueryCache &qdata, MapCache &mdata,
                                      const Graph::ConstPtr &, std::mutex &) {
  if (!config_->visualize) return;

  if (!map_pub_)
    map_pub_ =
        qdata.node->create_publisher<PointCloudMsg>("curr_map_points", 20);

  auto pc2_msg = std::make_shared<PointCloudMsg>();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (qdata.current_map_odo) {
    for (auto pt : (*qdata.current_map_odo).cloud.pts)
      cloud.points.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    pcl::toROSMsg(cloud, *pc2_msg);
  }
  pc2_msg->header.frame_id = "odometry keyframe";
  pc2_msg->header.stamp = *qdata.rcl_stamp;

  map_pub_->publish(*pc2_msg);
}

}  // namespace tactic
}  // namespace vtr