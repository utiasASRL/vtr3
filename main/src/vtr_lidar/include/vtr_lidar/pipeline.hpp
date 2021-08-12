#pragma once

#include <vtr_lidar/cache.hpp>
#include <vtr_lidar/modules/modules.hpp>
#include <vtr_logging/logging.hpp>
#include <vtr_tactic/modules/base_module.hpp>
#include <vtr_tactic/modules/module_factory.hpp>
#include <vtr_tactic/pipelines/base_pipeline.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vtr_messages/msg/movability.hpp>
#include <vtr_messages/msg/point_map.hpp>

namespace vtr {
namespace tactic {

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointXYZMsg = vtr_messages::msg::PointXYZ;
using MovabilityMsg = vtr_messages::msg::Movability;
using PointMapMsg = vtr_messages::msg::PointMap;

class LidarPipeline : public BasePipeline {
 public:
  using Ptr = std::shared_ptr<LidarPipeline>;

  /** \brief Static pipeline identifier. */
  static constexpr auto static_name = "lidar";

  /** \brief Collection of config parameters */
  struct Config {
    std::vector<std::string> preprocessing;
    std::vector<std::string> odometry;
    std::vector<std::string> localization;

    float map_voxel_size = 0.2;
  };

  LidarPipeline(const std::string &name = static_name) : BasePipeline{name} {
    addModules();
  }

  virtual ~LidarPipeline() {}

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string &param_prefix) override;

  void initialize(const Graph::Ptr &graph) override;

  void preprocess(QueryCache::Ptr &qdata, const Graph::Ptr &graph) override;
  void visualizePreprocess(QueryCache::Ptr &qdata,
                           const Graph::Ptr &graph) override;

  void runOdometry(QueryCache::Ptr &qdata, const Graph::Ptr &graph) override;
  void visualizeOdometry(QueryCache::Ptr &qdata,
                         const Graph::Ptr &graph) override;

  void runLocalization(QueryCache::Ptr &qdata,
                       const Graph::Ptr &graph) override;
  void visualizeLocalization(QueryCache::Ptr &qdata,
                             const Graph::Ptr &graph) override;

  void processKeyframe(QueryCache::Ptr &qdata, const Graph::Ptr &graph,
                       VertexId live_id) override;

  void wait() override;

  void reset() override;

 private:
  void addModules();

  void setOdometryPrior(LidarQueryCache::Ptr &qdata, const Graph::Ptr &graph);

  void savePointcloudMap(LidarQueryCache::Ptr qdata, const Graph::Ptr graph,
                         VertexId live_id);

 private:
  /** \brief Pipeline configuration */
  std::shared_ptr<Config> config_ = std::make_shared<Config>();

  std::vector<BaseModule::Ptr> preprocessing_;
  std::vector<BaseModule::Ptr> odometry_;
  std::vector<BaseModule::Ptr> localization_;

  /**
   * \brief A candidate cache in case for odometry failure, where the candidate
   * cache is used to create a keyframe.
   */
  LidarQueryCache::Ptr candidate_qdata_ = nullptr;

  /** \brief Current map being built */
  std::shared_ptr<vtr::lidar::IncrementalPointMap> new_map_;

  /** \brief Current map and its vertex for odometry */
  std::shared_ptr<vtr::lidar::IncrementalPointMap> odo_map_;
  std::shared_ptr<VertexId> odo_map_vid_;
  std::shared_ptr<lgmath::se3::TransformationWithCovariance> odo_map_T_v_m_;

  std::mutex map_saving_mutex_;
  std::future<void> map_saving_thread_future_;

  /**
   * \brief a trjacetory to estimate transform at a future time
   * \note no need to use a lock since this variable is only used in odometry to
   * get a better T_r_m prior.
   */
  std::shared_ptr<steam::se3::SteamTrajInterface> trajectory_;
  /** \brief the time at which the trajectory was estimated */
  common::timing::time_point trajectory_time_point_;
};

}  // namespace tactic
}  // namespace vtr
