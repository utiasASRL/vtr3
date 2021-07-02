#pragma once

#include <steam.hpp>

#include <vtr_lidar/icp/lgicp.hpp>
#include <vtr_tactic/modules/base_module.hpp>

namespace vtr {
namespace tactic {

namespace lidar {

/** \brief Preprocess raw pointcloud points and compute normals */
class ICPModule : public BaseModule {
 public:
  /** \brief Static module identifier. */
  static constexpr auto static_name = "lidar.icp";

  /** \brief Collection of config parameters */
  struct Config : public vtr::lidar::ICPParams,
                  steam::VanillaGaussNewtonSolver::Params {
    std::string source = "live";
    bool use_pose_prior = false;
    float min_matched_ratio = 0.4;
    // trajectory smoothing
    bool trajectory_smoothing = false;
    bool use_constant_acc = true;
    double lin_acc_std_dev_x = 10.0;
    double lin_acc_std_dev_y = 10.0;
    double lin_acc_std_dev_z = 10.0;
    double ang_acc_std_dev_x = 1.0;
    double ang_acc_std_dev_y = 1.0;
    double ang_acc_std_dev_z = 1.0;
  };

  ICPModule(const std::string &name = static_name)
      : BaseModule{name}, config_(std::make_shared<Config>()){};

  void configFromROS(const rclcpp::Node::SharedPtr &node,
                     const std::string param_prefix) override;

 private:
  void runImpl(QueryCache &qdata, MapCache &mdata,
               const Graph::ConstPtr &graph) override;

  /** \brief Module configuration. */
  std::shared_ptr<Config> config_;

 private:
  void addPosePrior(
      const EdgeTransform &T_r_m,
      const steam::se3::TransformEvaluator::Ptr &T_r_m_eval,
      const steam::ParallelizedCostTermCollection::Ptr &prior_cost_terms);

  void computeTrajectory(
      QueryCache &qdata, const Graph::ConstPtr &graph,
      const steam::se3::TransformEvaluator::Ptr &T_r_m_eval,
      std::map<unsigned int, steam::StateVariableBase::Ptr> &state_vars,
      const steam::ParallelizedCostTermCollection::Ptr &prior_cost_terms);

  std::shared_ptr<steam::se3::SteamTrajInterface> trajectory_ = nullptr;
  Eigen::Matrix<double, 6, 6> smoothing_factor_information_ =
      Eigen::Matrix<double, 6, 6>::Zero();
};

}  // namespace lidar
}  // namespace tactic
}  // namespace vtr