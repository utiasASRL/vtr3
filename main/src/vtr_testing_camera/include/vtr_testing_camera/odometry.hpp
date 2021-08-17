#pragma once

#include <vtr_testing_camera/offline.hpp>

class OdometryNavigator : public OfflineNavigator {
 public:
  OdometryNavigator(const rclcpp::Node::SharedPtr node, std::string output_dir)
      : OfflineNavigator(node, output_dir) {
    // normally the state machine would add a run when a goal is started. We
    // spoof that here.
    tactic_->addRun();

    // Create a branch pipeline.
    tactic_->setPipeline(tactic::PipelineMode::Branching);
  }

 private:
};