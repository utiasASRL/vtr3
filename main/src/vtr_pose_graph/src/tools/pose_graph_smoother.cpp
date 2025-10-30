#include "vtr_pose_graph/tools/pose_graph_smoother.hpp"
#include "vtr_pose_graph/serializable/rc_graph.hpp"
#include "vtr_logging/logging_init.hpp"

using namespace vtr::pose_graph;

int main(int argc, char const *argv[])
{
  vtr::logging::configureLogging("", true, {"pose_graph.smoothing"});
  auto graph = std::make_shared<RCGraph>("/home/alec/ASRL/vtr3/temp/lelr/autonav_mission/graph");
  
  CLOG(DEBUG, "pose_graph.smoothing") << "Smoothing loaded graph!";
  GraphSmoother<RCVertex, RCEdge> sm{graph};

  sm.runBranchSmoothing();
  return 0;
}
