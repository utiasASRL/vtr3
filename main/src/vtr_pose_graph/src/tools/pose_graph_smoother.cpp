#include "vtr_pose_graph/tools/pose_graph_smoother.hpp"
#include "vtr_pose_graph/serializable/rc_graph.hpp"
#include "vtr_logging/logging_init.hpp"

using namespace vtr::pose_graph;

int main(int argc, char const *argv[])
{
  vtr::logging::configureLogging("", true, {"pose_graph.smoothing"});


  if (argc != 2) {
    CLOG(ERROR, "pose_graph.smoothing") << "Wrong number of arguments! Call as ros2 run vtr_pose_graph vtr_pose_graph_smoother /file/path/graph";
    return 1;
  }

  auto graph = std::make_shared<RCGraph>(argv[1]);
  
  CLOG(DEBUG, "pose_graph.smoothing") << "Smoothing loaded graph!";
  GraphSmoother<RCVertex, RCEdge> sm{graph};

  sm.runBranchSmoothing();
  return 0;
}
