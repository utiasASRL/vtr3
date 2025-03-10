#include <vtr_logging/logging_init.hpp>  
#include <iostream>
#include <fstream>
#include <memory>
#include <vtr_pose_graph/serializable/rc_graph.hpp>

int main(int argc, char** argv) {
  vtr::logging::configureLogging();  // Initialize logging
  (void)argc;
  (void)argv;
  // Path to your graph folder
  std::string graph_folder = "data/GraphFromAlec/graph"; 
  // Load the pose graph
  using Graph = vtr::pose_graph::RCGraph;
  std::shared_ptr<Graph> graph;
  try {
    graph = std::make_shared<Graph>(graph_folder); 
    LOG(INFO) << "Graph loaded successfully from: " << graph_folder;
  } catch (const std::exception& e) {
    LOG(ERROR) << "Error loading graph: " << e.what();
    return -1;
  }
  // Open a CSV file to save the path
  std::ofstream csv_file("robot_path.csv");
  if (!csv_file.is_open()) {
    LOG(ERROR) << "Error: Could not open output file!";
    return -1;
  }
  LOG(INFO) << "Opened robot_path.csv for writing.";
  // Write the CSV header
  csv_file << "x,y,z\n";
  // Iterate through the vertices in the graph
  for (const auto& vertex : *graph) {
    auto pose_with_cov = vertex.T();
    const lgmath::se3::Transformation& pose = pose_with_cov;
    // Extract translation from the transformation matrix
    auto matrix = pose.matrix();
    if (matrix.isIdentity()) {
        LOG(WARNING) << "Identity matrix encountered, skipping vertex.";
        continue;
    }
    auto translation = matrix.block<3, 1>(0, 3);
    // Write the translation to the CSV file
    csv_file << translation(0) << "," << translation(1) << "," << translation(2) << "\n";
  }
  // Close the CSV file
  csv_file.close();
  LOG(INFO) << "Robot path saved to robot_path.csv";
  return 0;
}






