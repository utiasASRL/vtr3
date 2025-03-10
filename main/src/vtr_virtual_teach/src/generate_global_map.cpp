#include <array>
#include <cmath>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <matplotlibcpp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/path/pose_cache.hpp>
#include <vtr_pose_graph/serializable/rc_graph.hpp>
#include <vtr_pose_graph/id/id.hpp>
#include <vtr_pose_graph/serializable/rc_edge.hpp>
#include <vtr_pose_graph/serializable/bubble_interface.hpp>
#include <vtr_pose_graph/serializable/rc_vertex.hpp>
#include <vtr_pose_graph/index/edge_base.hpp>
#include <vtr_pose_graph/index/vertex_base.hpp>
#include <vtr_pose_graph/index/graph_base.hpp>
#include "vtr_lidar/pipeline.hpp"
#include "vtr_lidar/data_types/pointmap.hpp"
#include "vtr_lidar/data_types/pointmap_pointer.hpp"
#include "vtr_lidar/modules/pointmap/intra_exp_merging_module_v2.hpp"
#include "vtr_lidar/data_types/point.hpp"
#include "vtr_storage/stream/message.hpp"
#include "vtr_tactic/modules/factory.hpp"
#include "vtr_tactic/modules/memory/graph_mem_manager_module.hpp"

// Use necessary namespaces
namespace plt = matplotlibcpp;
using namespace vtr::logging;
using namespace vtr::pose_graph;
using namespace vtr::storage;
using namespace vtr::tactic;
using namespace vtr::lidar;

pcl::PointCloud<pcl::PointNormal>::Ptr loadPointCloud(const std::string& filename) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
  if (pcl::io::loadPCDFile<pcl::PointNormal>(filename, *cloud) == -1) {
    throw std::runtime_error("Failed to load PCD file: " + filename);
  }
  return cloud;
}

std::vector<std::pair<Eigen::Matrix4d, Timestamp>> readTransformMatricesWithTimestamps(const std::string& filename) {
  std::vector<std::pair<Eigen::Matrix4d, Timestamp>> data;
  std::ifstream file(filename);
  if (!file.is_open()) 
    throw std::runtime_error("Could not open file: " + filename);

  // Skip header line
  std::string line;
  std::getline(file, line);

  // Read and parse each line
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string value;

    // Parse the timestamp (first column contains it)
    std::getline(ss, value, ',');
    double fractional_timestamp = std::stod(value);  // Retain fractional seconds
    Timestamp timestamp = static_cast<int64_t>(fractional_timestamp * 1e9);  // Convert to nanoseconds
    
    // Parse 4x4 transformation matrix
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        std::getline(ss, value, ',');
        transform(i, j) = std::stod(value);
      }
    }
    data.emplace_back(transform, timestamp);
  }
  return data;
}

std::shared_ptr<RCGraph> createPoseGraph(
    const std::vector<std::pair<Eigen::Matrix4d, Timestamp>>& data, 
    const std::string& graph_path) {

  // Initialize pose graph
  auto graph = std::make_shared<RCGraph>(graph_path, false);
  graph->addRun();
  vtr::pose_graph::RCGraph::MapInfoMsg map_info_msg;
  
  map_info_msg.lat = 43.7822;
  map_info_msg.lng = -79.4661;
  map_info_msg.theta = 1.3;
  map_info_msg.scale = 1.0;
  map_info_msg.set= true;

  graph->setMapInfo(map_info_msg);


  if (data.empty()) {
    throw std::runtime_error("No data provided to create the pose graph.");
  }
  
  const auto& [initial_transform, initial_timestamp] = data.front();  
  graph->addVertex(initial_timestamp); //dont need to assign a transform, this is the origin

  for (size_t i = 1; i < data.size(); ++i) {   // Add vertices and edges for each transformation and timestamp
    const auto& [transform, timestamp] = data[i];
    RCVertex::Ptr new_vertex = graph->addVertex(timestamp);    
    EdgeTransform edge_transform(transform);
    edge_transform.setZeroCovariance();
    graph->addEdge(new_vertex->id() - 1, new_vertex->id(), EdgeType::Temporal, true, edge_transform);
    }

  // Save the graph
  graph->save();

  return graph;
}

Eigen::Matrix4d computeAbsolutePoseByTimestamp(
  const std::vector<std::pair<Eigen::Matrix4d, Timestamp>>& transforms_with_timestamps,
  Timestamp vertex_time) {
  if (transforms_with_timestamps.empty()) {
    throw std::runtime_error("No transformation data available.");
  }

  // Initialize global_pose with the first transformation
  Eigen::Matrix4d global_pose = Eigen::Matrix4d::Identity();

  std::vector<double> x_coords;
  std::vector<double> y_coords;
  std::vector<double> z_coords;

  for (size_t i = 1; i < transforms_with_timestamps.size(); i++) {
    const auto& [transform, timestamp] = transforms_with_timestamps[i];
    if (timestamp <= vertex_time) {     // Accumulate transforms up to and including the vertex timestamp
      global_pose = transform * global_pose; 
      x_coords.push_back(global_pose(0, 3)); 
      y_coords.push_back(global_pose(1, 3));
      z_coords.push_back(global_pose(2, 3));
      if (timestamp == vertex_time) {
        break;
      }
    } else {
      break;
    }
  }
  return global_pose;
}

int main(int argc, char **argv) {  
  try {
    // Redirect std::cout to a log file
    std::ofstream log_file("output_log.txt");
    std::streambuf* cout_buf = std::cout.rdbuf();
    std::cout.rdbuf(log_file.rdbuf());

    // Initialize ROS2 node
    rclcpp::init(argc, argv);  
    auto node = std::make_shared<rclcpp::Node>("lidar_pipeline_node");

    auto config_ = vtr::lidar::LidarPipeline::Config::fromROS(node, "lidar_pipeline");

    // Configure logging
    std::string log_filename = "example_log.txt";
    bool enable_debug = true;
    std::vector<std::string> enabled_loggers = {"test"}; //removed "pose_graph"
    vtr::logging::configureLogging(log_filename, enable_debug, enabled_loggers);

    // Load point cloud data
    auto cloud = loadPointCloud("/home/desiree/ASRL/vtr3/data/grassy/point_cloud.pcd");
    std::cout << "Point cloud loaded successfully." << std::endl;

    // Read transformation matrices from CSV
    std::string odometry_csv_path = "/home/desiree/ASRL/vtr3/data/grassy/nerf_gazebo_relative_transforms.csv";
    auto matrices_with_timestamps = readTransformMatricesWithTimestamps(odometry_csv_path);

    // This transform brings the first pose (absolute) to identity.
    Eigen::Matrix4d origin_transform = matrices_with_timestamps.front().first;
    Eigen::Matrix4d rebase_transform = origin_transform.inverse();

    // Rebase the point cloud: transform it using the rebase_transform.
    pcl::PointCloud<pcl::PointNormal>::Ptr rebased_cloud = std::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    pcl::transformPointCloudWithNormals(*cloud, *rebased_cloud, rebase_transform);
    cloud = rebased_cloud; 

    // Create and populate pose graph
    std::string graph_path = "/home/desiree/ASRL/vtr3/data/grassy/graph";
    auto graph = createPoseGraph(matrices_with_timestamps, graph_path);

    // Reload the saved graph
    auto loaded_graph = RCGraph::MakeShared(graph_path, true);

    vtr::pose_graph::VertexId last_submap_vertex_id = vtr::pose_graph::VertexId::Invalid();
    Eigen::Matrix4d last_submap_pose = Eigen::Matrix4d::Identity();  // Initialize with identity

    // Parameters for the cylindrical filter
    float cylinder_radius = 40.0;  //changed to 50 and 15 for grassy
    float cylinder_height = 20.0;   

    // Iterate through all vertices in the graph 
    for (auto it = loaded_graph->begin(0ul); it != loaded_graph->end(); ++it) {
      auto vertex = *it;  // Dereference iterator to access vertex
      auto vertex_id = vertex.v()->id();
      auto vertex_time = vertex.v()->vertexTime();
      std::cout << "Vertex ID: " << vertex_id
                << ", Timestamp (nanoseconds): " << vertex_time << std::endl;
      
      using EnvInfoLM = vtr::storage::LockableMessage<EnvInfo>;
      const auto env_info_data = std::make_shared<EnvInfo>();
      const auto env_info_msg = std::make_shared<EnvInfoLM>(env_info_data, vertex_time);
      vertex.v()->insert<EnvInfo>("env_info", "vtr_tactic_msgs/msg/EnvInfo", env_info_msg);

      using WaypointNameLM = vtr::storage::LockableMessage<WaypointName>;
      const auto waypoint_name_data = std::make_shared<WaypointName>();
      const auto waypoint_name_msg = std::make_shared<WaypointNameLM>(waypoint_name_data, vertex_time);
      vertex.v()->insert<WaypointName>("waypoint_name", "vtr_tactic_msgs/msg/WaypointNames", waypoint_name_msg);

      // Compute the current absolute pose for this vertex
      Eigen::Matrix4d current_pose = computeAbsolutePoseByTimestamp(matrices_with_timestamps, vertex_time);
      
      bool createNewSubmap = false;

      // No submap has been created yet - create one.
      if (!last_submap_vertex_id.isValid()) {
          createNewSubmap = true;
      } else {
          // Compute transformation from last submap vertex to current robot position
          const auto T_sv_r = last_submap_pose * current_pose.inverse(); // Equivalent to T_sv_m_odo_ * T_r_m_odo_->inverse()
          
          // Extract translation component (3D distance)
          Eigen::Vector3d translation = T_sv_r.block<3,1>(0,3);  
          double dtran = translation.norm();  // Euclidean distance traveled
          
          // Extract rotation component (Euler angles in radians convert to degrees)
          Eigen::Vector3d rotation_vec = T_sv_r.block<3,3>(0,0).eulerAngles(0, 1, 2);  
          double drot = rotation_vec.norm() * (180.0 / M_PI);  

          std::cout << "Distance from last submap: " << dtran << " m" << std::endl;
          std::cout << "Rotation from last submap: " << drot << " degrees" << std::endl;

          // Apply the same thresholds as VTR3
          if (dtran > 1.5 || drot > 30) { //when replaced with config_ references, every vertex gets a submap (tested on lidar odom w nerf cloud)... not sure why cuz its the same odometry as the existing teach graph where not every vertex has a submap
              createNewSubmap = true;
          }
      }
      
      if (createNewSubmap) {
        std::cout << "Creating new submap at vertex " << vertex_id << std::endl;

        // Compute the absolute pose using vertex timestamp
        Eigen::Matrix4d absolute_pose = computeAbsolutePoseByTimestamp(
            matrices_with_timestamps, vertex_time);
        std::cout << "Transformation Matrix:\n" << absolute_pose << std::endl;

        if (cloud->empty()) {
            throw std::runtime_error("Input point cloud is empty!");
        }
        // Transform the point cloud to align with the vertex's pose and convert to PointWithInfo
        pcl::PointCloud<PointWithInfo>::Ptr converted_cloud(new pcl::PointCloud<PointWithInfo>());
        pcl::copyPointCloud(*cloud, *converted_cloud);
        pcl::PointCloud<PointWithInfo>::Ptr transformed_cloud(new pcl::PointCloud<PointWithInfo>());
        pcl::transformPointCloudWithNormals(*converted_cloud, *transformed_cloud, absolute_pose); 
        
        // Apply cylindrical filter extending upwards by 3/4 the height and downwards by 1/4 the height
        pcl::PointCloud<PointWithInfo>::Ptr cropped_cloud(new pcl::PointCloud<PointWithInfo>());
        float lower_bound = -cylinder_height / 4.0;
        float upper_bound = 3 * cylinder_height / 4.0;
        for (auto& point : *transformed_cloud) {
          const float x = point.x;
          const float y = point.y;
          const float z = point.z;
          const float distance_xy = std::sqrt(x * x + y * y);
          if ((distance_xy <= cylinder_radius) && (z >= lower_bound) && (z <= upper_bound)) {
            point.normal_score = 1; 
            cropped_cloud->push_back(point);
          }
        }
        
        //debugging
        std::cout << "Original cloud size: " << cloud->size() << std::endl;
        std::cout << "Transformed cloud size: " << transformed_cloud->size() << std::endl;
        std::cout << "Cropped cloud size: " << cropped_cloud->size() << std::endl;

        // Create a submap and update it with the transformed point cloud
        float voxel_size = 0.7; //chaged from 0.9 to 0.7 for grassy // Adjust based on nerf point cloud - was 0.9, 0.5 0.05 - now trying 1.5 because ICP script previous did 1.0 but not using anymore
        auto submap_odo = std::make_shared<PointMap<PointWithInfo>>(voxel_size); 
        submap_odo->update(*cropped_cloud); 

        // Save the submap as a lockable message
        using PointMapLM = vtr::storage::LockableMessage<PointMap<PointWithInfo>>;
        auto submap_msg = std::make_shared<PointMapLM>(submap_odo, vertex_time);

        // Insert the submap into the vertex
        vertex.v()->insert<PointMap<PointWithInfo>>(
            "pointmap", "vtr_lidar_msgs/msg/PointMap", submap_msg);
        std::cout << "Point cloud associated with vertex " << vertex_id << "." << std::endl;
    
        std::cout << "Submap pointer saved for vertex " << vertex_id << "." << std::endl; // Update the last submap vertex ID
        // Update the last submap tracking variables
        last_submap_vertex_id = vertex_id;
        last_submap_pose = current_pose;  // Update last submap pose

      } 
      std::cout << "Using previous submap for vertex " << vertex_id << std::endl;
      
      // Compute the relative transform from the last submap to the current pose.
      Eigen::Matrix4d relative_transform = current_pose * last_submap_pose.inverse();
      
      auto submap_ptr = std::make_shared<PointMapPointer>();
      submap_ptr->this_vid = vertex_id; 
      submap_ptr->map_vid = last_submap_vertex_id;
      submap_ptr->T_v_this_map = EdgeTransform(relative_transform);
      submap_ptr->T_v_this_map.setZeroCovariance();
      
      using PointMapPointerLM = vtr::storage::LockableMessage<PointMapPointer>;
      auto submap_ptr_msg = std::make_shared<PointMapPointerLM>(submap_ptr, vertex_time);
      vertex.v()->insert<PointMapPointer>(
          "pointmap_ptr", "vtr_lidar_msgs/msg/PointMapPointer", submap_ptr_msg);
      
      std::cout << "Submap pointer saved for vertex " << vertex_id << "." << std::endl;
      
    }

    // loop closure code 


    rclcpp::shutdown();
    
    // Restore std::cout to its original state before exiting
    std::cout.rdbuf(cout_buf);
    return 0;

  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\n";
    return -1;
  }
}