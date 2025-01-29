// Copyright 2023, Autonomous Space Robotics Lab (ASRL)
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
 * \file torch_module.cpp
 * \author Alec Krawciw, Autonomous Space Robotics Lab (ASRL)
 */
 
 /*
#include <vtr_lidar/modules/scan_matching/scan_matching.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cmath> 
#include <list>
#include <algorithm>
#include <cstdlib>
#include <Python.h>
#include <open3d/Open3D.h>
#include <open3d/pipelines/registration/Registration.h>
#include <filesystem>
#include "matplotlibcpp.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
namespace vtr {
namespace lidar {

using namespace tactic;


namespace fs = std::filesystem;
//Terminal command: g++ -o main main.cpp -I/home/hendrik/anaconda3/envs/new_logg/include/python3.9 -L/home/hendrik/anaconda3/envs/new_logg/lib -Wl,-rpath=/home/hendrik/anaconda3/envs/new_logg/lib -lpython3.9 -std=c++17


std::vector<std::vector<double>> load_embeddings(const std::string& filePath) {
    std::ifstream file(filePath);
    std::vector<std::vector<double>> array;

    std::string line;
    while (getline(file, line)) {
        std::istringstream iss(line);
        std::vector<double> row;
        double num;

        while (iss >> num) {
            row.push_back(num);
        }

        std::cout << "Loaded row: ";
        for (const double& val : row) {
            std::cout << val << " ";
        }
        std::cout << std::endl;

        array.push_back(row);
    }

    file.close();
    return array;
}



std::pair<std::vector<DataPoint>, std::vector<DataPoint>> readDataPoints(const std::string& filePath) {
    std::ifstream file(filePath);
    std::vector<DataPoint> dataPoints;
    std::vector<DataPoint> positions;



    std::string line;
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        std::vector<std::string> values;

        while (std::getline(ss, value, ',')) {
            values.push_back(value);
        }

        DataPoint dp;
        dp.timestamp = std::stod(values[0]);
        dp.x = std::stod(values[1]);
        dp.y = std::stod(values[2]);
        dp.z = std::stod(values[3]);
        dp.qx = std::stod(values[4]);
        dp.qy = std::stod(values[5]);
        dp.qz = std::stod(values[6]);
        dp.qw = std::stod(values[7]);

        dataPoints.push_back(dp);

        DataPoint ps;
        ps.timestamp = std::stod(values[0]);
        ps.x = std::stod(values[1]);
        ps.y = std::stod(values[2]);
        ps.z = std::stod(values[3]);
        positions.push_back(dp);
    }
    
    file.close();

    return {dataPoints, positions};
}

double dot_product(const std::vector<double>& v1, const std::vector<double>& v2) {
    double sum = 0.0;
    if (v1.size() != v2.size()) {
        throw std::invalid_argument("Vectors must be of the same length");
    }
    for (size_t i = 0; i < v1.size(); ++i) {
        sum += v1[i] * v2[i];
    }
    return sum;
}

double magnitude(const std::vector<double>& v) {
    double sum = 0.0;
    for (double elem : v) {
        sum += elem * elem;
    }
    return std::sqrt(sum);
}

double cosine_dist(const std::vector<double>& v1, const std::vector<double>& v2) {
    double dot = dot_product(v1, v2);
    double mag1 = magnitude(v1);
    double mag2 = magnitude(v2);
    if (mag1 == 0 || mag2 == 0) {
        throw std::invalid_argument("Magnitude of vector must not be zero");
    }
    double cos_theta = dot / (mag1 * mag2);
    return 1.0 - cos_theta;  
}

double euclideanDist(const DataPoint& a, const DataPoint& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) +
                     (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}

std::vector<std::string> get_pointcloud_files(const std::string& base_directory) {
    // Read all files in the directory and add them to the vector

    std::vector<std::string> file_paths;

    // Read all files in the directory and add their full paths to the vector
    for (const auto& entry : fs::directory_iterator(base_directory)) {
        if (fs::is_regular_file(entry.status())) {
            file_paths.push_back(entry.path().string());
        }
    }

    // Sort the file paths alphabetically
    std::sort(file_paths.begin(), file_paths.end());


    return file_paths;
}

std::vector<std::vector<double>> read_matrix_from_txt(const std::string& file_path, int rows, int cols) {
    std::vector<std::vector<double>> matrix(rows, std::vector<double>(cols));  // Initialize 2D vector
    std::ifstream file(file_path);


    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (!(file >> matrix[i][j])) {  
                std::cerr << "Error reading element (" << i << ", " << j << ")" << std::endl;
            }
        }
    }

    file.close();  
    return matrix; 
}


int scan_matching(tactic::QueryCache &qdata0){
  auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);



  std::string file_path = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/run_0/run_0_latent_vectors/run_0_latent.txt";
  std::string file_path_current_scan = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/current_scan/current_scan_latent.txt";
  int rows = 216;
  int cols = 1024;
  
  std::vector<std::vector<double>> embeddings_teach = read_matrix_from_txt(file_path, rows, cols);

  std::vector<std::vector<double>> current_embedding= read_matrix_from_txt(file_path_current_scan, 1, 1024);

  std::string teach_poses_file = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/run_0/pose_graph_run_0.csv";
  auto [teach_poses, teach_pos] = readDataPoints(teach_poses_file);
  std::string path_0 = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/run_0/run_0_pcd/";
  std::vector<std::string> teach_inidivdual_pointcloud_files = get_pointcloud_files(path_0);

  std::string submap_repeat_poses_file = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/submaps_0/submaps_0.csv";
  auto [submap_poses, submap_pos] = readDataPoints(submap_repeat_poses_file);    
  std::string path_submap = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/submaps_0/submaps_0_pcd";
  std::vector<std::string> submap_pointcloud_files = get_pointcloud_files(path_submap);
  

  std::vector<double> all_dist_latent; 


  //Get the current position of the robot and find the closest match on the teach path:
  
  int current_idx = embeddings_teach.size() - 1; //change to make not hard coded
  auto current_pose = teach_pos[current_idx];



  for (int i = 0; i < embeddings_teach.size(); i++) {

      if (i==current_idx){
          continue;
      }
  all_dist_latent.push_back(cosine_dist(current_embedding[0], embeddings_teach[i]));  
  }


  int smallest_dist_idx = std::distance(std::begin(all_dist_latent), std::min_element(std::begin(all_dist_latent), std::end(all_dist_latent)));

  return smallest_dist_idx;

}













void reportCriticalError(){
  std::cerr << "CRITICAL ERROR ERROR ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;

}






}  // namespace nn
}  // namespace vtr




// //old 


// std::vector<std::vector<double>> load_embeddings(const std::string& filePath) {
//     std::ifstream file(filePath);
//     std::vector<std::vector<double>> array;

//     if (!file.is_open()) {
//         std::cerr << "Error opening file: " << filePath << std::endl;
//         return array;
//     }

//     std::string line;
//     while (getline(file, line)) {
//         std::istringstream iss(line);
//         std::vector<double> row;
//         double num;

//         while (iss >> num) {
//             row.push_back(num);
//         }

//         // Print the current row
//         std::cout << "Loaded row: ";
//         for (const double& val : row) {
//             std::cout << val << " ";
//         }
//         std::cout << std::endl;

//         array.push_back(row);
//     }

//     file.close();
//     return array;
// }



// std::pair<std::vector<DataPoint>, std::vector<DataPoint>> readDataPoints(const std::string& filePath) {
//     std::ifstream file(filePath);
//     std::vector<DataPoint> dataPoints;
//     std::vector<DataPoint> positions;



//     std::string line;
//     std::getline(file, line);

//     while (std::getline(file, line)) {
//         std::stringstream ss(line);
//         std::string value;
//         std::vector<std::string> values;

//         // Split the line by commas
//         while (std::getline(ss, value, ',')) {
//             values.push_back(value);
//         }

//         DataPoint dp;
//         dp.timestamp = std::stod(values[0]);
//         dp.x = std::stod(values[1]);
//         dp.y = std::stod(values[2]);
//         dp.z = std::stod(values[3]);
//         dp.qx = std::stod(values[4]);
//         dp.qy = std::stod(values[5]);
//         dp.qz = std::stod(values[6]);
//         dp.qw = std::stod(values[7]);

//         dataPoints.push_back(dp);

//         DataPoint ps;
//         ps.timestamp = std::stod(values[0]);
//         ps.x = std::stod(values[1]);
//         ps.y = std::stod(values[2]);
//         ps.z = std::stod(values[3]);
//         positions.push_back(dp);
//     }
    
//     file.close();

//     return {dataPoints, positions};
// }

// double dot_product(const std::vector<double>& v1, const std::vector<double>& v2) {
//     double sum = 0.0;
//     if (v1.size() != v2.size()) {
//         throw std::invalid_argument("Vectors must be of the same length");
//     }
//     for (size_t i = 0; i < v1.size(); ++i) {
//         sum += v1[i] * v2[i];
//     }
//     return sum;
// }

// double magnitude(const std::vector<double>& v) {
//     double sum = 0.0;
//     for (double elem : v) {
//         sum += elem * elem;
//     }
//     return std::sqrt(sum);
// }

// double cosine_dist(const std::vector<double>& v1, const std::vector<double>& v2) {
//     double dot = dot_product(v1, v2);
//     double mag1 = magnitude(v1);
//     double mag2 = magnitude(v2);
//     if (mag1 == 0 || mag2 == 0) {
//         throw std::invalid_argument("Magnitude of vector must not be zero");
//     }
//     double cos_theta = dot / (mag1 * mag2);
//     return 1.0 - cos_theta;  
// }

// double euclideanDist(const DataPoint& a, const DataPoint& b) {
//     return std::sqrt((a.x - b.x) * (a.x - b.x) +
//                      (a.y - b.y) * (a.y - b.y) +
//                      (a.z - b.z) * (a.z - b.z));
// }

// void init_python_interpreter() {
//     Py_Initialize();
//     PyRun_SimpleString("import sys");
//     PyRun_SimpleString("sys.path.append('/home/hendrik/Desktop/Wild-Places/scripts/eval')");
// }

// void finalize_python_interpreter() {
//     Py_Finalize();
// }

// void call_python_function() {
//     PyObject *pName, *pModule, *pFunc;
//     PyObject *pArgs, *pValue;

//     // Load the module
//     pName = PyUnicode_DecodeFSDefault("intra-sequence");  // Without '.py' extension
//     pModule = PyImport_Import(pName);
//     Py_DECREF(pName);

//     if (pModule != nullptr) {
//         pFunc = PyObject_GetAttrString(pModule, "get_pointcloud_embedding");
//         if (pFunc && PyCallable_Check(pFunc)) {
//             pArgs = PyTuple_New(3);
//             PyTuple_SetItem(pArgs, 0, PyUnicode_FromString("/home/hendrik/Datasets/LongLoop_converted/submaps/submaps_0/submaps_0_pcd"));
//             PyTuple_SetItem(pArgs, 1, PyUnicode_FromString("/home/hendrik/Desktop/Wild-Places/latent_vectors"));
//             PyTuple_SetItem(pArgs, 2, PyUnicode_FromString("submaps_0"));

//             pValue = PyObject_CallObject(pFunc, pArgs);
//             Py_DECREF(pArgs);
//             if (pValue != nullptr) {
//                 std::cout << "Function call successful" << std::endl;
//                 Py_DECREF(pValue);
//             } else {
//                 PyErr_Print();
//                 std::cerr << "Call failed" << std::endl;
//             }
//         } else {
//             if (PyErr_Occurred())
//                 PyErr_Print();
//             std::cerr << "Cannot find function 'get_pointcloud_embedding'" << std::endl;
//         }
//         Py_XDECREF(pFunc);
//         Py_DECREF(pModule);
//     } else {
//         PyErr_Print();
//         std::cerr << "Failed to load the module" << std::endl;
//     }
// }
// std::vector<std::string> get_pointcloud_files(const std::string& base_directory) {
//     // Read all files in the directory and add them to the vector

//     std::vector<std::string> file_paths;

//     // Read all files in the directory and add their full paths to the vector
//     for (const auto& entry : fs::directory_iterator(base_directory)) {
//         if (fs::is_regular_file(entry.status())) {
//             file_paths.push_back(entry.path().string());
//         }
//     }

//     // Sort the file paths alphabetically
//     std::sort(file_paths.begin(), file_paths.end());


//     return file_paths;
// }

// Eigen::Matrix4d reconstruct_tf_from_quaternion(const std::vector<DataPoint>& coords, int idx) {
//     Eigen::Matrix4d Tf = Eigen::Matrix4d::Identity();  // Initialize with identity matrix

//     // Extract position and quaternion from coords

//     Eigen::Vector3d position(coords[idx].x, coords[idx].y, coords[idx].z);
//     Eigen::Quaterniond quaternion(coords[idx].qw, coords[idx].qx, coords[idx].qy, coords[idx].qz);


//     // Convert quaternion to rotation matrix and take the transpose
//     Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix().transpose();

//     // Set the rotation part of the transformation matrix
//     Tf.block<3,3>(0,0) = rotation_matrix;

//     // Set the translation part of the transformation matrix
//     Tf.block<3,1>(0,3) = position;

//     return Tf;

// }
// std::vector<std::vector<double>> read_matrix_from_txt(const std::string& file_path, int rows, int cols) {
//     std::vector<std::vector<double>> matrix(rows, std::vector<double>(cols));  // Initialize 2D vector
//     std::ifstream file(file_path);

//     if (!file.is_open()) {
//         std::cerr << "Unable to open file: " << file_path << std::endl;
//         return matrix;  // Return empty matrix on failure
//     }

//     for (int i = 0; i < rows; ++i) {
//         for (int j = 0; j < cols; ++j) {
//             if (!(file >> matrix[i][j])) {  // Read each element
//                 std::cerr << "Error reading element (" << i << ", " << j << ")" << std::endl;
//             }
//         }
//     }

//     file.close();  // Close file after reading
//     return matrix;  // Return populated 2D vector
// }


// int findIndexOfEntry(const std::vector<std::string>& vec, const std::string& target) {
//     for (size_t i = 0; i < vec.size(); ++i) {
//         if (vec[i] == target) {
//             return i;  // Return the index if the target is found
//         }
//     }
//     return -1;  // Return -1 if the target is not found
// }

// void initialisation(){

    


// }


// void execute_scan_matching(tactic::QueryCache &qdata0) {
//     // auto &qdata = dynamic_cast<LidarQueryCache &>(qdata0);
//     static std::vector<std::vector<double>> all_embeddings_teach;
//     static std::vector<double> all_timestamps;
//     static std::vector<Timestamp> all_timestamps_double;
//     // using namespace open3d;
//     // using namespace open3d::pipelines::registration;
//     namespace plt = matplotlibcpp;
//     // Load source and target point clouds
//     // auto source = std::make_shared<geometry::PointCloud>();
//     // auto target = std::make_shared<geometry::PointCloud>();
//     static auto source_o3d = open3d::geometry::PointCloud();
//     static auto target_o3d = open3d::geometry::PointCloud();


//     // int test = *qdata.stamp;

//     // // std::cerr << test << std::endl;
//     // Timestamp timestamp = qdata.pointcloud_msg->header.stamp.sec * 1e9 + qdata.pointcloud_msg->header.stamp.nanosec;
//     // std::cout << "timestamp" << timestamp << std::endl;

//     // all_timestamps_double.push_back(timestamp);
//     // std::string timestamp_lidar = std::to_string(timestamp);

//     // if (timestamp_lidar.length() > 10) {
//     //     timestamp_lidar.insert(timestamp_lidar.length() - 9, ".");
//     // }

//     // double timestamp_lidar1 = std::stod(timestamp_lidar);

//     // std::cout << "timestamp stod" << std::stod(timestamp_lidar) << std::endl;
//     // CLOG(DEBUG, "navigation") << "Lidar timestamp " << timestamp_lidar;




//     // //Idea: Instead of saving the observed timestamps and poses to a file, create a class that keep track of it. Then you append it to each parameter: Teach.timestamp, Teach.timestamp.embedding, teach.timestamp.submap, Teach.timestamp.pose



//     // std::string file_path = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/run_0/run_0_latent_vectors/run_0_latent.txt";
//     // int rows = 216;
//     // int cols = 1024;
    
//     // std::vector<std::vector<double>> embeddings_teach = read_matrix_from_txt(file_path, rows, cols);

//     // std::string teach_poses_file = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/run_0/pose_graph_run_0.csv";
//     // auto [teach_poses, teach_pos] = readDataPoints(teach_poses_file);
//     // std::string path_0 = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/run_0/run_0_pcd/";
//     // std::vector<std::string> teach_inidivdual_pointcloud_files = get_pointcloud_files(path_0);



//     // std::string target_file = path_0 + timestamp_lidar + ".pcd";

//     // int pcd_idx = findIndexOfEntry(teach_inidivdual_pointcloud_files, target_file);


//     // //If timestamp not found in the pcd recordings: 
//     // if (pcd_idx == -1) {
//     //   return;
//     // }
//     // all_timestamps.push_back((timestamp_lidar1));

//     // std::cout << "pcd_idx" << pcd_idx << std::endl;

//     // std::vector<double> current_embedding = embeddings_teach[pcd_idx];

//     // std::vector<double> numbers; 

//     // for (int t = 0; t < 215; t++){

//     //     numbers.push_back(cosine_dist(current_embedding, embeddings_teach[t]));

//     // }
//     // std::vector<std::pair<int, int>> values_with_indices;

//     // for (int i = 0; i < numbers.size(); ++i) {
//     //     values_with_indices.push_back({numbers[i], i});  // Store the value and its original index
//     //     std::cout << numbers[i] << std::endl;

//     // }
    
//     // // Sort the vector of pairs based on the value (first element of the pair)
//     // std::sort(values_with_indices.begin(), values_with_indices.end());
    
//     // // Print the first five smallest values along with their original indices
//     // std::cout << "The first five smallest values and their original indices are:\n";
//     // for (int i = 0; i < 10; ++i) {
//     //     std::cout << "Value: " << values_with_indices[i].first
//     //               << ", Original Index: " << values_with_indices[i].second << std::endl;
//     // }
//     // all_embeddings_teach.push_back(current_embedding);

//     std::string file_path = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/run_0/run_0_latent_vectors/run_0_latent.txt";
//     int rows = 216;
//     int cols = 1024;
    
//     std::vector<std::vector<double>> embeddings_teach = read_matrix_from_txt(file_path, rows, cols);

//     std::string teach_poses_file = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/run_0/pose_graph_run_0.csv";
//     auto [teach_poses, teach_pos] = readDataPoints(teach_poses_file);
//     std::string path_0 = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/run_0/run_0_pcd/";
//     std::vector<std::string> teach_inidivdual_pointcloud_files = get_pointcloud_files(path_0);

//     std::string submap_repeat_poses_file = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/submaps_0/submaps_0.csv";
//     auto [submap_poses, submap_pos] = readDataPoints(submap_repeat_poses_file);    
//     std::string path_submap = "/home/hendrik/ASRL/vtr3/Datasets_converted/Jordy_Loop/submaps_0/submaps_0_pcd";
//     std::vector<std::string> submap_pointcloud_files = get_pointcloud_files(path_submap);
    


//     std::vector<double> all_dist_latent; 
//     std::vector<double> pose_error_x; 
//     std::vector<double> pose_error_y; 
//     std::vector<double> pose_error_z;

//     std::vector<double> dist_travelled; 


//     // auto source_o3d = std::make_shared<open3d::geometry::PointCloud>();
//     // auto target_o3d = std::make_shared<open3d::geometry::PointCloud>();


//     int count = 0;  
//     int l = 0;


//     for (int k = 0; k < 216; k++) {

//         if (k == 0){
//             dist_travelled.push_back(std::sqrt(std::pow(teach_pos[k].x, 2) + std::pow(teach_pos[k].y, 2) + std::pow(teach_pos[k].z, 2)));
//         }
//         else {
//             dist_travelled.push_back(std::sqrt(std::pow(teach_pos[k].x - teach_pos[k-1].x, 2) + std::pow(teach_pos[k].y - teach_pos[k-1].y, 2) + std::pow(teach_pos[k].z - teach_pos[k-1].z, 2)) + dist_travelled[k-1]);

//         // std::cout <<  std::fixed << std::setprecision(10) <<"Pose threshold" <<  dist_travelled<< std::endl;

//         }


//         if (k < 5) {
//             continue;
//         }

//         std::vector<double> all_dist_latent; 

//         for (int l = 0; l < k; l++) {

//                 if (k == l) {
//                     break;
//                 }

//                 all_dist_latent.push_back(cosine_dist(embeddings_teach[k], embeddings_teach[l]));  
//             }

//             double smallest_dist_idx = std::distance(std::begin(all_dist_latent), std::min_element(std::begin(all_dist_latent), std::end(all_dist_latent)));

//             auto smallest_latent = std::min_element(std::begin(all_dist_latent), std::end(all_dist_latent));


//             //Remove smallest element
//             all_dist_latent.erase (all_dist_latent.begin() + smallest_dist_idx);

//             double second_smallest_dist_idx = std::distance(std::begin(all_dist_latent), std::min_element(std::begin(all_dist_latent), std::end(all_dist_latent)));
//             auto second_smallest_dist_latent = std::min_element(std::begin(all_dist_latent), std::end(all_dist_latent));

//             double dist_idx = smallest_dist_idx;



//             std::vector<double> pose_error_total;
//             std::vector<Eigen::Matrix4d> all_icp_tfs;
//             std::vector<Eigen::Matrix4d> all_estimates;


//             //2 because we run ICP on the smallest and second smallest distance
//             for (int i = 0; i < 2; i++){

//                 if (i == 1) {
//                     dist_idx = second_smallest_dist_idx;

//                 }



//                 if (std::abs(dist_travelled[k] - dist_travelled[dist_idx]) < 10.0){
//                     continue;
//                 }

//                 std::cout << "Dist travelled difference: " << std::abs(dist_travelled[k] - dist_travelled[dist_idx]) << std::endl;



//                 std::vector<double> distances;

//                 // Becuase we are finding the corresponding submap distance wise, we need to limit the look up table of all submaps to simulate real-time. 

//                 for (int m = 0; m < std::round(k/4); m++){

//                     distances.push_back(euclideanDist(teach_pos[dist_idx], submap_pos[m]));

//                 }

//                 int submap_idx =  std::distance(std::begin(distances), std::min_element(std::begin(distances), std::end(distances)));

//                 std::cout << "current idx: " << k << std::fixed << std::setprecision(10) <<"Timestamp" <<  teach_pos[k].timestamp << std::endl;

//                 std::cout << " idx: "<< dist_idx << "Its cosine value: " << *smallest_latent << std::fixed << std::setprecision(10) <<"Timestamp" <<  teach_pos[dist_idx].timestamp <<"Time difference: " <<  std::abs(teach_pos[k].timestamp - teach_pos[dist_idx].timestamp)<<std::endl;

//                 // std::cout << "Second idx: "<< second_smallest_dist_idx << "Its cosine value: " <<*second_smallest_dist_latent<< std::fixed << std::setprecision(10) <<"Timestamp: " <<  teach_pos[second_smallest_dist_idx].timestamp<< "Time difference: " <<  std::abs(teach_pos[k].timestamp - teach_pos[second_smallest_dist_idx].timestamp)<<std::endl;



//                 // auto target_o3d = std::make_shared<open3d::geometry::PointCloud>();
//                 // open3d::io::ReadPointCloud(submap_pointcloud_files[submap_idx], *target_o3d);




//                 // auto source_o3d = std::make_shared<open3d::geometry::PointCloud>();
//                 // open3d::io::ReadPointCloud(teach_inidivdual_pointcloud_files[k], *source_o3d);
//                 // source_o3d->EstimateNormals();


//                 // Eigen::Matrix4d flip_axis;
//                 // flip_axis << -1,  0,  0, 0,
//                 //             0,  1,  0, 0,
//                 //             0,  0, -1, 0,
//                 //             0,  0,  0, 1;

//                 // Eigen::Matrix4d target_to_target_tf;
//                 // target_to_target_tf <<  9.99992980e-01, -3.62860204e-03,  9.34243994e-04, -2.54583940e-02,
//                 //                         3.62984168e-03,  9.99992529e-01, -1.32863085e-03, -6.89743590e-04,
//                 //                         -9.29415942e-04,  1.33201268e-03,  9.99998681e-01, -8.39290591e-01,
//                 //                         0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00;

//                 // target_o3d->Transform(flip_axis);
//                 // target_o3d->Transform(target_to_target_tf.inverse());
//                 // target_o3d->EstimateNormals();

//                 // const double total_rotation = 2 * M_PI; 
//                 // int num_intervals = 8;

//                 // for (int j = 0; j < num_intervals; j++){ //Change to increase the orientations
//                 //     double theta = j * total_rotation / num_intervals;
//                 //     Eigen::Matrix4d orientation;
//                 //     orientation <<  cos(theta), -sin(theta), 0,     0,
//                 //                         sin(theta),  cos(theta), 0,     0, 
//                 //                         0,           0,          1,     0,
//                 //                         0,           0,          0,     1;

//                 //     std::cout<<"TETEETET"<<std::endl;
//                 //     open3d::pipelines::registration::RegistrationResult icp_result = open3d::pipelines::registration::RegistrationICP(*source_o3d, *target_o3d, 0.3, orientation, open3d::pipelines::registration::TransformationEstimationPointToPlane(), open3d::pipelines::registration::ICPConvergenceCriteria(20000, 0.00001));

//                 //     std::cout << "ICP TF: "<< icp_result.transformation_ << std::endl;

//                 //     std::cout << "submap idx: " << submap_idx << "Teach idx: " << k <<  std::endl;

//                 //     // std::cout<< "Submap file time: " << submap_pointcloud_files[submap_idx] << "Repeat Ts: " << query_pointcloud_files[k]<<std::endl;
                    


//                 //     Eigen::Matrix4d tf_lidar_base;
//                 //     tf_lidar_base << -1,  0,  0, 0.025,
//                 //                     0,  1,  0, 0.002,
//                 //                     0,  0, -1, 0.843,
//                 //                     0,  0,  0,    1;


//                 //     Eigen::Matrix4d tf_r1_r2 = tf_lidar_base.inverse() * icp_result.transformation_ * tf_lidar_base;





//                 //     Eigen::Matrix4d Tf_teach_gt = reconstruct_tf_from_quaternion(submap_poses, submap_idx);
//                 //     // std::cout << teach_poses[submap_idx].x << teach_poses[submap_idx].y << teach_poses[submap_idx].z <<std::endl;



//                 //     Eigen::Matrix4d repeat_estimate = Tf_teach_gt * tf_r1_r2;




//                 //     all_icp_tfs.push_back(icp_result.transformation_);

//                 //     all_estimates.push_back(repeat_estimate);

//                 //     // pose_error_total.push_back(std::abs(repeat_estimate(0,3) - repeat_poses[k].x) +  std::abs(repeat_estimate(1,3) - repeat_poses[k].y) + std::abs(repeat_estimate(2,3) - repeat_poses[k].z));







//                 //     //Visualisation


//                 // }


//         }

//         // auto min_it = std::min_element(pose_error_total.begin(), pose_error_total.end());
//         // int min_idx = std::distance(pose_error_total.begin(), min_it);

//         // Eigen::Matrix4d icp_tf_final = all_icp_tfs[min_idx];
//         // Eigen::Matrix4d repeat_estimate_final = all_estimates[min_idx];

//         // // Eigen::Matrix4d Tf_repeat_gt = reconstruct_tf_from_quaternion(repeat_poses, k); //Grount truth

//         // // Eigen::Matrix4d T_error = repeat_estimate_final * Tf_repeat_gt.inverse();

//         // // Append absolute differences to the error vectors
//         // pose_error_x.push_back(std::abs(repeat_estimate_final(0,3) - teach_poses[k].x));
//         // pose_error_y.push_back(std::abs(repeat_estimate_final(1,3) - teach_poses[k].y));
//         // pose_error_z.push_back(std::abs(repeat_estimate_final(2,3) - teach_poses[k].z));
//         // std::cout << "Final error: "<< std::abs(repeat_estimate_final(0,3) - teach_poses[k].x) +  std::abs(repeat_estimate_final(1,3) - teach_poses[k].y) + std::abs(repeat_estimate_final(2,3) - teach_poses[k].z) << std::endl;
//         // std::cout << "Intermediate error" <<std::abs(repeat_estimate_final(0,3) - teach_poses[k].x) +  std::abs(repeat_estimate_final(1,3) - teach_poses[k].y) + std::abs(repeat_estimate_final(2,3) - teach_poses[k].z) << std::endl;
//         // // source_o3d->Transform(icp_tf_final);
//         // // visualization::DrawGeometries({source_o3d, target_o3d});







        
//     }
//     // plt::figure_size(1200, 780);
//     // plt::named_plot("Pose Error X", pose_error_x);
//     // plt::named_plot("Pose Error Y", pose_error_y);
//     // plt::named_plot("Pose Error Z", pose_error_z);
//     // plt::xlabel("Sample Index");
//     // plt::ylabel("Error");
//     // plt::title("Pose Errors");
//     // plt::legend();
//     // plt::show();

// }

        
    
//     // plt::figure_size(1200, 780);
//     // plt::named_plot("Pose Error X", pose_error_x);
//     // plt::named_plot("Pose Error Y", pose_error_y);
//     // plt::named_plot("Pose Error Z", pose_error_z);
//     // plt::xlabel("Sample Index");
//     // plt::ylabel("Error");
//     // plt::title("Pose Errors");
//     // plt::legend();
//     // plt::show();




*/




