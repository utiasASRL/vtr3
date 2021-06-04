//
//
//		0==========================0
//		|    Local feature test    |
//		0==========================0
//
//		version 1.0 :
//			>
//
//---------------------------------------------------
//
//		Cloud source :
//		Define usefull Functions/Methods
//
//----------------------------------------------------
//
//		Hugues THOMAS - 10/02/2017
//

#include "vtr_lidar/cloud/cloud.h"

// Getters
// *******

PointXYZ max_point(const std::vector<PointXYZ>& points) {
  // Initialize limits
  PointXYZ maxP(points[0]);

  // Loop over all points
  for (auto p : points) {
    if (p.x > maxP.x) maxP.x = p.x;

    if (p.y > maxP.y) maxP.y = p.y;

    if (p.z > maxP.z) maxP.z = p.z;
  }

  return maxP;
}

PointXYZ min_point(const std::vector<PointXYZ>& points) {
  // Initialize limits
  PointXYZ minP(points[0]);

  // Loop over all points
  for (auto p : points) {
    if (p.x < minP.x) minP.x = p.x;

    if (p.y < minP.y) minP.y = p.y;

    if (p.z < minP.z) minP.z = p.z;
  }

  return minP;
}

PointXYZ max_point(const PointXYZ A, const PointXYZ B) {
  // Initialize limits
  PointXYZ maxP(A);
  if (B.x > maxP.x) maxP.x = B.x;
  if (B.y > maxP.y) maxP.y = B.y;
  if (B.z > maxP.z) maxP.z = B.z;
  return maxP;
}

PointXYZ min_point(const PointXYZ A, const PointXYZ B) {
  // Initialize limits
  PointXYZ maxP(A);
  if (B.x < maxP.x) maxP.x = B.x;
  if (B.y < maxP.y) maxP.y = B.y;
  if (B.z < maxP.z) maxP.z = B.z;
  return maxP;
}

void filter_pointcloud(std::vector<PointXYZ>& pts, std::vector<float>& scores,
                       float filter_value) {
  // Remove every points whose score is < filter_value
  auto pts_address = pts.data();
  pts.erase(
      std::remove_if(pts.begin(), pts.end(),
                     [&scores, pts_address, filter_value](const PointXYZ& p) {
                       return scores[(size_t)(&p - pts_address)] < filter_value;
                     }),
      pts.end());
}

void filter_floatvector(std::vector<float>& vec, std::vector<float>& scores,
                        float filter_value) {
  // Remove every element whose score is < filter_value
  auto vec_address = vec.data();
  vec.erase(
      std::remove_if(vec.begin(), vec.end(),
                     [&scores, vec_address, filter_value](const float& f) {
                       return scores[(size_t)(&f - vec_address)] < filter_value;
                     }),
      vec.end());
}

void filter_floatvector(std::vector<float>& vec, float filter_value) {
  vec.erase(std::remove_if(
                vec.begin(), vec.end(),
                [filter_value](const float s) { return s < filter_value; }),
            vec.end());
}

// Debug functions
// ***************

void save_cloud(std::string dataPath, std::vector<PointXYZ>& points,
                std::vector<PointXYZ>& normals, std::vector<float>& features) {
  // Variables
  uint64_t num_points = points.size();
  uint64_t num_normals = normals.size();
  uint64_t num_features = features.size() / num_points;

  // Safe check
  if (num_features * num_points != features.size()) {
    std::cout << "Warning: features dimension do not match point cloud"
              << std::endl;
    std::cout << "         ply saving canceled" << std::endl;
    return;
  }
  if (num_normals != num_points && num_normals != 0) {
    std::cout << "Warning: normal dimension do not match point cloud"
              << std::endl;
    std::cout << "         ply saving canceled" << std::endl;
    return;
  }

  // Open file
  npm::PLYFileOut file(dataPath);

  // Push fields
  file.pushField(num_points, 3, npm::PLY_FLOAT, {"x", "y", "z"}, points);
  if (num_normals > 0)
    file.pushField(num_points, 3, npm::PLY_FLOAT, {"nx", "ny", "nz"}, normals);

  std::vector<std::vector<float>> fields(num_features);
  for (size_t i = 0; i < num_features; i++) {
    char buffer[100];
    sprintf(buffer, "f%d", (int)i);
    fields[i] = std::vector<float>(features.begin() + i * num_points,
                                   features.begin() + (i + 1) * num_points);
    file.pushField(num_points, 1, npm::PLY_FLOAT, {std::string(buffer)},
                   fields[i]);
  }
  file.write();
}

void save_cloud(std::string dataPath, std::vector<PointXYZ>& points,
                std::vector<PointXYZ>& normals) {
  std::vector<float> no_f;
  save_cloud(dataPath, points, normals, no_f);
}

void save_cloud(std::string dataPath, std::vector<PointXYZ>& points,
                std::vector<float>& features) {
  std::vector<PointXYZ> no_norms;
  save_cloud(dataPath, points, no_norms, features);
}

void save_cloud(std::string dataPath, std::vector<PointXYZ>& points) {
  std::vector<float> no_f;
  save_cloud(dataPath, points, no_f);
}

void load_cloud(std::string& dataPath, std::vector<PointXYZ>& points) {
  std::vector<float> float_scalar;
  std::string float_scalar_name = "";
  std::vector<int> int_scalar;
  std::string int_scalar_name = "";

  load_cloud(dataPath, points, float_scalar, float_scalar_name, int_scalar,
             int_scalar_name);
}

void load_cloud(std::string& dataPath, std::vector<PointXYZ>& points,
                std::vector<float>& float_scalar,
                std::string& float_scalar_name, std::vector<int>& int_scalar,
                std::string& int_scalar_name) {
  // Variables
  uint64_t num_points(0);
  std::vector<npm::PLYType> types;
  std::vector<std::string> properties;
  char buffer[500];

  size_t float_str_n = strlen(float_scalar_name.c_str());
  size_t int_str_n = strlen(int_scalar_name.c_str());

  // Open file
  npm::PLYFileIn file(dataPath);

  // Read Header
  if (!file.read(&num_points, &types, &properties)) {
    std::cout << "ERROR: wrong ply header" << std::endl;
    return;
  }

  // Prepare containers
  points.reserve(num_points);
  float_scalar.reserve(num_points);
  int_scalar.reserve(num_points);

  // Get the points
  for (size_t i = 0; i < properties.size(); i++) {
    if (properties[i].size() == 1 &&
        strncmp(properties[i].c_str(), "x", 1) == 0)
      file.getField(i, 3, points);

    if (properties[i].size() == float_str_n &&
        strncmp(properties[i].c_str(), float_scalar_name.c_str(),
                float_str_n) == 0)
      file.getField(i, 1, float_scalar);

    if (properties[i].size() == int_str_n &&
        strncmp(properties[i].c_str(), int_scalar_name.c_str(), int_str_n) == 0)
      file.getField(i, 1, int_scalar);
  }

  return;
}

void load_cloud_normals(std::string& dataPath, std::vector<PointXYZ>& points,
                        std::vector<PointXYZ>& normals,
                        std::vector<float>& float_scalar,
                        std::string& float_scalar_name,
                        std::vector<int>& int_scalar,
                        std::string& int_scalar_name) {
  // Variables
  uint64_t num_points(0);
  std::vector<npm::PLYType> types;
  std::vector<std::string> properties;
  char buffer[500];

  size_t float_str_n = strlen(float_scalar_name.c_str());
  size_t int_str_n = strlen(int_scalar_name.c_str());

  // Open file
  npm::PLYFileIn file(dataPath);

  // Read Header
  if (!file.read(&num_points, &types, &properties)) {
    std::cout << "ERROR: wrong ply header" << std::endl;
    return;
  }

  // Prepare containers
  points.reserve(num_points);
  normals.reserve(num_points);
  float_scalar.reserve(num_points);
  int_scalar.reserve(num_points);

  // Get the points
  for (size_t i = 0; i < properties.size(); i++) {
    if (properties[i].size() == 1 &&
        strncmp(properties[i].c_str(), "x", 1) == 0)
      file.getField(i, 3, points);

    if (properties[i].size() == 2 &&
        strncmp(properties[i].c_str(), "nx", 2) == 0)
      file.getField(i, 3, normals);

    if (properties[i].size() == float_str_n &&
        strncmp(properties[i].c_str(), float_scalar_name.c_str(),
                float_str_n) == 0)
      file.getField(i, 1, float_scalar);

    if (properties[i].size() == int_str_n &&
        strncmp(properties[i].c_str(), int_scalar_name.c_str(), int_str_n) == 0)
      file.getField(i, 1, int_scalar);
  }

  return;
}
