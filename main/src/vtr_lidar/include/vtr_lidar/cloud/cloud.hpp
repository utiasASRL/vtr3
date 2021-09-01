// Copyright 2021, Autonomous Space Robotics Lab (ASRL)
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
 * \file cloud.hpp
 * \brief PointXYZ, PointXY, PointCloud class definition
 *
 * \author Hugues Thomas, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <unordered_map>
#include <vector>

#include <time.h>

#include <Eigen/Dense>

#include <vtr_lidar/npm_ply/ply_file_in.h>
#include <vtr_lidar/npm_ply/ply_file_out.h>
#include <vtr_lidar/npm_ply/ply_types.h>

namespace vtr {
namespace lidar {

//------------------------------------------------------------------------------------------------------------
// Point class
// ***********
//
//------------------------------------------------------------------------------------------------------------

class PointXYZ {
 public:
  using Vector3fMap = Eigen::Map<Eigen::Vector3f>;
  using Vector3fMapConst = const Eigen::Map<const Eigen::Vector3f>;
  // Elements
  // ********
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float data[3];
  };

  // Methods
  // *******

  // Constructor
  PointXYZ(float x0 = 0, float y0 = 0, float z0 = 0) : x(x0), y(y0), z(z0) {}

  Vector3fMap getVector3fMap() { return (Vector3fMap(data)); }
  Vector3fMapConst getVector3fMap() const { return (Vector3fMapConst(data)); }

  // array type accessor
  float operator[](int i) const {
    if (i == 0)
      return x;
    else if (i == 1)
      return y;
    else
      return z;
  }

  // opperations
  float dot(const PointXYZ P) const { return x * P.x + y * P.y + z * P.z; }

  float sq_norm() const { return x * x + y * y + z * z; }

  PointXYZ cross(const PointXYZ P) const {
    return PointXYZ(y * P.z - z * P.y, z * P.x - x * P.z, x * P.y - y * P.x);
  }

  PointXYZ& operator+=(const PointXYZ& P) {
    x += P.x;
    y += P.y;
    z += P.z;
    return *this;
  }

  PointXYZ& operator-=(const PointXYZ& P) {
    x -= P.x;
    y -= P.y;
    z -= P.z;
    return *this;
  }

  PointXYZ& operator*=(const float& a) {
    x *= a;
    y *= a;
    z *= a;
    return *this;
  }
};

// Point Opperations
// *****************

inline PointXYZ operator+(const PointXYZ A, const PointXYZ B) {
  return PointXYZ(A.x + B.x, A.y + B.y, A.z + B.z);
}

inline PointXYZ operator-(const PointXYZ A, const PointXYZ B) {
  return PointXYZ(A.x - B.x, A.y - B.y, A.z - B.z);
}

inline PointXYZ operator*(const PointXYZ P, const float a) {
  return PointXYZ(P.x * a, P.y * a, P.z * a);
}

inline PointXYZ operator*(const float a, const PointXYZ P) {
  return PointXYZ(P.x * a, P.y * a, P.z * a);
}

inline PointXYZ operator/(const PointXYZ P, const float a) {
  return PointXYZ(P.x / a, P.y / a, P.z / a);
}

inline PointXYZ operator/(const float a, const PointXYZ P) {
  return PointXYZ(P.x / a, P.y / a, P.z / a);
}

inline std::ostream& operator<<(std::ostream& os, const PointXYZ P) {
  return os << "[" << P.x << ", " << P.y << ", " << P.z << "]";
}

inline bool operator==(const PointXYZ A, const PointXYZ B) {
  return A.x == B.x && A.y == B.y && A.z == B.z;
}

inline PointXYZ floor(const PointXYZ P) {
  return PointXYZ(std::floor(P.x), std::floor(P.y), std::floor(P.z));
}

PointXYZ max_point(const std::vector<PointXYZ>& points);
PointXYZ min_point(const std::vector<PointXYZ>& points);
PointXYZ max_point(const PointXYZ A, const PointXYZ B);
PointXYZ min_point(const PointXYZ A, const PointXYZ B);

//------------------------------------------------------------------------------------------------------------
// Point class 2D
// **************
//
//------------------------------------------------------------------------------------------------------------

class PointXY {
 public:
  // Elements
  // ********

  float x, y;

  // Methods
  // *******

  // Constructor
  PointXY() {
    x = 0;
    y = 0;
  }
  PointXY(float x0, float y0) {
    x = x0;
    y = y0;
  }
  PointXY(PointXYZ P) {
    x = P.x;
    y = P.y;
  }

  // array type accessor
  float operator[](int i) const {
    if (i == 0)
      return x;
    else
      return y;
  }

  // opperations
  float dot(const PointXY P) const { return x * P.x + y * P.y; }

  float sq_norm() const { return x * x + y * y; }

  float cross(const PointXY P) const { return x * P.y - y * P.x; }

  PointXY& operator+=(const PointXY& P) {
    x += P.x;
    y += P.y;
    return *this;
  }

  PointXY& operator-=(const PointXY& P) {
    x -= P.x;
    y -= P.y;
    return *this;
  }

  PointXY& operator*=(const float& a) {
    x *= a;
    y *= a;
    return *this;
  }
};

// Point Opperations
// *****************

inline PointXY operator+(const PointXY A, const PointXY B) {
  return PointXY(A.x + B.x, A.y + B.y);
}

inline PointXY operator-(const PointXY A, const PointXY B) {
  return PointXY(A.x - B.x, A.y - B.y);
}

inline PointXY operator*(const PointXY P, const float a) {
  return PointXY(P.x * a, P.y * a);
}

inline PointXY operator*(const float a, const PointXY P) {
  return PointXY(P.x * a, P.y * a);
}

inline PointXY operator/(const PointXY P, const float a) {
  return PointXY(P.x / a, P.y / a);
}

inline PointXY operator/(const float a, const PointXY P) {
  return PointXY(P.x / a, P.y / a);
}

inline std::ostream& operator<<(std::ostream& os, const PointXY P) {
  return os << "[" << P.x << ", " << P.y << "]";
}

inline bool operator==(const PointXY A, const PointXY B) {
  return A.x == B.x && A.y == B.y;
}

inline PointXY floor(const PointXY P) {
  return PointXY(std::floor(P.x), std::floor(P.y));
}

//------------------------------------------------------------------------------------------------------------
// Pointcloud class
// ****************
//
//------------------------------------------------------------------------------------------------------------

struct PointCloud {
  std::vector<PointXYZ> pts;

  // Must return the number of data points
  inline size_t kdtree_get_point_count() const { return pts.size(); }

  // Returns the dim'th component of the idx'th point in the class:
  // Since this is inlined and the "dim" argument is typically an immediate
  // value, the
  //  "if/else's" are actually solved at compile time.
  inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return pts[idx].x;
    else if (dim == 1)
      return pts[idx].y;
    else
      return pts[idx].z;
  }

  // Optional bounding-box computation: return false to default to a standard
  // bbox computation loop.
  //   Return true if the BBOX was already computed by the class and returned in
  //   "bb" so it can be avoided to redo it again. Look at bb.size() to find out
  //   the expected dimensionality (e.g. 2 or 3 for point clouds)
  template <class BBOX>
  bool kdtree_get_bbox(BBOX& /* bb */) const {
    return false;
  }
};

// Utility function for pointclouds
template <typename T1, typename T2>
void filterVector(std::vector<T1>& vec, std::vector<T2>& scores,
                  T2 filter_value) {
  // Remove every element whose score is < filter_value
  auto vec_address = vec.data();
  vec.erase(std::remove_if(vec.begin(), vec.end(),
                           [&scores, vec_address, filter_value](const T1& f) {
                             return scores[(size_t)(&f - vec_address)] <
                                    filter_value;
                           }),
            vec.end());
}

template <typename T>
void filterVector(std::vector<T>& vec, T filter_value) {
  vec.erase(std::remove_if(
                vec.begin(), vec.end(),
                [filter_value](const float s) { return s < filter_value; }),
            vec.end());
}

template <typename T>
void filterFloatVector(std::vector<T>& vec, std::vector<float>& scores,
                       float filter_value) {
  // Remove every element whose score is < filter_value
  auto vec_address = vec.data();
  vec.erase(std::remove_if(vec.begin(), vec.end(),
                           [&scores, vec_address, filter_value](const T& f) {
                             return scores[(size_t)(&f - vec_address)] <
                                    filter_value;
                           }),
            vec.end());
}

void filterPointCloud(std::vector<PointXYZ>& pts, std::vector<float>& scores,
                      float filter_value);
void filterFloatVector(std::vector<float>& vec, float filter_value);

// PLY reading/saving functions
void save_cloud(std::string dataPath, std::vector<PointXYZ>& points,
                std::vector<PointXYZ>& normals, std::vector<float>& features);
void save_cloud(std::string dataPath, std::vector<PointXYZ>& points,
                std::vector<float>& features);
void save_cloud(std::string dataPath, std::vector<PointXYZ>& points,
                std::vector<PointXYZ>& normals);
void save_cloud(std::string dataPath, std::vector<PointXYZ>& points);

void load_cloud(std::string& dataPath, std::vector<PointXYZ>& points);

void load_cloud(std::string& dataPath, std::vector<PointXYZ>& points,
                std::vector<float>& float_scalar,
                std::string& float_scalar_name, std::vector<int>& int_scalar,
                std::string& int_scalar_name);

void load_cloud_normals(std::string& dataPath, std::vector<PointXYZ>& points,
                        std::vector<PointXYZ>& normals,
                        std::vector<float>& float_scalar,
                        std::string& float_scalar_name,
                        std::vector<int>& int_scalar,
                        std::string& int_scalar_name);

//------------------------------------------------------------------------------------------------------------
// Plane3D class
// *************
//
//------------------------------------------------------------------------------------------------------------

class Plane3D {
 public:
  // Elements
  // ********

  // The plane is define by the equation a*x + b*y + c*z = d. The values (a, b,
  // c) are stored in a PointXYZ called u.
  PointXYZ u;
  float d;

  // Methods
  // *******

  // Constructor
  Plane3D() {
    u.x = 1;
    u.y = 0;
    u.z = 0;
    d = 0;
  }
  Plane3D(const float a0, const float b0, const float c0, const float d0) {
    u.x = a0;
    u.y = b0;
    u.z = c0;
    d = d0;
  }
  Plane3D(const PointXYZ P0, const PointXYZ N0) {
    // Init with point and normal
    u = N0;
    d = N0.dot(P0);
  }
  Plane3D(const PointXYZ A, const PointXYZ B, const PointXYZ C) {
    // Init with three points
    u = (B - A).cross(C - A);
    d = u.dot(A);
  }

  // Method getting distance to one point
  float point_distance(const PointXYZ P) {
    return std::abs((u.dot(P) - d) / std::sqrt(u.sq_norm()));
  }

  // Method getting square distance to one point
  float point_sq_dist(const PointXYZ P) {
    float tmp = u.dot(P) - d;
    return tmp * tmp / u.sq_norm();
  }

  // Method getting distances to some points
  void point_distances(std::vector<PointXYZ>& points,
                       std::vector<float>& distances) {
    if (distances.size() != points.size())
      distances = std::vector<float>(points.size());
    size_t i = 0;
    float inv_norm_u = 1 / std::sqrt(u.sq_norm());
    for (auto& p : points) {
      distances[i] = std::abs((u.dot(p) - d) * inv_norm_u);
      i++;
    }
  }

  int in_range(std::vector<PointXYZ>& points, float threshold) {
    int count = 0;
    float inv_norm_u = 1 / std::sqrt(u.sq_norm());
    for (auto& p : points) {
      if (std::abs((u.dot(p) - d) * inv_norm_u) < threshold) count++;
    }
    return count;
  }
};

}  // namespace lidar
}  // namespace vtr