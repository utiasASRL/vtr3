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
 * \file grid_subsampling.hpp
 * \brief Grid subsampling utilities
 *
 * \author Hugues Thomas, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include "vtr_lidar/types.hpp"

namespace vtr {
namespace lidar {

class PointXYZ {
 public:
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

  // array type accessor
  float operator[](int i) const {
    if (i == 0)
      return x;
    else if (i == 1)
      return y;
    else
      return z;
  }

  // operations
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

// Point Operations
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

template <class PointT>
PointXYZ max_point(const std::vector<PointT>& points) {
  // Initialize limits
  PointXYZ maxP(points[0].x, points[0].y, points[0].z);
  // Loop over all points
  for (auto p : points) {
    if (p.x > maxP.x) maxP.x = p.x;
    if (p.y > maxP.y) maxP.y = p.y;
    if (p.z > maxP.z) maxP.z = p.z;
  }
  return maxP;
}

template <class PointT>
PointXYZ min_point(const std::vector<PointXYZ>& points) {
  // Initialize limits
  PointXYZ minP(points[0].x, points[0].y, points[0].z);
  // Loop over all points
  for (auto p : points) {
    if (p.x < minP.x) minP.x = p.x;
    if (p.y < minP.y) minP.y = p.y;
    if (p.z < minP.z) minP.z = p.z;
  }
  return minP;
}


}  // namespace lidar
}  // namespace vtr