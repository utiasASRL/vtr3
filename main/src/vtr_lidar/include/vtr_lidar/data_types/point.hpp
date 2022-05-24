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
 * \file point.hpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// clang-format off
namespace vtr {
namespace lidar {

#define PCL_ADD_UNION_FLEXIBLE1 \
  union EIGEN_ALIGN16 { \
    __uint128_t raw_flex1; \
    float data_flex1[4]; \
    struct { \
      float flex11; \
      float flex12; \
      float flex13; \
      float flex14; \
    }; \
    struct { \
      float rho; \
      float theta; \
      float phi; \
    }; \
    struct { \
      float dynamic_obs; \
      float total_obs; \
      float static_score; \
      float life_time; \
    }; \
    __uint128_t bits; \
    struct { \
      uint32_t bit1; \
      uint32_t bit2; \
      uint32_t bit3; \
      uint32_t bit4; \
    }; \
  };

#define PCL_ADD_EIGEN_MAPS_POLAR4D \
  inline pcl::Vector3fMap getPolarVector3fMap () { return (pcl::Vector3fMap (data_flex1)); } \
  inline pcl::Vector3fMapConst getPolarVector3fMap () const { return (pcl::Vector3fMapConst (data_flex1)); } \
  inline pcl::Vector4fMap getPolarVector4fMap () { return (pcl::Vector4fMap (data_flex1)); } \
  inline pcl::Vector4fMapConst getPolarVector4fMap () const { return (pcl::Vector4fMapConst (data_flex1)); } \
  inline pcl::Array3fMap getPolarArray3fMap () { return (pcl::Array3fMap (data_flex1)); } \
  inline pcl::Array3fMapConst getPolarArray3fMap () const { return (pcl::Array3fMapConst (data_flex1)); } \
  inline pcl::Array4fMap getPolarArray4fMap () { return (pcl::Array4fMap (data_flex1)); } \
  inline pcl::Array4fMapConst getPolarArray4fMap () const { return (pcl::Array4fMapConst (data_flex1)); }

#define PCL_ADD_FLEXIBLE1 \
  PCL_ADD_UNION_FLEXIBLE1 \
  PCL_ADD_EIGEN_MAPS_POLAR4D

#define PCL_ADD_UNION_FLEXIBLE2 \
  union EIGEN_ALIGN16 { \
    __uint128_t raw_flex2; \
    float data_flex2[4]; \
    struct { \
      float flex21; \
      float flex22; \
      float flex23; \
      float flex24; \
    }; \
    struct { \
      int64_t timestamp; \
      float radial_velocity; \
      float normal_score; \
    }; \
    struct { \
      float multi_exp_obs; \
      float unused1; \
      float unused2; \
      float unused3; \
    }; \
  };

#define PCL_ADD_FLEXIBLE2 \
  PCL_ADD_UNION_FLEXIBLE2

struct EIGEN_ALIGN16 _PointWithInfo {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_FLEXIBLE1;
  PCL_ADD_FLEXIBLE2;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointWithInfo : public _PointWithInfo
{
  inline PointWithInfo (const _PointWithInfo &p)
  {
    x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
    normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.0f;
    raw_flex1 = p.raw_flex1;
    raw_flex2 = p.raw_flex2;
  }

  inline PointWithInfo ()
  {
    x = y = z = 0.0f; data[3] = 1.0f;
    normal_x = normal_y = normal_z = data_n[3] = 0.0f;
    raw_flex1 = 0;
    raw_flex2 = 0;
  }

  static constexpr size_t size() { return 16; }
  static constexpr size_t cartesian_offset() { return 0; }
  static constexpr size_t normal_offset() { return 4; }
  static constexpr size_t polar_offset() { return 8; }
  static constexpr size_t flex1_offset() { return 8; }
  static constexpr size_t flex2_offset() { return 12; }
};

}  // namespace lidar
}  // namespace vtr

/// \note we rely on PointCloud2 to load/store point cloud, this registration is
/// required to define PointWithInfo <-> PointCloud2 message conversion
/// functions.
/// \note No very clear yet what fields need to be registered so that the
/// conversion happens without losing any information - currently tested
/// experimentally in test_point_cloud.cpp - should look into pcl library how
/// this MACRO is defined.
POINT_CLOUD_REGISTER_POINT_STRUCT(
    vtr::lidar::PointWithInfo,
    // cartesian coordinates
    (float, x, x)
    (float, y, y)
    (float, z, z)
    // normal vector
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    // polar coordinates + static score + multi exp bit vector
    (float, flex11, flex11)
    (float, flex12, flex12)
    (float, flex13, flex13)
    (float, flex14, flex14)
    // time stamp, normal variance, normal score
    (float, flex21, flex21)
    (float, flex22, flex22)
    (float, flex23, flex23)
    (float, flex24, flex24))
// clang-format on