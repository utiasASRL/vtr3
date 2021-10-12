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
 * \file types.hpp
 * \brief
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace vtr {
namespace lidar {

// clang-format off
#define PCL_ADD_UNION_FLEXIBLE4D \
  union EIGEN_ALIGN16 { \
    float data_p[4]; \
    struct { \
      float flex1; \
      float flex2; \
      float flex3; \
      float flex4; \
    }; \
    struct { \
      float rho; \
      float theta; \
      float phi; \
    }; \
    struct { \
      float dynamic_obs; \
      float total_obs; \
      float dynamic_score; \
    }; \
  };

#define PCL_ADD_EIGEN_MAPS_FLEXIBLE4D \
  inline pcl::Vector4fMap getFlexibleVector4fMap () { return (pcl::Vector4fMap (data_p)); } \
  inline pcl::Vector4fMapConst getFlexibleVector4fMap () const { return (pcl::Vector4fMapConst (data_p)); } \
  inline pcl::Array4fMap getFlexibleArray4fMap () { return (pcl::Array4fMap (data_p)); } \
  inline pcl::Array4fMapConst getFlexibleArray4fMap () const { return (pcl::Array4fMapConst (data_p)); }

#define PCL_ADD_EIGEN_MAPS_POLAR4D \
  inline pcl::Vector3fMap getPolarVector3fMap () { return (pcl::Vector3fMap (data_p)); } \
  inline pcl::Vector3fMapConst getPolarVector3fMap () const { return (pcl::Vector3fMapConst (data_p)); } \
  inline pcl::Vector4fMap getPolarVector4fMap () { return (pcl::Vector4fMap (data_p)); } \
  inline pcl::Vector4fMapConst getPolarVector4fMap () const { return (pcl::Vector4fMapConst (data_p)); } \
  inline pcl::Array3fMap getPolarArray3fMap () { return (pcl::Array3fMap (data_p)); } \
  inline pcl::Array3fMapConst getPolarArray3fMap () const { return (pcl::Array3fMapConst (data_p)); } \
  inline pcl::Array4fMap getPolarArray4fMap () { return (pcl::Array4fMap (data_p)); } \
  inline pcl::Array4fMapConst getPolarArray4fMap () const { return (pcl::Array4fMapConst (data_p)); }

#define PCL_ADD_FLEXIBLE4D \
  PCL_ADD_UNION_FLEXIBLE4D \
  PCL_ADD_EIGEN_MAPS_FLEXIBLE4D \
  PCL_ADD_EIGEN_MAPS_POLAR4D

struct EIGEN_ALIGN16 _PointWithInfo {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  PCL_ADD_FLEXIBLE4D;
  union EIGEN_ALIGN16
  {
    float data_s[4];
    struct
    {
      double time;  // acquisition time of the point
      float normal_score;  // a score of the normal with range [0, 1]
      float icp_score;  // a score for ICP registration
    };
  };
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

struct PointWithInfo : public _PointWithInfo
{
  inline PointWithInfo (const _PointWithInfo &p)
  {
    x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
    normal_x = p.normal_x; normal_y = p.normal_y; normal_z = p.normal_z; data_n[3] = 0.0f;
    rho = p.rho; theta = p.theta; phi = p.phi; data_p[3] = 1.0f;
    time = p.time; normal_score = p.normal_score; icp_score = p.icp_score;
  }

  inline PointWithInfo ()
  {
    x = y = z = 0.0f; data[3] = 1.0f;
    normal_x = normal_y = normal_z = data_n[3] = 0.0f;
    rho = theta = phi = 0.0f; data_p[3] = 1.0f;
    time = normal_score = icp_score = -1.0f;
  }
};

}  // namespace lidar
}  // namespace vtr

POINT_CLOUD_REGISTER_POINT_STRUCT(
    vtr::lidar::PointWithInfo,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, flex1, flex1)
    (float, flex2, flex2)
    (float, flex3, flex3)
    (float, flex4, flex4)
    (double, time, time)
    (float, normal_score, normal_score)
    (float, icp_score, icp_score))

// clang-format on