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
 * \file gps_aligned_frame.cpp
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#if 0
#include <Eigen/Core>
#define ACCEPT_USE_OF_DEPRECATED_PROJ_API_H 1
#include <proj_api.h>
#endif

#include <vtr_pose_graph/relaxation/gps_aligned_frame.hpp>

namespace vtr {
namespace pose_graph {
#if 0
constexpr char GpsAlignedFrame::gpsStream[];

GpsAlignedFrame::GpsAlignedFrame(const RCGraphBase::Ptr &graph, IterType begin,
                                 bool lazy)
    : Base(begin, graph->end(), true, true),
      utmZone_(0),
      C_(Eigen::Matrix3d::Identity()),
      r_(0, 0, 0) {
  // We need to have a GPS stream to make this work; otherwise just leave it as
  // a normal expansion
  if (!graph->hasVertexStream(gpsStream)) {
    LOG(ERROR) << "[GpsAlignedFrame] Graph does not have GPS data!";
    return;
  }

  graph->loadVertexStream(gpsStream);
  std::map<VertexIdType, Eigen::Vector3d> coords;
  Eigen::Vector3d r_utm(0, 0, 0), r_local(0, 0, 0);

  // Load GPS data and find the mean for each vertex
  for (auto it = graph->beginVertex(), ite = graph->endVertex(); it != ite;
       ++it) {
    auto data = it->template retrieveData<GpsMsgType>(gpsStream);
    if (data.size() > 0) {
      Eigen::Vector3d mean(0, 0, 0);

      for (auto &&jt : data) {
        mean[0] += jt->latitude();
        mean[1] += jt->longitude();
        mean[2] += jt->altitude();
      }

      mean /= data.size();
      coords.emplace(it->id(), mean);
      r_utm += mean;

      // Also find the mean for the local expansion points
      r_local += tfMap_[it->id()].r_ba_ina();
    }
  }

  r_utm /= coords.size();
  r_local /= coords.size();

  std::string pstr(
      "+proj=utm +ellps=WGS84 +datum=WGS84 +units=m +no_defs +zone=");
  utmZone_ = uint32_t((r_utm(1) + 180.) / 6.) + 1;
  pstr += std::to_string(utmZone_);
  projPJ pj_utm;

  if (!(pj_utm = pj_init_plus(pstr.c_str()))) {
    LOG(ERROR) << "[GpsAlignedFrame] Could not build UTM projection";
    graph->unloadVertexStream(gpsStream);
    return;
  }

  // Copy the transform map and then reset it; we will replace what we can with
  // GPS data, and the rest will be re-expanded
  Base pf(begin, graph->end(), false);
  tfMap_.clear();

  // Convert the GPS centroid into UTM coordinates in [m]
  projUV src, res;
  src.u = r_utm(1) * DEG_TO_RAD;
  src.v = r_utm(0) * DEG_TO_RAD;
  res = pj_fwd(src, pj_utm);
  r_utm(0) = res.u;
  r_utm(1) = res.v;

  Eigen::Matrix4d N(Eigen::Matrix4d::Zero());

  for (auto &&it : coords) {
    // Project the GPS coordinates of each vertex in UTM coordinates in [m]
    src.u = it.second(1) * DEG_TO_RAD;
    src.v = it.second(0) * DEG_TO_RAD;
    res = pj_fwd(src, pj_utm);

    it.second(0) = res.u;
    it.second(1) = res.v;

    // Horn's method... just trust me
    Eigen::Matrix3d S =
        (pf[it.first].r_ba_ina() - r_local) * (it.second - r_utm).transpose();
    Eigen::Matrix4d Ni;
    Ni << (S(0, 0) + S(1, 1) + S(2, 2)), S(1, 2) - S(2, 1), S(2, 0) - S(0, 2),
        S(0, 1) - S(1, 0), S(1, 2) - S(2, 1), (S(0, 0) - S(1, 1) - S(2, 2)),
        S(0, 1) + S(1, 0), S(2, 0) + S(0, 2), S(2, 0) - S(0, 2),
        S(0, 1) + S(1, 0), (-S(0, 0) + S(1, 1) - S(2, 2)), S(1, 2) + S(2, 1),
        S(0, 1) - S(1, 0), S(2, 0) + S(0, 2), S(1, 2) + S(2, 1),
        (-S(0, 0) - S(1, 1) + S(2, 2));
    N += Ni;
  }

  // The quaternion is the eigenvector with largest eigenvale.  Eigen3 always
  // sorts eigenvalues in ascending order.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(
      N, Eigen::DecompositionOptions::ComputeEigenvectors);
  Eigen::Vector4d q = solver.eigenvectors().col(0);
  double min = solver.eigenvalues()(0);

  for (int i = 1; i < 4; ++i) {
    if (solver.eigenvalues()(i) < min) {
      min = solver.eigenvalues()(i);
      q = solver.eigenvectors().col(i);
    }
  }

  Eigen::Matrix3d C;

  // Convert to rotation matrix
  C << -q(2) * q(2) - q(3) * q(3), q(1) * q(2) - q(3) * q(0),
      q(1) * q(3) + q(2) * q(0), q(1) * q(2) + q(3) * q(0),
      -q(1) * q(1) - q(3) * q(3), q(2) * q(3) - q(1) * q(0),
      q(1) * q(3) - q(2) * q(0), q(2) * q(3) + q(1) * q(0),
      -q(1) * q(1) - q(2) * q(2);
  C_ = (Eigen::Matrix3d::Identity() + 2 * C).transpose();

  // Offset from GPS frame to local frame
  Eigen::Vector3d r_ = C_.transpose() * r_utm - r_local;

  TransformType T(C_, r_);

  for (auto &&it : coords) {
    tfMap_.emplace(
        it.first,
        TransformType(Eigen::Matrix3d(pf[it.first].C_ba() * C_), it.second));
  }

  iter_ = ++graph->begin(tfMap_.begin()->first);

  if (!lazy) {
    computeAll();
  }

  graph->unloadVertexStream(gpsStream);
}
#endif
}  // namespace pose_graph
}  // namespace vtr