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
 * \file pointmap.cpp
 * \brief <Incremental, SingleExp, MultiExp>PointMap class methods definition
 *
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <vtr_lidar/pointmap/pointmap.hpp>

namespace vtr {
namespace lidar {

void IncrementalPointMap::update(
    const std::vector<PointXYZ>& points, const std::vector<PointXYZ>& normals,
    const std::vector<float>& normal_scores,
    const std::vector<std::pair<int, int>>& movabilities) {
  // Reserve new space if needed
  updateCapacity(points.size());

  size_t i = 0;
  size_t num_added = 0;
  for (auto& p : points) {
    // Get the corresponding key
    auto k = getKey(p);

    // Update the point count
    if (this->samples.count(k) < 1) {
      // Create a new sample at this location
      initSample(k, p, normals[i], normal_scores[i], movabilities[i]);
      num_added++;
    } else {
      updateSample(this->samples[k], p, normals[i], normal_scores[i],
                   movabilities[i]);
    }
    i++;
  }

  // Update tree
  this->tree.addPoints(this->cloud.pts.size() - num_added,
                       this->cloud.pts.size() - 1);

  this->number_of_updates++;
}

void IncrementalPointMapMigrator::update(
    const std::vector<PointXYZ>& points, const std::vector<PointXYZ>& normals,
    const std::vector<float>& normal_scores,
    const std::vector<std::pair<int, int>>& movabilities) {
  new_map_.updateCapacity(points.size());

  size_t i = 0;
  size_t num_added = 0;
  for (auto& p : points) {
    // Get the corresponding key
    auto k = new_map_.getKey(p);

    // Update the point count
    if (new_map_.samples.count(k) != 0) {
      new_map_.updateSample(new_map_.samples[k], p, normals[i],
                            normal_scores[i], movabilities[i]);
    } else {
      // Check if we see this point in the old map and copy over movabilities.
      /// \todo optionally update normal_scores and normals as well.
      auto p_old = p;
      auto p_old_vec = p_old.getVector3fMap();
      p_old_vec = C_on_ * p_old_vec + r_no_ino_;
      auto k2 = old_map_.getKey(p_old);

      auto mb = movabilities[i];
      if (old_map_.samples.count(k2) != 0) {
        const auto& old_mb = old_map_.movabilities[old_map_.samples.at(k2)];
        if (old_mb.second > mb.second) {
          mb.first = old_mb.first;
          mb.second = old_mb.second;
        }
      }

      // Create a new sample at this location
      new_map_.initSample(k, p, normals[i], normal_scores[i], mb);
      num_added++;
    }
    i++;
  }

  // Update tree
  new_map_.tree.addPoints(new_map_.cloud.pts.size() - num_added,
                          new_map_.cloud.pts.size() - 1);

  new_map_.number_of_updates++;
}

void SingleExpPointMap::update(
    const std::vector<PointXYZ>& points, const std::vector<PointXYZ>& normals,
    const std::vector<float>& normal_scores,
    const std::vector<std::pair<int, int>>& movabilities) {
  // Reserve new space if needed
  updateCapacity(points.size());

  size_t i = 0;
  for (auto& p : points) {
    // Ignore dynamic points
    if (remove_dynamic_) {
      // /// Remove points with few observations and observed to be dynamic,
      // /// however, this remove many point on the ground as well due to ray
      // /// tracing resolution.
      // if (movabilities[i].second < min_num_observations_ ||
      //     ((float)movabilities[i].first / (float)movabilities[i].second) >=
      //         min_movability_) {
      //   i++;
      //   continue;
      // }
      /// Only remove points that are for sure dynamic, this leaves some false
      /// dynamic points but hopefully can be removed when combined multiple
      /// experiences.
      /// min_num_observations should always be 1? only keep points with zero
      /// obs which means we never get a good normal for it?
      if (movabilities[i].second >= min_num_observations_ &&
          ((float)movabilities[i].first / (float)movabilities[i].second) >=
              min_movability_) {
        i++;
        continue;
      }
    }

    // Get the corresponding key
    auto k = getKey(p);

    // Update the point count
    if (this->samples.count(k) < 1) {
      // Create a new sample at this location
      initSample(k, p, normals[i], normal_scores[i]);
    } else {
      updateSample(this->samples[k], p, normals[i], normal_scores[i]);
    }
    i++;
  }
}

void MultiExpPointMap::update(
    const std::unordered_map<uint32_t,
                             std::shared_ptr<vtr::lidar::SingleExpPointMap>>&
        single_exp_maps) {
  for (auto iter = single_exp_maps.begin(); iter != single_exp_maps.end();
       ++iter) {
    const auto& map = iter->second;
    const auto& points = map->cloud.pts;
    const auto& normals = map->normals;
    const auto& normal_scores = map->normal_scores;

    // Reserve new space if needed
    updateCapacity(points.size());

    // Clear
    single_exp_obs_updated_.clear();

    size_t i = 0;
    for (auto& p : points) {
      // Get the corresponding key
      auto k = getKey(p);

      // Update the point count
      if (this->samples.count(k) < 1) {
        // Create a new sample at this location
        initSample(k, p, normals[i], normal_scores[i], iter->first);
      } else {
        updateSample(k, this->samples[k], p, normals[i], normal_scores[i],
                     iter->first);
      }
      i++;
    }

    this->number_of_experiences++;
  }
}

void MultiExpPointMap::filter(int num_observations) {
  if (tree_built_) {
    std::string err{"Cannot filter points after KD tree has been built."};
    LOG(ERROR) << err;
    throw std::runtime_error{err};
  }

  if (num_observations <= 1) return;

  for (size_t i = 0; i < cloud.pts.size(); i++) {
    const auto& obs = observations[i];
    const auto& p = cloud.pts[i];
    if (obs < num_observations) samples.erase(getKey(p));
  }
  filterVector(cloud.pts, observations, num_observations);
  filterVector(normals, observations, num_observations);
  filterVector(normal_scores, observations, num_observations);
  filterVector(experiences, observations, num_observations);
  filterVector(observations, num_observations);

  // Check consistency
  if (samples.size() != cloud.pts.size() || samples.size() != normals.size() ||
      samples.size() != normal_scores.size() ||
      samples.size() != experiences.size() ||
      samples.size() != observations.size()) {
    std::string err{"Map inconsistent after filtering points."};
    LOG(ERROR) << err;
    throw std::runtime_error{err};
  }
}

}  // namespace lidar
}  // namespace vtr