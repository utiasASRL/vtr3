#include <vtr_lidar/pointmap/pointmap.hpp>

namespace vtr {
namespace lidar {

void PointMap::update(const std::vector<PointXYZ>& points,
                      const std::vector<PointXYZ>& normals,
                      const std::vector<float>& scores,
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
      initSample(k, p, normals[i], scores[i], movabilities[i]);
      // Update grid limits
      updateLimits(k);
      num_added++;
    } else {
      updateSample(this->samples[k], p, normals[i], scores[i], movabilities[i]);
    }
    i++;
  }

  // Update tree
  this->tree.addPoints(this->cloud.pts.size() - num_added,
                       this->cloud.pts.size() - 1);

  this->number_of_updates++;
}

void PointMapMigrator::update(
    const std::vector<PointXYZ>& points, const std::vector<PointXYZ>& normals,
    const std::vector<float>& scores,
    const std::vector<std::pair<int, int>>& movabilities) {
  new_map_.updateCapacity(points.size());

  size_t i = 0;
  size_t num_added = 0;
  for (auto& p : points) {
    // Get the corresponding key
    auto k = new_map_.getKey(p);

    // Update the point count
    if (new_map_.samples.count(k) != 0) {
      new_map_.updateSample(new_map_.samples[k], p, normals[i], scores[i],
                            movabilities[i]);
    } else {
      // Check if we see this point in the old map and copy over movabilities.
      /// \todo optionally update scores and normals as well.
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
      new_map_.initSample(k, p, normals[i], scores[i], mb);
      // Update grid limits
      new_map_.updateLimits(k);
      num_added++;
    }
    i++;
  }

  // Update tree
  new_map_.tree.addPoints(new_map_.cloud.pts.size() - num_added,
                          new_map_.cloud.pts.size() - 1);

  new_map_.number_of_updates++;
}

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
    /// \todo magic numbers 20 and 0.5
    /// \todo better way of finding dynamic points
    if (movabilities[i].second < 20 ||
        ((float)movabilities[i].first / (float)movabilities[i].second) > 0.5) {
      i++;
      continue;
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

    size_t i = 0;
    for (auto& p : points) {
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

    this->number_of_experiences++;
  }
}

}  // namespace lidar
}  // namespace vtr