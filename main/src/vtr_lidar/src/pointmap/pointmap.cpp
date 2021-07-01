#include <vtr_lidar/pointmap/pointmap.hpp>

namespace vtr {
namespace lidar {

void PointMap::update(const std::vector<PointXYZ>& points,
                      const std::vector<PointXYZ>& normals,
                      const std::vector<float>& scores) {
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
      initSample(k, p, normals[i], scores[i]);
      // Update grid limits
      updateLimits(k);
      num_added++;
    } else {
      updateSample(this->samples[k], p, normals[i], scores[i]);
    }
    i++;
  }

  // Update tree
  this->tree.addPoints(this->cloud.pts.size() - num_added,
                       this->cloud.pts.size() - 1);
}

void PointMapMigrator::update(const std::vector<PointXYZ>& points,
                              const std::vector<PointXYZ>& normals,
                              const std::vector<float>& scores) {
  new_map_.updateCapacity(points.size());

  size_t i = 0;
  size_t num_added = 0;
  for (auto& p : points) {
    // Get the corresponding key
    auto k = new_map_.getKey(p);

    // Update the point count
    if (new_map_.samples.count(k) != 0) {
      new_map_.updateSample(new_map_.samples[k], p, normals[i], scores[i]);
    } else {
      // Create a new sample at this location
      new_map_.initSample(k, p, normals[i], scores[i]);
      // Update grid limits
      new_map_.updateLimits(k);
      /// \todo optionally update scores and normals as well.
      auto p_old = p;
      auto p_old_vec = p_old.getVector3fMap();
      p_old_vec = C_on_ * p_old_vec + r_no_ino_;
      auto k2 = old_map_.getKey(p_old);
      if (old_map_.samples.count(k2) != 0) {
        new_map_.movabilities.back() = old_map_.getMovability(k2);
      }
      num_added++;
    }
    i++;
  }

  // Update tree
  new_map_.tree.addPoints(new_map_.cloud.pts.size() - num_added,
                          new_map_.cloud.pts.size() - 1);
}

}  // namespace lidar
}  // namespace vtr