#include <vtr_vision/features/bow/incremental_bow_trainer.hpp>
#include <vtr_vision/features/matcher/asrl_feature_matcher.hpp>
#include <omp.h>
#include <list>

namespace vtr {
namespace vision {

IncrementalBOWTrainer::IncrementalBOWTrainer(double _clusterSize, bool first_not_mean) :
  cluster_size_(_clusterSize), already_clustered_(0), first_not_mean_(first_not_mean) {
}

IncrementalBOWTrainer::~IncrementalBOWTrainer() {
}

void IncrementalBOWTrainer::clear() {
  already_clustered_ = 0;
  centers_.release();
}

cv::Mat IncrementalBOWTrainer::cluster() const {
  CV_Assert(!descriptors.empty());

  // We need to convert the cv::Mat array coming in, to a single contiguous matrix
  int desc_count = 0;
  for(size_t i = 0; i < descriptors.size(); i++)
    desc_count += descriptors[i].rows;
  cv::Mat mergedDescriptors(desc_count, descriptors[0].cols,
      descriptors[0].type());
  for(size_t i = 0, start = 0; i < descriptors.size(); i++) {
    cv::Mat submut = mergedDescriptors.rowRange((int)start,
                                                (int)(start + descriptors[i].rows));
    descriptors[i].copyTo(submut);
    start += descriptors[i].rows;
  }

  // Do the clustering (non-incremental)
  cv::Mat centers = cluster(mergedDescriptors);

  // CHEATING, BUT OPENCV WON'T LET US BE CORRECT :(
  // Save the centers and count for incremental
  auto* mutable_this = const_cast<IncrementalBOWTrainer*>(this);
  if (mutable_this->centers_.empty()) {
    mutable_this->centers_ = centers;
  } else {
    mutable_this->centers_.push_back(centers);
  }
  mutable_this->already_clustered_ = desc_count;

  return centers_;
}

cv::Mat IncrementalBOWTrainer::cluster(const cv::Mat& descriptors) const {

  // Do the clustering to get Initial Centers (ICs)
  std::vector<unsigned> initial_center_ids = clusterFirstByIndex(descriptors);

  // Copy all the IC descriptors
  cv::Mat vocabulary;
  for (const auto & i : initial_center_ids) {
    vocabulary.push_back(descriptors.row(i));
  }

  // Refine centers if required
  if (!first_not_mean_) {
    vocabulary = refineCenters(descriptors, vocabulary);
  }

  return vocabulary;
}

std::vector<unsigned> IncrementalBOWTrainer::clusterFirstByIndex(const cv::Mat & descriptors,
                                                                 std::vector<unsigned> * closest) const {
  CV_Assert(!descriptors.empty());

  // Create initial centres guaranteeing a centre distance < minDist //

  std::vector<unsigned> initial_center_ids;
  initial_center_ids.push_back(0);

  // Allocate the assignment output if present
  if (closest) closest->resize(descriptors.rows);

  for (int i = 1; i < descriptors.rows; i++) {
    float min_dist = cluster_size_;
    unsigned new_j = initial_center_ids.size();
    unsigned min_j = new_j;
#pragma omp parallel if (initial_center_ids.size() > 100)
    for (int j = 0; j < (int)initial_center_ids.size(); j++) {
      // TODO (old) different feature types
      float my_dist = ASRLFeatureMatcher::surfmatch((float*)descriptors.row(i).data,
                                                    (float*)descriptors.row(initial_center_ids[j]).data,
                                                    64);
#pragma omp critical
      if (my_dist < min_dist) {
        min_dist = my_dist;
        min_j = j;
      }
    }

    // Add new cluster if outside of range
    if (min_j == new_j) initial_center_ids.push_back(i);
    // Save the cluster center if not
    if (closest) (*closest)[i] = min_j;
  }

  return initial_center_ids;
}

cv::Mat IncrementalBOWTrainer::refineCenters(const cv::Mat & descriptors,
                                             const cv::Mat & initial_centers) const {
  // Assign each descriptor to its closest centre //

  // Loop through all the descriptors again
  // TODO (old): Consider a kd-tree for this search
  std::vector<std::list<cv::Mat> > clusters;
  clusters.resize(initial_centers.rows);
#pragma omp parallel for schedule(dynamic, 200)
  for (int i = 0; i < descriptors.rows; i++) {
    int index = 0;
    double dist, minDist = DBL_MAX;
    for (int j = 0; j < initial_centers.rows; j++) {
      dist = cv::norm(descriptors.row(i),initial_centers.row(j));
      if (dist < minDist) {
        minDist = dist;
        index = j;
      }
    }
#pragma omp critical // Order doesn't matter here
    clusters[index].push_back(descriptors.row(i));
  }

  // Calculate the centre mean for each cluster //

  // Loop through all the clusters
  cv::Mat vocabulary;
#pragma omp parallel for schedule(static, 1) ordered
  for (int i = 0; i < (int)clusters.size(); i++) {
    // TODO (old): Throw away small clusters
    // TODO (old): Make this configurable
    // TODO (old): Re-assign?
    // if (clusters[i].size() < 3) continue;

    cv::Mat centre = cv::Mat::zeros(1,descriptors.cols,descriptors.type());
    for (std::list<cv::Mat>::iterator Ci = clusters[i].begin(); Ci != clusters[i].end(); Ci++) {
      centre += *Ci;
    }
    centre /= (double)clusters[i].size();
#pragma omp ordered // Ordered so it's identical to non omp.
    vocabulary.push_back(centre);
  }

  return vocabulary;
}

}
}
