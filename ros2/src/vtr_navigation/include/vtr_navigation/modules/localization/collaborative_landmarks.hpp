#if false
#pragma once

#include "asrl/common/utils/hash.hpp"
#include "asrl/navigation/modules/BaseModule.hpp"
#include <asrl/messages/exp_recog_status.pb.h>
#include <asrl/messages/LandmarkHelpers.hpp>
#include <asrl/vision/Types.hpp>
#include <asrl/vision/TypeHelpers.hpp>
#include <asrl/navigation/modules/localization/ExperienceTriage.hpp>

namespace asrl {
namespace navigation {

// TODO look at covisibility for PR literature
// A Connected Component of the landmark match graph
struct ConnectedMatches {
  std::vector<RunId> runs;
  VertexId owner;
//  ConnectedMatches & operator+=(const ConnectedMatches & rhs){
//    runs.reserve(runs.size()+rhs.runs.size());
//    for (const auto & r : rhs.runs) runs.push_back(r);
//    return *this;
//  }
  ConnectedMatches & operator+=(ConnectedMatches && rhs){
    if (runs.empty()) {
      runs = std::move(rhs.runs);
    } else {
      std::swap(runs, rhs.runs); // the rhs is almost always bigger
      runs.reserve(runs.size() + rhs.runs.size());
      std::move(rhs.runs.begin(), rhs.runs.end(), std::back_inserter(runs));
      std::sort(runs.begin(), runs.end());
      runs.erase(std::unique(runs.begin(), runs.end()), runs.end());
    }
    return *this;
  }
};

// Keeps track of connected components of matches
class MatchCache {
public:
  typedef vision::LandmarkId Key;
  typedef VertexId OwnerKey;
  typedef ConnectedMatches Value;
  struct Merger {
    Value operator()(Value lhs, Value && rhs) {
      return lhs += std::move(rhs);
    }
  };
  typedef std::vector<Key> KeyVec;
  struct ValuePack {
    KeyVec keys;
    Value value;
    ValuePack & operator+=(ValuePack && rhs) {
      if (this == &rhs) return *this;
      value = Merger()(std::move(value), std::move(rhs.value));
      if (keys.empty()) {
        keys = std::move(rhs.keys);
      } else {
        keys.reserve(keys.size() + rhs.keys.size());
        for (auto & k : rhs.keys) keys.emplace_back(std::move(k));
      }
      rhs.keys.clear();
      return *this;
    }
  };
  typedef std::list<ValuePack> ValuePackList;
  typedef ValuePackList::iterator iterator;
  typedef ValuePackList::const_iterator const_iterator;
  typedef std::unordered_map<Key, iterator> CacheMap;

  struct ValidIteratorHash {
    size_t operator()(ValuePackList::iterator const & it) const {
      return std::hash<ValuePack*>()(&*it);
    }
  };

private:
  ValuePackList connected_;
  CacheMap id_map_;

public:
  MatchCache(size_t sz = 50000) : id_map_(sz) {}
  /// Get the connected component list
  const ValuePackList & connected() { return connected_; }
  /// Get the key->component map
  const CacheMap & id_map() { return id_map_; }

  /// Find a connected component by key (end if not found)
  const_iterator find(const Key & key) {
    auto found = id_map_.find(key);
    if (found == id_map_.end()) return connected_.end();
    return found->second;
  }
  /// End iterator for the connected component list
  const_iterator end() { return connected_.end(); }

  /// insert a new value, finding and merging connected components
  std::pair<iterator, bool> emplace(ValuePack && vp_) {
    // Insert the new value on the list
    connected_.emplace_front(vp_);
    iterator vpi = connected_.begin();
    bool nomatches = true;

    // Find all the connected components (same match keys)
    KeyVec keys = std::move(vpi->keys);
    vpi->keys.reserve(keys.size());
    for (Key & key : keys) {
      auto found_it = id_map_.find(key);
      if (found_it == id_map_.end()) {
        id_map_[key] = vpi;
        vpi->keys.emplace_back(std::move(key));
      } else if (vpi != found_it->second) {
        auto found = found_it->second;
        nomatches = false;
        for (const Key & fkey : found->keys) id_map_[fkey] = vpi;
        *vpi += std::move(*found);
        connected_.erase(found);
      }
    }
    return std::make_pair(vpi, nomatches);
  }
  std::pair<iterator, bool> insert(const ValuePack & vp) { return emplace(ValuePack(vp)); }

  ValuePackList::iterator erase(const ValuePackList::iterator & it) {
    for (const auto & k : it->keys) {
      id_map_.erase(k);
    }
    return connected_.erase(it);
  }

  /// The number of connected components
  auto size() -> decltype(connected_.size()) { return connected_.size(); }

  /// Remove the oldest components until there are at most n
  void shrink(unsigned n){
    while (size() > n) {
      erase(--connected_.rbegin().base());
    }
  }

  typedef std::function<bool(const ValuePack &)> IsStale;
  /// Remove connected components that the callable considers 'stale'
  void shrink(IsStale is_stale, bool stop_early = true) {
    for (auto it = connected_.rbegin(); it != connected_.rend(); ) {
      bool stale  = is_stale(*it);
      if (stale) erase(--it.base());
      else ++it;
      if (!stale && stop_early) return;
    }
  }
};

////////////////////////////////////////////////////////////////////////////////
/// Decide which landmarks in the local map are the most relevant
////////////////////////////////////////////////////////////////////////////////
class CollaborativeLandmarksModule : public BaseModule {
 public:
  template<class T> using Ptr = std::shared_ptr<T>;

  /// Module name
  static constexpr auto type_str_ = "collaborative_landmarks";

  /// Module Configuration.
  struct Config {
    // exp(-alpha * dt):
    // 0: 1.0
    // 1: 0.3678794411714423
    // 2: 0.1353352832366127
    // 3: 0.0497870683678639
    // 4: 0.0183156388887342
    double similarity_decay = 0.25;
    double prediction_decay = 0.5;
    bool in_the_loop = true;
    bool verbose = false;
    int num_exp = 10;
    bool recommend_landmarks = false;
  };

  /// Module State.
  struct State {
    VertexId vid = VertexId::Invalid(); ///< Represents the state at this vertex
    double Qlive = 0.;
    struct RunState {
      double Q = 0.;
      double P = 0.;
    };
    std::unordered_map<RunId, RunState> runs;
  };

  struct MultiMatch {
    RunId rid;
    std::unordered_set<vision_msgs::FeatureId> down_match;
    std::unordered_set<vision_msgs::FeatureId> up_match;
    std::unordered_set<RunId> down_rid;
    std::unordered_set<RunId> up_rid;
  };
  typedef std::unordered_map<vision_msgs::FeatureId, MultiMatch> MatchMap;
  typedef std::unordered_map<RunId, unsigned> RunCounter;

  /// Uses past localization matches to predict match likelihoods for local map landmarks.
  virtual void run(
          QueryCache & qdata, ///< Information about the live image
          MapCache & mdata, ///< Information about the map
          const std::shared_ptr<const Graph> & graph); ///< The spatio-temporal pose graph

  /// Saves any necessary information to the pose graph
  virtual void updateGraph(
          QueryCache & qdata, // /< Information about the live image
          MapCache & mdata, // /< Information about the map
          const std::shared_ptr<Graph> & graph, // /< The pose grpah
          VertexId live_id);  // /< The live vertex id

  /// Sets the module configuration.
  void setConfig(const std::shared_ptr<Config> & config) { ///< The new config
    config_ = config;
  }

private:

  /// The clique index of landmark matches (both to and from and chained)
  MatchCache match_cache_;
  std::unordered_set<VertexId> submap_ids_;
  std::unordered_map<VertexId, unsigned> cached_counts_;
//  MatchMap match_map_;
  RunCounter lm_counts_;
  unsigned live_lm_count_;

  /// The status message to save to the graph
  status_msgs::ExpRecogStatus status_msg_to_save_;

  // Temporary, will be in the cache
  std::multimap<float,vision::LandmarkId> scored_lms_;

  /// The saved state for incremental similarity
  State state_;

  /// The module configuration
  Ptr<Config> config_;
};

} // navigation
} // asrl
#endif
