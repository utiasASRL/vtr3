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
 * \file kruskal_mst_functions.hpp
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#pragma once

#include <list>
#include <map>
#include <unordered_map>
#include <vector>

#include "vtr_pose_graph/id/id.hpp"

namespace vtr {
namespace pose_graph {
namespace simple {
namespace kruskal {

/** \brief A structure to represent a weighted edge in graph */
struct WeightedEdge {
  WeightedEdge() {}
  WeightedEdge(const EdgeId &edge, double weight)
      : edge(edge), weight(weight) {}

  bool operator<(const WeightedEdge &other) const {
    return (weight < other.weight);
  }

  EdgeId edge;
  double weight;
};

/** \brief A structure to represent a subset for union-find */
struct UnionFindSubset {
  VertexId parent;
  VertexId rank;
};

/**
 * \brief A utility function to find set of an element i (uses path compression
 * technique)
 */
VertexId find(std::unordered_map<VertexId, UnionFindSubset> *subsetMap,
              VertexId i) {
  // Find root and make root as parent of i (path compression)
  if (subsetMap->at(i).parent != i) {
    subsetMap->at(i).parent = find(subsetMap, subsetMap->at(i).parent);
  }
  return subsetMap->at(i).parent;
}

/**
 * \brief A function that does union of two sets of x and y (uses union by
 * rank)
 */
void unionSubsets(std::unordered_map<VertexId, UnionFindSubset> *subsetMap,
                  VertexId x, VertexId y) {
  VertexId xroot = find(subsetMap, x);
  VertexId yroot = find(subsetMap, y);

  // Attach smaller rank tree under root of high rank tree (Union by Rank)
  if (subsetMap->at(xroot).rank < subsetMap->at(yroot).rank) {
    subsetMap->at(xroot).parent = yroot;
  } else if (subsetMap->at(xroot).rank > subsetMap->at(yroot).rank) {
    subsetMap->at(yroot).parent = xroot;
  } else {
    // Ranks are the same, make one root and increment its rank by one
    subsetMap->at(yroot).parent = xroot;
    subsetMap->at(xroot).rank++;
  }
}

}  // namespace kruskal
}  // namespace simple
}  // namespace pose_graph
}  // namespace vtr
