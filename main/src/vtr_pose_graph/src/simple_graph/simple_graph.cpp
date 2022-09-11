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
 * \file simple_graph.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include "vtr_pose_graph/simple_graph/simple_graph.hpp"

#include "vtr_logging/logging.hpp"
#include "vtr_pose_graph/simple_graph/kruskal_mst_functions.hpp"
#include "vtr_pose_graph/simple_graph/simple_iterator.hpp"

namespace vtr {
namespace pose_graph {
namespace simple {

SimpleGraph::SimpleGraph(const EdgeList& edges) {
  for (auto it = edges.begin(); it != edges.end(); ++it) this->addEdge(*it);
}

SimpleGraph::SimpleGraph(const VertexList& vertices, bool cyclic) {
  for (auto it = vertices.begin(); it != vertices.end(); ++it) {
    auto itn = std::next(it);
    if (itn != vertices.end())
      this->addEdge(*it, *itn);
    else if (cyclic)
      this->addEdge(*it, *vertices.begin());
  }
}

void SimpleGraph::addVertex(const VertexId& vertex) {
  // Insert, but don't overwrite if this vertex already exists
  node_map_.emplace(vertex, SimpleNode(vertex));
}

void SimpleGraph::addEdge(const EdgeId& edge) {
  // Get iterators to id-node pairs, or create them
  auto node1 = (node_map_.emplace(edge.id1(), SimpleNode(edge.id1()))).first;
  auto node2 = (node_map_.emplace(edge.id2(), SimpleNode(edge.id2()))).first;

  // Check that edge does not exist
  const std::list<VertexId>& adj = node1->second.getAdjacent();
  if (std::find(adj.begin(), adj.end(), edge.id2()) != adj.end()) {
    CLOG(ERROR, "pose_graph") << "Edge " << edge << " already exists";
    throw std::invalid_argument("Tried to add edge that already exists!");
  }

  // Add adjacency
  node1->second.addAdjacent(node2->first);
  node2->second.addAdjacent(node1->first);

  // Add new edge
  edges_.push_back(edge);
}

void SimpleGraph::addEdge(const VertexId& id1, const VertexId& id2) {
  this->addEdge(EdgeId(id1, id2));
}

auto SimpleGraph::begin(VertexId root, double max_depth,
                        const eval::mask::Ptr& mask) const -> OrderedIter {
  return beginBfs(root, max_depth, mask);
}

auto SimpleGraph::beginBfs(VertexId root, double max_depth,
                           const eval::mask::Ptr& mask) const -> OrderedIter {
  // Handle the case of and empty graph separately
  if (node_map_.size() == 0) return OrderedIter::End(this);

  // If we didn't specify a root, pick one arbitrarily
  if (root == VertexId::Invalid()) root = node_map_.begin()->first;

  return OrderedIter::BFS(this, root, max_depth, mask);
}

auto SimpleGraph::beginDfs(VertexId root, double max_depth,
                           const eval::mask::Ptr& mask) const -> OrderedIter {
  // Handle the case of and empty graph separately
  if (node_map_.size() == 0) return OrderedIter::End(this);

  // If we didn't specify a root, pick one arbitrarily
  if (root == VertexId::Invalid()) root = node_map_.begin()->first;

  return OrderedIter::DFS(this, root, max_depth, mask);
}

auto SimpleGraph::beginDijkstra(VertexId root, double max_depth,
                                const eval::mask::Ptr& mask,
                                const eval::weight::Ptr& weight) const
    -> OrderedIter {
  // Handle the case of and empty graph separately
  if (node_map_.size() == 0) return OrderedIter::End(this);

  // If we didn't specify a root, pick one arbitrarily
  if (root == VertexId::Invalid()) root = node_map_.begin()->first;

  return OrderedIter::Dijkstra(this, root, max_depth, mask, weight);
}

SimpleGraph::OrderedIter SimpleGraph::end() const {
  return OrderedIter::End(this);
}

auto SimpleGraph::pathDecomposition(ComponentList& paths,
                                    ComponentList& cycles) const
    -> JunctionSet {
  JunctionSet junctions;

  if (node_map_.size() == 0) return junctions;

  std::list<VertexId> searchQueue;
  std::unordered_set<VertexId> exploredSet;
  // Find a vertex of degree 1 (path termination) or >2 (junction) to start the
  // search at
  for (auto&& it : node_map_) {
    if (it.second.getAdjacent().size() != 2) {
      searchQueue.push_back(it.first);
      break;
    }
  }

  // Special case: 2-regular graph.  If the above loop doesn't break, the graph
  // is a single cycle In this case, just break it at some arbitrary point and
  // don't return a junction vertex
  if (searchQueue.empty()) {
    std::list<VertexId> path;
    for (auto it = this->beginDfs(node_map_.begin()->first); it != this->end();
         ++it) {
      path.push_back(it->v());
    }

    path.push_back(node_map_.begin()->first);
    cycles.push_back(path);
    return junctions;
  }

  // Loop through all junction vertices
  while (!searchQueue.empty()) {
    VertexId root = searchQueue.front();
    searchQueue.pop_front();

    // Only search a junction once
    if (junctions.find(root) != junctions.end()) continue;

    junctions.insert(root);
    exploredSet.insert(root);

    for (auto&& it : node_map_.at(root).getAdjacent()) {
      // Don't double explore paths.  We only need to check this once, as we
      // always explore an entire path in one go
      if (exploredSet.find(it) != exploredSet.end()) continue;

      std::list<VertexId> path = {root};
      VertexId branch(it);

      // Expand outward until we hit something that branches/dead-ends
      while (node_map_.at(branch).getAdjacent().size() == 2) {
        // The next vertex is the neighbour that isn't in the path yet
        const std::list<VertexId>& neighbours =
            node_map_.at(branch).getAdjacent();
        VertexId next(path.back() == neighbours.front() ? neighbours.back()
                                                        : neighbours.front());

        path.push_back(branch);
        exploredSet.insert(branch);
        branch = next;
      }

      path.push_back(branch);
      if (node_map_.at(branch).getAdjacent().size() > 2) {
        searchQueue.push_back(branch);
      } else {
        // The case of deg(v) == 1 isn't a junction, but it's still a special
        // case and we probably want to plot it on a map as it likely represents
        // a point of interest
        junctions.insert(branch);
      }

      if (branch != root) {
        paths.push_back(path);
      } else {
        cycles.push_back(path);
      }
    }
  }

  return junctions;
}

SimpleGraph SimpleGraph::getSubgraph(const VertexVec& nodes,
                                     const eval::mask::Ptr& mask) const {
  // Check for nodes
  if (nodes.size() == 0) {
    CLOG(ERROR, "pose_graph") << "No nodes specified for subgraph";
    throw std::invalid_argument("No nodes specified for subgraph");
  }

  // Collect edges
  std::list<EdgeId> subgraphEdges;
  std::unordered_set<VertexId> nodeSet(nodes.begin(), nodes.end());

  // For each node in inputs
  for (auto&& it : nodes) {
    if (!mask->operator[](it)) continue;

    // Check that node is part of graph
    auto nodeIter = node_map_.find(it);
    if (nodeIter == node_map_.end()) {
      CLOG(ERROR, "pose_graph") << "An input node did not exist in graph.";
      throw std::invalid_argument("An input node did not exist in graph.");
    }

    // Get node and adjacent node references
    const SimpleNode& node = nodeIter->second;
    const std::list<VertexId>& adj = node.getAdjacent();

    // For each adjacent node
    for (auto&& adjIter : adj) {
      if (!mask->operator[](adjIter) || !mask->operator[](EdgeId(it, adjIter)))
        continue;

      // Check if adjacent is in 'nodes'
      auto foundIter = nodeSet.find(adjIter);

      // If both nodes were found, add this edge to our subgraph
      if (foundIter != nodeSet.end())
        subgraphEdges.push_back(EdgeId(node.getId(), adjIter));
    }
  }

  // Make edges unique
  subgraphEdges.sort();
  subgraphEdges.unique();

  // Make subgraph
  SimpleGraph subgraph(subgraphEdges);

  // If there was only one vertex, add it.
  if (nodes.size() == 1) subgraph.addVertex(nodes[0]);

  return subgraph;
}

SimpleGraph SimpleGraph::getSubgraph(VertexId root_id,
                                     const eval::mask::Ptr& mask) const {
  return dijkstraTraverseToDepth(
      root_id, 0.0, std::make_shared<eval::weight::ConstEval>(1, 1), mask);
}

SimpleGraph SimpleGraph::getSubgraph(VertexId root_id, double max_depth,
                                     const eval::mask::Ptr& mask) const {
  return dijkstraTraverseToDepth(
      root_id, max_depth, std::make_shared<eval::weight::ConstEval>(1, 1),
      mask);
}

SimpleGraph& SimpleGraph::operator+=(const SimpleGraph& other) {
  for (auto&& it : other.node_map_) {
    // Add a new node, or retreive the existing one from $this
    auto node = node_map_.emplace(it.first, SimpleNode(it.first)).first;
    auto adj = node->second.getAdjacent();

    // For all neighbours of the node in the other graph...
    for (auto&& neighbour : it.second.getAdjacent()) {
      // If $node was not adjacent to $neighbour in $this, and $neighbour is
      // currently in $this...
      if (std::find(adj.begin(), adj.end(), neighbour) == adj.end() &&
          node_map_.find(neighbour) != node_map_.end()) {
        // Update the adjacency list of each node, and push the edge onto the
        // edge list
        node_map_[neighbour].addAdjacent(it.first);
        node->second.addAdjacent(neighbour);
        edges_.push_back(EdgeId(it.first, neighbour));
      }
      // If the neighbour was found, then the edge existed in both graphs
      // If $neighbour was not in the node map, then it must be in $other, and
      // this condition will trigger later
    }
  }

  return *this;
}

SimpleGraph SimpleGraph::dijkstraTraverseToDepth(
    VertexId root_id, double max_depth, const eval::weight::Ptr& weights,
    const eval::mask::Ptr& mask) const {
  // Initialized result
  SimpleGraph subgraph;

  // Check that root exists
  auto rootIter = node_map_.find(root_id);
  if (rootIter == node_map_.end()) {
    CLOG(ERROR, "pose_graph") << "Root node did not exist in graph.";
    throw std::invalid_argument("Root node did not exist in graph.");
  }

  // Check valid depth input
  if (max_depth < 0.0) {
    CLOG(ERROR, "pose_graph") << "max_depth must >=0 with 0 meaning no limit";
    throw std::invalid_argument("max_depth must >=0 with 0 meaning no limit.");
  }

  // Distance to each node (also tracks who has been visited)
  std::unordered_map<VertexId, double> nodeDepths;

  // Parent of each node (nodeId/parentId)
  BacktraceMap nodeParents;

  // Init search queue
  // * Note this is stored in <depth, <nodeId,parentId> > so that we can
  //   sort and process minimum depth first
  using DepthNodeParent = std::pair<double, std::pair<VertexId, VertexId>>;
  std::list<DepthNodeParent> searchQueue;
  searchQueue.push_back(
      std::make_pair(0.0, std::make_pair(root_id, VertexId::Invalid())));

  // Until our search queue is empty
  while (!searchQueue.empty()) {
    // Pop front node (next shortest depth from root)
    DepthNodeParent currDepthNodeParent = searchQueue.front();
    searchQueue.pop_front();

    // Get current node depth, id and reference
    double currNodeDepth = currDepthNodeParent.first;
    VertexId currNodeId = currDepthNodeParent.second.first;
    VertexId currNodeParentId = currDepthNodeParent.second.second;
    const SimpleNode& currNode = node_map_.at(currNodeId);

    // We can add the edge without further checks, as we can't ever reach the
    // same node twice from the same parent
    if (currNodeParentId != VertexId::Invalid()) {
      subgraph.addEdge(EdgeId(currNodeId, currNodeParentId));
    } else if (mask->operator[](currNodeId)) {
      subgraph.addVertex(currNodeId);  /// special case for the root vertex
    }

    // This shouldn't be necessary, as we don't add masked out vertices to the
    // queue, but check in case
    if (!mask->operator[](currNodeId)) continue;

    // Insert garbage here to test if the key exists, and get a reference to the
    // correct map position for assignment
    auto depthPair = nodeDepths.insert(std::make_pair(currNodeId, -1));

    // Check if the depth was already contained in the map (node already
    // visited)
    if (!depthPair.second) {
      // Double check that recorded depth is indeed less than or equal to
      // proposed depth
      if ((*depthPair.first).second > currNodeDepth) {
        CLOG(ERROR, "pose_graph") << "found a shorter path...";
        throw std::runtime_error("found a shorter path...");
      }
      continue;
    }

    // Mark node as visited (record the depth it was visited at)
    (*depthPair.first).second = currNodeDepth;
    nodeParents[currNodeId] = currNodeParentId;

    // For each adjacent node
    for (auto adjIter = currNode.getAdjacent().begin();
         adjIter != currNode.getAdjacent().end(); ++adjIter) {
      // Get child id
      VertexId childId = *adjIter;

      // Make edge
      EdgeId currEdge(currNode.getId(), childId);

      if (!mask->operator[](currEdge) || !mask->operator[](childId)) continue;

      // Get edge weight (default weight evaluator always returns 1)
      double edgeWeight = weights->operator[](currEdge);

      // Calculate depth to visit node
      double newChildDepth = currNodeDepth + edgeWeight;

      // Check if we have already visited this node
      auto childDepthIter = nodeDepths.find(childId);
      if (childDepthIter != nodeDepths.end()) {
        // Double check that recorded depth is indeed less than or equal to
        // proposed depth
        if (childDepthIter->second > newChildDepth) {
          CLOG(ERROR, "pose_graph") << "found a shorter path...";
          throw std::runtime_error("found a shorter path...");
        }
        continue;
      }

      // If visiting node is less than max depth, add to queue and add to graph
      if (max_depth == 0.0 || newChildDepth <= max_depth) {
        searchQueue.push_back(
            std::make_pair(newChildDepth, std::make_pair(childId, currNodeId)));
      }
    }

    // After adding new adjacent edges, we sort queue to make sure we try
    // shortest depths first
    searchQueue.sort();
  }

  return subgraph;
}

SimpleGraph SimpleGraph::dijkstraSearch(VertexId root_id, VertexId search_id,
                                        const eval::weight::Ptr& weights,
                                        const eval::mask::Ptr& mask) const {
  VertexVec search_ids;
  search_ids.push_back(search_id);
  return this->dijkstraMultiSearch(root_id, search_ids, weights, mask);
}

SimpleGraph SimpleGraph::dijkstraMultiSearch(
    VertexId root_id, const VertexVec& search_ids,
    const eval::weight::Ptr& weights, const eval::mask::Ptr& mask) const {
  // Check that root exists
  NodeMap::const_iterator rootIter = node_map_.find(root_id);
  if (rootIter == node_map_.end()) {
    CLOG(ERROR, "pose_graph") << "Root node did not exist in graph.";
    throw std::invalid_argument("Root node did not exist in graph.");
  }

  // Check valid search input
  if (search_ids.size() == 0) {
    CLOG(ERROR, "pose_graph") << "Root node did not exist in graph.";
    throw std::invalid_argument("search_ids size is zero.");
  }

  // Init search size variables
  unsigned long numSearches = search_ids.size();
  unsigned long numFound = 0;

  // Distance to each node (also tracks who has been visited)
  std::unordered_map<VertexId, double> nodeDepths;

  // Parent of each node (nodeId/parentId)
  BacktraceMap nodeParents;

  // Init search queue
  // * Note this is stored in <depth, <nodeId,parentId> > so that we can
  //   sort and process minimum depth first
  using DepthNodeParent = std::pair<double, std::pair<VertexId, VertexId>>;
  std::list<DepthNodeParent> searchQueue;
  searchQueue.push_back(
      std::make_pair(0.0, std::make_pair(root_id, VertexId::Invalid())));

  // Until our search queue is empty
  while (!searchQueue.empty() && numFound < numSearches) {
    // Pop front node (next shortest depth from root)
    DepthNodeParent currDepthNodeParent = searchQueue.front();
    searchQueue.pop_front();

    // Get current node depth, id and reference
    double currNodeDepth = currDepthNodeParent.first;
    VertexId currNodeId = currDepthNodeParent.second.first;
    VertexId currNodeParentId = currDepthNodeParent.second.second;
    const SimpleNode& currNode = node_map_.at(currNodeId);

    // This shouldn't be necessary, as we don't add masked out vertices to the
    // queue, but check in case
    if (!mask->operator[](currNodeId)) continue;

    // Insert garbage here to test if the key exists, and get a reference to the
    // correct map position for assignment
    auto depthPair = nodeDepths.insert(std::make_pair(currNodeId, -1));

    // Check if the depth was already contained in the map (node already
    // visited)
    if (!depthPair.second) {
      // Double check that recorded depth is indeed less than or equal to
      // proposed depth
      if ((*depthPair.first).second > currNodeDepth) {
        CLOG(ERROR, "pose_graph") << "found a shorter path...";
        throw std::runtime_error("found a shorter path...");
      }
      continue;
    }

    // Mark node as visited (record the depth it was visited at)
    (*depthPair.first).second = currNodeDepth;
    nodeParents[currNodeId] = currNodeParentId;

    // Check if current node is one we are searching for
    auto foundSearchIter =
        std::find(search_ids.begin(), search_ids.end(), currNodeId);
    if (foundSearchIter != search_ids.end()) {
      // Note we only visit each node once, so we do not need to record which
      // one we found...
      numFound++;
    }

    // For each adjacent node
    for (auto adjIter = currNode.getAdjacent().begin();
         adjIter != currNode.getAdjacent().end(); ++adjIter) {
      // Get child id
      VertexId childId = *adjIter;

      // Make edge
      EdgeId currEdge(currNodeId, childId);

      if (!mask->operator[](currEdge) || !mask->operator[](childId)) continue;

      // Get edge weight
      double edgeWeight = weights->operator[](currEdge);

      // Calculate depth to visit node
      double newChildDepth = currNodeDepth + edgeWeight;

      // Check if we have already visited this node
      auto childDepthIter = nodeDepths.find(childId);
      if (childDepthIter != nodeDepths.end()) {
        // Double check that recorded depth is indeed less than or equal to
        // proposed depth
        if (childDepthIter->second > newChildDepth) {
          CLOG(ERROR, "pose_graph") << "found a shorter path...";
          throw std::runtime_error("found a shorter path...");
        }
        continue;
      }

      // Add to queue
      searchQueue.push_back(
          std::make_pair(newChildDepth, std::make_pair(childId, currNodeId)));
    }

    // After adding new adjacent edges, we sort queue to make sure we try
    // shortest depths first
    searchQueue.sort();
  }

  if (numFound < numSearches) {
    std::string err{"Did not find all nodes."};
    CLOG(ERROR, "pose_graph") << err;
    throw std::runtime_error{err};
  }

  // Get unique list of edges
  std::list<EdgeId> edges;
  for (unsigned int i = 0; i < search_ids.size(); i++) {
    VertexId nodeId = search_ids[i];
    backtraceEdgesToRoot(nodeParents, nodeId, &edges);
  }
  edges.sort();
  edges.unique();

  return SimpleGraph(edges);
}

SimpleGraph SimpleGraph::breadthFirstTraversal(VertexId root_id,
                                               double max_depth) const {
  return this->dijkstraTraverseToDepth(root_id, max_depth);
}

SimpleGraph SimpleGraph::breadthFirstTraversal(
    VertexId root_id, double max_depth, const eval::mask::Ptr& mask) const {
  return this->dijkstraTraverseToDepth(
      root_id, max_depth, std::make_shared<eval::weight::ConstEval>(1, 1),
      mask);
}

SimpleGraph SimpleGraph::breadthFirstSearch(VertexId root_id,
                                            VertexId search_id) const {
  return this->dijkstraSearch(root_id, search_id);
}

SimpleGraph SimpleGraph::breadthFirstMultiSearch(
    VertexId root_id, const VertexVec& search_ids) const {
  return this->dijkstraMultiSearch(root_id, search_ids);
}

SimpleGraph SimpleGraph::getMinimalSpanningTree(
    const eval::weight::Ptr& weights, const eval::mask::Ptr& mask) const {
  // Get weighted edges
  std::vector<kruskal::WeightedEdge> weightedEdges;
  weightedEdges.reserve(edges_.size());

  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    // Add weighted edge
    try {
      if (mask->operator[](*it)) {
        weightedEdges.push_back(
            kruskal::WeightedEdge(*it, weights->operator[](*it)));
      }
    } catch (std::exception& e) {
      CLOG(ERROR, "pose_graph")
          << "edge did not exist/could not be computed in weight/mask map";
      throw std::invalid_argument(
          "edge did not exist/could not be computed in weight/mask map");
    }
  }

  // This will store the resultant MST
  SimpleGraph mst;

  // Sort all the edges in non-decreasing order of their weight.
  std::sort(weightedEdges.begin(), weightedEdges.end());

  // Allocate memory for creating V subsets
  // Note the algorithm is already linear in the number of edges
  std::unordered_map<VertexId, kruskal::UnionFindSubset> subsetMap;
  for (NodeMap::const_iterator nodeIter = node_map_.begin();
       nodeIter != node_map_.end(); ++nodeIter) {
    if (mask->operator[](nodeIter->first)) {
      subsetMap[nodeIter->second.getId()].parent = nodeIter->second.getId();
      subsetMap[nodeIter->second.getId()].rank = VertexId(0, 0);
    }
  }

  // Number of edges to be taken is equal to  nodes-1
  unsigned int numOfMstEdges = 0;
  int proposedIndex = 0;
  while (numOfMstEdges < node_map_.size() - 1) {
    // Iterate through the edges (smallest weight first)
    const kruskal::WeightedEdge& proposedEdge = weightedEdges[proposedIndex++];

    // Find the subsets belonging to the adjacent nodes
    VertexId x = kruskal::find(&subsetMap, proposedEdge.edge.id1());
    VertexId y = kruskal::find(&subsetMap, proposedEdge.edge.id2());

    // If including this edge does't cause cycle, include it
    // in result and increment the index of result for next edge
    if (x != y) {
      // Add edge
      mst.addEdge(proposedEdge.edge);
      numOfMstEdges++;

      // Union subsets
      kruskal::unionSubsets(&subsetMap, x, y);
    }
  }

  return mst;
}

void SimpleGraph::print() const {
  std::stringstream ss;
  ss << std::endl;
  ss << "Nodes: ";
  for (NodeMap::const_iterator it = node_map_.begin(); it != node_map_.end();
       ++it)
    ss << it->second.getId() << " ";
  ss << std::endl;

  ss << "Edges: ";
  // Sort for print - cleaner
  std::list<EdgeId> sorted = edges_;
  sorted.sort();
  for (std::list<EdgeId>::const_iterator it = sorted.begin();
       it != sorted.end(); ++it)
    ss << *it << " ";
  CLOG(INFO, "pose_graph") << ss.str();
}

void SimpleGraph::backtraceEdgesToRoot(const BacktraceMap& nodeParents,
                                       VertexId node,
                                       std::list<EdgeId>* edges) {
  VertexId parent = nodeParents.at(node);
  if (parent != VertexId::Invalid()) {
    edges->push_back(EdgeId(parent, node));
    backtraceEdgesToRoot(nodeParents, parent, edges);
  }
}

}  // namespace simple
}  // namespace pose_graph
}  // namespace vtr
