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
 * \brief
 * \details
 *
 * \author Autonomous Space Robotics Lab (ASRL)
 */
#include <iostream>
#include <vtr_pose_graph/simple_graph/kruskal_mst_functions.hpp>
#include <vtr_pose_graph/simple_graph/simple_graph.hpp>
#include <vtr_pose_graph/simple_graph/simple_iterator.hpp>

namespace vtr {
namespace pose_graph {
namespace simple {
SimpleGraph::SimpleGraph(const std::list<SimpleEdge>& edges) {
  for (auto it = edges.begin(); it != edges.end(); ++it) this->addEdge(*it);
}

SimpleGraph::SimpleGraph(const std::list<SimpleVertex>& vertices, bool cyclic) {
  for (auto it = vertices.begin(); it != vertices.end(); ++it) {
    auto itn = std::next(it);
    if (itn != vertices.end())
      this->addEdge(*it, *itn);
    else if (cyclic)
      this->addEdge(*it, *vertices.begin());
  }
}

void SimpleGraph::addVertex(const SimpleVertex& vertex) {
  // Insert, but don't overwrite if this vertex already exists
  (void)nodeMap_.emplace(SimpleVertex(vertex), SimpleNode(vertex));
}

void SimpleGraph::addEdge(const SimpleEdge& edge) {
  // Get iterators to id-node pairs, or create them
  auto node1 =
      (nodeMap_.emplace(SimpleVertex(edge.first), SimpleNode(edge.first)))
          .first;
  auto node2 =
      (nodeMap_.emplace(SimpleVertex(edge.second), SimpleNode(edge.second)))
          .first;

  // Check that edge does not exist
  const std::list<SimpleVertex>& adj = node1->second.getAdjacent();
  if (std::find(adj.begin(), adj.end(), edge.second) != adj.end())
    throw std::invalid_argument("Tried to add edge that already exists!");

  // Add adjacency
  node1->second.addAdjacent(node2->first);
  node2->second.addAdjacent(node1->first);

  // Add new edge
  edges_.push_back(SimpleGraph::getEdge(edge.first, edge.second));
}

void SimpleGraph::addEdge(SimpleVertex id1, SimpleVertex id2) {
  this->addEdge(std::make_pair(id1, id2));
}

SimpleGraph::VertexVec SimpleGraph::getNodeIds() const {
  VertexVec result;
  for (NodeMap::const_iterator it = nodeMap_.begin(); it != nodeMap_.end();
       ++it) {
    result.push_back(it->second.getId());
  }
  return result;
}

SimpleGraph::EdgeVec SimpleGraph::getEdges() const {
  EdgeVec result;
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    result.push_back(*it);
  }
  return result;
}

SimpleGraph::OrderedIter SimpleGraph::begin(
    SimpleVertex root, double maxDepth, const eval::Mask::Ptr& mask,
    const eval::Weight::Ptr& weight) const {
  return beginBfs(root, maxDepth, mask, weight);
}

SimpleGraph::OrderedIter SimpleGraph::beginBfs(
    SimpleVertex root, double maxDepth, const eval::Mask::Ptr& mask,
    const eval::Weight::Ptr& weight) const {
  // Handle the case of and empty graph separately
  if (nodeMap_.size() == 0) {
    return SimpleGraphIterator::End(this);
  }

  // If we didn't specify a root, pick one arbitrarily
  if (root == SimpleVertex(-1)) {
    root = nodeMap_.begin()->first;
  }
  return SimpleGraphIterator::BFS(this, root, maxDepth, mask, weight);
}

SimpleGraph::OrderedIter SimpleGraph::beginDfs(
    SimpleVertex root, double maxDepth, const eval::Mask::Ptr& mask,
    const eval::Weight::Ptr& weight) const {
  // Handle the case of and empty graph separately
  if (nodeMap_.size() == 0) {
    return SimpleGraphIterator::End(this);
  }

  // If we didn't specify a root, pick one arbitrarily
  if (root == SimpleVertex(-1)) {
    root = nodeMap_.begin()->first;
  }
  return SimpleGraphIterator::DFS(this, root, maxDepth, mask, weight);
}

SimpleGraph::OrderedIter SimpleGraph::beginDijkstra(
    SimpleVertex root, double maxDepth, const eval::Mask::Ptr& mask,
    const eval::Weight::Ptr& weight) const {
  // Handle the case of and empty graph separately
  if (nodeMap_.size() == 0) {
    return SimpleGraphIterator::End(this);
  }

  // If we didn't specify a root, pick one arbitrarily
  if (root == SimpleVertex(-1)) {
    root = nodeMap_.begin()->first;
  }
  return SimpleGraphIterator::Dijkstra(this, root, maxDepth, mask, weight);
}

SimpleGraph::OrderedIter SimpleGraph::end() const {
  return SimpleGraphIterator::End(this);
}

std::unordered_set<SimpleVertex> SimpleGraph::pathDecomposition(
    ComponentList* paths, ComponentList* cycles) const {
  std::list<SimpleVertex> searchQueue;
  std::unordered_set<SimpleVertex> junctions;
  std::unordered_set<SimpleVertex> exploredSet;

  if (nodeMap_.size() == 0) return junctions;

  // Find a vertex of degree 1 (path termination) or >2 (junction) to start the
  // search at
  for (auto&& it : nodeMap_) {
    if (it.second.getAdjacent().size() != 2) {
      searchQueue.push_back(it.first);
      break;
    }
  }

  // Special case: 2-regular graph.  If the above loop doesn't break, the graph
  // is a single cycle In this case, just break it at some arbitrary point and
  // don't return a junction vertex
  if (searchQueue.empty()) {
    std::list<SimpleVertex> path;
    for (auto it = this->beginDfs(nodeMap_.begin()->first); it != this->end();
         ++it) {
      path.push_back(it->v());
    }

    path.push_back(nodeMap_.begin()->first);
    cycles->push_back(path);
    return junctions;
  }

  // Loop through all junction vertices
  while (!searchQueue.empty()) {
    SimpleVertex root = searchQueue.front();
    searchQueue.pop_front();

    // Only search a junction once
    if (junctions.find(root) != junctions.end()) continue;

    junctions.insert(root);
    exploredSet.insert(root);

    for (auto&& it : nodeMap_.at(root).getAdjacent()) {
      // Don't double explore paths.  We only need to check this once, as we
      // always explore an entire path in one go
      if (exploredSet.find(it) != exploredSet.end()) continue;

      std::list<SimpleVertex> path = {root};
      SimpleVertex branch(it);

      // Expand outward until we hit something that branches/dead-ends
      while (nodeMap_.at(branch).getAdjacent().size() == 2) {
        // The next vertex is the neighbour that isn't in the path yet
        const std::list<SimpleVertex>& neighbours =
            nodeMap_.at(branch).getAdjacent();
        SimpleVertex next(path.back() == neighbours.front()
                              ? neighbours.back()
                              : neighbours.front());

        path.push_back(branch);
        exploredSet.insert(branch);
        branch = next;
      }

      path.push_back(branch);
      if (nodeMap_.at(branch).getAdjacent().size() > 2) {
        searchQueue.push_back(branch);
      } else {
        // The case of deg(v) == 1 isn't a junction, but it's still a special
        // case and we probably want to plot it on a map as it likely represents
        // a point of interest
        junctions.insert(branch);
      }

      if (branch != root) {
        paths->push_back(path);
      } else {
        // TODO: Does this case ever happen?  The case of a graph being a single
        // loop has already been handled...
        cycles->push_back(path);
      }
    }
  }

  return junctions;
}

SimpleGraph SimpleGraph::getSubgraph(const VertexVec& nodes,
                                     const eval::Mask::Ptr& mask) const {
  // Check for nodes
  if (nodes.size() == 0) {
    throw std::invalid_argument("[SimpleGraph][getSubgraph] no nodes.");
  }

  // Collect edges
  std::list<SimpleEdge> subgraphEdges;
  std::unordered_set<SimpleVertex> nodeSet(nodes.begin(), nodes.end());

  // For each node in inputs
  for (auto&& it : nodes) {
    if (!mask->operator[](it)) continue;

    // Check that node is part of graph
    auto nodeIter = nodeMap_.find(it);
    if (nodeIter == nodeMap_.end()) {
      throw std::invalid_argument(
          "[SimpleGraph][getSubgraph] an input node did not exist in graph.");
    }

    // Get node and adjacent node references
    const SimpleNode& node = nodeIter->second;
    const std::list<SimpleVertex>& adj = node.getAdjacent();

    // For each adjacent node
    for (auto&& adjIter : adj) {
      if (!mask->operator[](adjIter) || !mask->operator[](getEdge(it, adjIter)))
        continue;

      // Check if adjacent is in 'nodes'
      auto foundIter = nodeSet.find(adjIter);

      // If both nodes were found, add this edge to our subgraph
      if (foundIter != nodeSet.end())
        subgraphEdges.push_back(SimpleGraph::getEdge(node.getId(), adjIter));
    }
  }

  // Make edges unique
  subgraphEdges.sort();
  subgraphEdges.unique();

  // Make subgraph
  SimpleGraph subgraph(subgraphEdges);

  // If there was only one vertex, add it.
  if (nodes.size() == 1) subgraph.addVertex(nodes[0]);

  // Check for disconnection
  /*
  SimpleGraph bft = subgraph.breadthFirstTraversal(nodes[0], 0.0);
  if (subgraph.numberOfNodes() != bft.numberOfNodes()) {
    std::cout << "disconnected subgraph..." << std::endl;
    subgraph.print();
    std::cout << "breadth first traversal from node[0] found..." << std::endl;
    bft.print();
    throw std::invalid_argument("[SimpleGraph][getSubgraph] subgraph is
  disconnected.");
  }
  */

  return subgraph;
}

SimpleGraph SimpleGraph::getSubgraph(SimpleVertex rootId,
                                     const eval::Mask::Ptr& mask) const {
  return dijkstraTraverseToDepth(rootId, 0.0,
                                 eval::Weight::Const::MakeShared(0), mask);
}

SimpleGraph SimpleGraph::getSubgraph(SimpleVertex rootId, double maxDepth,
                                     const eval::Mask::Ptr& mask) const {
  return dijkstraTraverseToDepth(rootId, maxDepth,
                                 eval::Weight::Const::MakeShared(0), mask);
}

SimpleGraph& SimpleGraph::operator+=(const SimpleGraph& other) {
  for (auto&& it : other.nodeMap_) {
    // Add a new node, or retreive the existing one from $this
    auto node = nodeMap_.emplace(it.first, SimpleNode(it.first)).first;
    auto adj = node->second.getAdjacent();

    // For all neighbours of the node in the other graph...
    for (auto&& neighbour : it.second.getAdjacent()) {
      // If $node was not adjacent to $neighbour in $this, and $neighbour is
      // currently in $this...
      if (std::find(adj.begin(), adj.end(), neighbour) == adj.end() &&
          nodeMap_.find(neighbour) != nodeMap_.end()) {
        // Update the adjacency list of each node, and push the edge onto the
        // edge list
        nodeMap_[neighbour].addAdjacent(it.first);
        node->second.addAdjacent(neighbour);
        edges_.push_back(SimpleGraph::getEdge(it.first, neighbour));
      }
      // If the neighbour was found, then the edge existed in both graphs
      // If $neighbour was not in the node map, then it must be in $other, and
      // this condition will trigger later
    }
  }

  return *this;
}

SimpleGraph SimpleGraph::dijkstraTraverseToDepth(
    SimpleVertex rootId, double maxDepth, const eval::Weight::Ptr& weights,
    const eval::Mask::Ptr& mask) const {
  // Initialized result
  SimpleGraph bft;

  // Check that root exists
  auto rootIter = nodeMap_.find(rootId);
  if (rootIter == nodeMap_.end()) {
    throw std::invalid_argument(
        "[SimpleGraph][dijkstraTraverseToDepth] root does not exist!");
  }

  // Check valid depth input
  if (maxDepth < 0.0) {
    throw std::invalid_argument(
        "[SimpleGraph][dijkstraTraverseToDepth] maxDepth must be greater "
        "than zero, or 0.0 to indicate infinite depth.");
  }

  // Distance to each node (also tracks who has been visited)
  std::unordered_map<SimpleVertex, double> nodeDepths;

  // Parent of each node (nodeId/parentId)
  BacktraceMap nodeParents;

  // Init search queue
  // * Note this is stored in <depth, <nodeId,parentId> > so that we can
  //   sort and process minimum depth first
  using DepthNodeParent =
      std::pair<double, std::pair<SimpleVertex, SimpleVertex> >;
  std::list<DepthNodeParent> searchQueue;
  searchQueue.push_back(std::make_pair(0.0, std::make_pair(rootId, -1)));

  // Until our search queue is empty
  while (!searchQueue.empty()) {
    // Pop front node (next shortest depth from root)
    DepthNodeParent currDepthNodeParent = searchQueue.front();
    searchQueue.pop_front();

    // Get current node depth, id and reference
    double currNodeDepth = currDepthNodeParent.first;
    SimpleVertex currNodeId = currDepthNodeParent.second.first;
    SimpleVertex currNodeParentId = currDepthNodeParent.second.second;
    const SimpleNode& currNode = nodeMap_.at(currNodeId);

    // We can add the edge without further checks, as we can't ever reach the
    // same node twice from the same parent
    if (currNodeParentId != SimpleVertex(-1)) {
      bft.addEdge(SimpleGraph::getEdge(currNodeId, currNodeParentId));
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
        throw std::runtime_error(
            "[SimpleGraph][dijkstraTraverseToDepth] found a shorter path...");
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
      SimpleVertex childId = *adjIter;

      // Make edge
      SimpleEdge currEdge = SimpleGraph::getEdge(currNode.getId(), childId);

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
          throw std::runtime_error(
              "[SimpleGraph][dijkstraTraverseToDepth] found a shorter path...");
        }
        continue;
      }

      // If visiting node is less than max depth, add to queue and add to graph
      if (maxDepth == 0.0 || newChildDepth <= maxDepth) {
        searchQueue.push_back(
            std::make_pair(newChildDepth, std::make_pair(childId, currNodeId)));
      }
    }

    // After adding new adjacent edges, we sort queue to make sure we try
    // shortest depths first
    searchQueue.sort();
  }

  return bft;
}

SimpleGraph SimpleGraph::dijkstraSearch(SimpleVertex rootId,
                                        SimpleVertex searchId,
                                        const eval::Weight::Ptr& weights,
                                        const eval::Mask::Ptr& mask) const {
  VertexVec searchIds;
  searchIds.push_back(searchId);
  return this->dijkstraMultiSearch(rootId, searchIds, weights, mask);
}

SimpleGraph SimpleGraph::dijkstraMultiSearch(
    SimpleVertex rootId, const VertexVec& searchIds,
    const eval::Weight::Ptr& weights, const eval::Mask::Ptr& mask) const {
  // Check that root exists
  NodeMap::const_iterator rootIter = nodeMap_.find(rootId);
  if (rootIter == nodeMap_.end()) {
    throw std::invalid_argument(
        "[SimpleGraph][dijkstraMultiSearch] root does not exist!");
  }

  // Check valid search input
  if (searchIds.size() == 0) {
    throw std::invalid_argument(
        "[SimpleGraph][dijkstraMultiSearch] searchIds is size zero.");
  }

  // Init search size variables
  unsigned long numSearches = searchIds.size();
  unsigned long numFound = 0;

  // Distance to each node (also tracks who has been visited)
  std::unordered_map<SimpleVertex, double> nodeDepths;

  // Parent of each node (nodeId/parentId)
  BacktraceMap nodeParents;

  // Init search queue
  // * Note this is stored in <depth, <nodeId,parentId> > so that we can
  //   sort and process minimum depth first
  using DepthNodeParent =
      std::pair<double, std::pair<SimpleVertex, SimpleVertex> >;
  std::list<DepthNodeParent> searchQueue;
  searchQueue.push_back(std::make_pair(0.0, std::make_pair(rootId, -1)));

  // Until our search queue is empty
  while (!searchQueue.empty() && numFound < numSearches) {
    // Pop front node (next shortest depth from root)
    DepthNodeParent currDepthNodeParent = searchQueue.front();
    searchQueue.pop_front();

    // Get current node depth, id and reference
    double currNodeDepth = currDepthNodeParent.first;
    SimpleVertex currNodeId = currDepthNodeParent.second.first;
    SimpleVertex currNodeParentId = currDepthNodeParent.second.second;
    const SimpleNode& currNode = nodeMap_.at(currNodeId);

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
        throw std::runtime_error(
            "[SimpleGraph][dijkstraMultiSearch] found a shorter path...");
      }
      continue;
    }

    // Mark node as visited (record the depth it was visited at)
    (*depthPair.first).second = currNodeDepth;
    nodeParents[currNodeId] = currNodeParentId;

    // Check if current node is one we are searching for
    auto foundSearchIter =
        std::find(searchIds.begin(), searchIds.end(), currNodeId);
    if (foundSearchIter != searchIds.end()) {
      // Note we only visit each node once, so we do not need to record which
      // one we found...
      numFound++;
    }

    // For each adjacent node
    for (auto adjIter = currNode.getAdjacent().begin();
         adjIter != currNode.getAdjacent().end(); ++adjIter) {
      // Get child id
      SimpleVertex childId = *adjIter;

      // Make edge
      SimpleEdge currEdge = SimpleGraph::getEdge(currNodeId, childId);

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
          throw std::runtime_error(
              "[SimpleGraph][dijkstraMultiSearch] found a shorter path...");
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
  std::list<SimpleEdge> edges;
  for (unsigned int i = 0; i < searchIds.size(); i++) {
    SimpleVertex nodeId = searchIds[i];
    backtraceEdgesToRoot(nodeParents, nodeId, &edges);
  }
  edges.sort();
  edges.unique();

  return SimpleGraph(edges);
}

SimpleGraph SimpleGraph::breadthFirstTraversal(SimpleVertex rootId,
                                               double maxDepth) const {
  return this->dijkstraTraverseToDepth(rootId, maxDepth);
}

SimpleGraph SimpleGraph::breadthFirstTraversal(
    SimpleVertex rootId, double maxDepth, const eval::Mask::Ptr& mask) const {
  return this->dijkstraTraverseToDepth(
      rootId, maxDepth, eval::Weight::Const::MakeShared(0), mask);
}

SimpleGraph SimpleGraph::breadthFirstSearch(SimpleVertex rootId,
                                            SimpleVertex searchId) const {
  return this->dijkstraSearch(rootId, searchId);
}

SimpleGraph SimpleGraph::breadthFirstMultiSearch(
    SimpleVertex rootId, const VertexVec& searchIds) const {
  return this->dijkstraMultiSearch(rootId, searchIds);
}

SimpleGraph SimpleGraph::getMinimalSpanningTree(
    const eval::Weight::Ptr& weights, const eval::Mask::Ptr& mask) const {
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
      throw std::invalid_argument(
          "[SimpleGraph][getMST] edge did not exist/could not be computed in "
          "weight/mask map");
    }
  }

  // This will store the resultant MST
  SimpleGraph mst;

  // Sort all the edges in non-decreasing order of their weight.
  std::sort(weightedEdges.begin(), weightedEdges.end());

  // Allocate memory for creating V subsets
  // Note the algorithm is already linear in the number of edges
  std::unordered_map<SimpleVertex, kruskal::UnionFindSubset> subsetMap;
  for (NodeMap::const_iterator nodeIter = nodeMap_.begin();
       nodeIter != nodeMap_.end(); ++nodeIter) {
    if (mask->operator[](nodeIter->first)) {
      subsetMap[nodeIter->second.getId()].parent = nodeIter->second.getId();
      subsetMap[nodeIter->second.getId()].rank = 0;
    }
  }

  // Number of edges to be taken is equal to  nodes-1
  unsigned int numOfMstEdges = 0;
  int proposedIndex = 0;
  while (numOfMstEdges < nodeMap_.size() - 1) {
    // Iterate through the edges (smallest weight first)
    const kruskal::WeightedEdge& proposedEdge = weightedEdges[proposedIndex++];

    // Find the subsets belonging to the adjacent nodes
    SimpleVertex x = kruskal::find(&subsetMap, proposedEdge.edge.first);
    SimpleVertex y = kruskal::find(&subsetMap, proposedEdge.edge.second);

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
  std::cout << "Nodes: ";
  for (NodeMap::const_iterator it = nodeMap_.begin(); it != nodeMap_.end();
       ++it)
    std::cout << it->second.getId() << " ";
  std::cout << std::endl;

  std::cout << "Edges: ";
  // Sort for print - cleaner
  std::list<SimpleEdge> sorted = edges_;
  sorted.sort();
  for (std::list<SimpleEdge>::const_iterator it = sorted.begin();
       it != sorted.end(); ++it)
    std::cout << "(" << it->first << "," << it->second << ") ";
  std::cout << std::endl;
}

SimpleEdge SimpleGraph::getEdge(SimpleVertex id1, SimpleVertex id2) {
  if (id1 < id2)
    return std::make_pair(id1, id2);
  else if (id2 < id1)
    return std::make_pair(id2, id1);
  else
    throw std::invalid_argument("[SimpleGraph][order] ids were equal");
}

void SimpleGraph::backtraceEdgesToRoot(const BacktraceMap& nodeParents,
                                       SimpleVertex node,
                                       std::list<SimpleEdge>* edges) {
  SimpleVertex parent = nodeParents.at(node);
  if (parent != SimpleVertex(-1)) {
    edges->push_back(SimpleGraph::getEdge(parent, node));
    backtraceEdgesToRoot(nodeParents, parent, edges);
  }
}

}  // namespace simple
}  // namespace pose_graph
}  // namespace vtr
