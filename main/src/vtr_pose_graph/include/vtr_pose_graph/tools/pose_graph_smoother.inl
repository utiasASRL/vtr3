#include "vtr_pose_graph/path/accumulators.hpp"

namespace vtr::pose_graph {

template <class V, class E>
std::set<EdgeId> GraphSmoother<V, E>::findBranchEdges() const
{
  std::set<EdgeId> branches;

  for (auto it = priv_graph_->beginEdge(); it != priv_graph_->endEdge(); it++) 
  {
    if (it->id().isValid() && it->from().majorId() != it->to().majorId()) {
      branches.insert(it->id());
    }
  }

  return branches;
}

template <class V, class E>
void GraphSmoother<V, E>::runBranchSmoothing() const 
{
  const auto branches = findBranchEdges();
  std::for_each(branches.begin(), branches.end(), [this](const EdgeId eid){this->smoothBranch(eid);});
}

template <class V, class E>
void GraphSmoother<V, E>::smoothBranch(const EdgeId eid) const
{
  const typename E::Ptr branch = graph_->at(eid);
  CLOG(DEBUG, "pose_graph.smoothing") << "From: " << branch->from() << " to " << branch->to();

  const Eigen::Vector<double, 6> branch_vec = branch->T().vec();
  const VertexId branch_id = eid.id2();
  const VertexId trunk_id = eid.id1();
  auto nbs = priv_graph_->neighbors(branch_id);

  if (nbs.size() != 2) {
    CLOG(INFO, "pose_graph.smoothing") << "Can't process a branch without two neighbours";
    return;
  }
  nbs.erase(trunk_id);


  for (const auto &nb : nbs) {
    EdgeId to_eval (branch_id, nb);
    const typename E::Ptr next_branch = graph_->at(to_eval);
    CLOG(DEBUG, "pose_graph.smoothing") << "Comparing branch from: " << next_branch->from() << " to " << next_branch->to();

    
    const Eigen::Vector<double, 6> next_branch_vec = (next_branch->from() == branch_id) ? -next_branch->T().vec() : next_branch->T().vec();
    if (branch_vec.dot(next_branch_vec) < 0) {
      CLOG(DEBUG, "pose_graph.smoothing") << "Found mini direction switch at " << eid;

      auto trunk_nbs = priv_graph_->neighbors(trunk_id);
      trunk_nbs.erase(branch_id);

      const auto const_eval = std::make_shared<eval::weight::ConstEval>(1, 1);

      for (const auto &t_nb : trunk_nbs) {
        auto connected = priv_graph_->dijkstraSearch(t_nb, branch_id);
        const auto T_b_t = eval::ComposeTfAccumulator(connected->beginDfs(t_nb), connected->end(), EdgeTransform(true));
        if (next_branch_vec.dot(T_b_t.vec()) > 0) {
          const auto new_edge = EdgeId(branch_id, t_nb);
          CLOG(DEBUG, "pose_graph.smoothing") << "Found connection between: " << new_edge << " that would be straight"; 

          if (!graph_->contains(new_edge)) {
            graph_->addEdge(branch_id, t_nb, branch->type(), true, T_b_t);
            CLOG(INFO, "pose_graph.smoothing") << "Added new edge" << new_edge; 
            return;
          }
        }
      }
    } 
  }

}



}