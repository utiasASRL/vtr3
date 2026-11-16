/**
 * \file p2plane_batch_cost_term.hpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * Batched lidar point-to-plane cost term for STEAM.
 *
 * Computes the same cost and Gauss-Newton terms as one
 *   WeightedLeastSqCostTerm<3>(p2pError(T_ms(t_j), ref_j, qry_j), W_j, L2)
 * per lidar point, but for a whole set of points at once:
 *
 *   e_j = ref_j - T_ms(t_j) qry_j,      cost = sum_j 0.5 e_j^T W_j e_j
 *
 * The points are grouped by their pose T_ms(t) (one group for a rigid scan,
 * one group per firing time with continuous-time estimation). For each group,
 * the pose is evaluated once and the point terms are summed in the pose
 * perturbation space,
 *
 *   H_g = sum_j J_j^T W_j J_j,   g_g = sum_j J_j^T W_j e_j,
 *   J_j = de_j / d(pose perturbation) = -(T q_j)^odot   (as P2PErrorEvaluator)
 *
 * and then chained once to the state variables through the pose evaluator,
 *   A += Jx^T H_g Jx,   b += -Jx^T g_g,
 * which is exact (the per-point Jacobian factorizes as J_j Jx). This removes
 * the per-point evaluator graphs and the per-point locking of the
 * per-point cost terms. STEAM parallelizes over cost terms, so the
 * correspondences are split into many small terms (see
 * OdometryLIVModule: STEAM assigns cost terms to threads in contiguous blocks).
 */
#pragma once

#include <map>
#include <vector>

#include <Eigen/Core>

#include "lgmath.hpp"
#include "steam.hpp"

namespace vtr {
namespace lidar {

class P2PlaneBatchCostTerm : public steam::BaseCostTerm {
 public:
  using Ptr = std::shared_ptr<P2PlaneBatchCostTerm>;
  using ConstPtr = std::shared_ptr<const P2PlaneBatchCostTerm>;
  using PoseEvaluable = steam::Evaluable<lgmath::se3::Transformation>;

  /// One lidar correspondence.
  struct Point {
    Eigen::Vector3d query;      ///< measured point (sensor frame)
    Eigen::Vector3d reference;  ///< map point (map frame)
    Eigen::Matrix3d W;          ///< information matrix
  };

  /// Points that share one pose T_ms (map <- sensor).
  struct Group {
    PoseEvaluable::ConstPtr T_ms;
    std::vector<Point> points;
  };

  static Ptr MakeShared(std::vector<Group> groups, const bool rm_ori = false) {
    return std::make_shared<P2PlaneBatchCostTerm>(std::move(groups), rm_ori);
  }

  P2PlaneBatchCostTerm(std::vector<Group> groups, const bool rm_ori = false)
      : groups_(std::move(groups)), rm_ori_(rm_ori) {
    name_ = "P2PlaneBatchCostTerm";
  }

  size_t numPoints() const {
    size_t n = 0;
    for (const auto& g : groups_) n += g.points.size();
    return n;
  }

  /** \brief sum_j 0.5 e_j^T W_j e_j (L2 loss, as the per-point terms) */
  double cost() const override {
    double cost = 0.0;
    for (const auto& g : groups_) {
      const Eigen::Matrix4d T = g.T_ms->value().matrix();
      const Eigen::Matrix3d C = T.block<3, 3>(0, 0);
      const Eigen::Vector3d r = T.block<3, 1>(0, 3);
      for (const auto& p : g.points) {
        const Eigen::Vector3d e = p.reference - (C * p.query + r);
        cost += 0.5 * e.dot(p.W * e);
      }
    }
    return cost;
  }

  void getRelatedVarKeys(KeySet& keys) const override {
    for (const auto& g : groups_) g.T_ms->getRelatedVarKeys(keys);
  }

  void buildGaussNewtonTerms(const steam::StateVector& state_vec,
                             steam::BlockSparseMatrix* approximate_hessian,
                             steam::BlockVector* gradient_vector) const override {
    // contributions of all groups, summed per state block (pair)
    std::map<unsigned int, Eigen::MatrixXd> b_blocks;
    std::map<std::pair<unsigned int, unsigned int>, Eigen::MatrixXd> A_blocks;

    for (const auto& g : groups_) {
      if (!g.T_ms->active() || g.points.empty()) continue;

      // pose once per group
      const auto node = g.T_ms->forward();
      const Eigen::Matrix4d T = node->value().matrix();
      const Eigen::Matrix3d C = T.block<3, 3>(0, 0);
      const Eigen::Vector3d r = T.block<3, 1>(0, 3);

      // sum of the point terms in the pose perturbation space
      Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Matrix<double, 6, 1> grad = Eigen::Matrix<double, 6, 1>::Zero();
      Eigen::Matrix<double, 3, 6> J;
      J.leftCols<3>() = -Eigen::Matrix3d::Identity();
      for (const auto& p : g.points) {
        const Eigen::Vector3d Tq = C * p.query + r;
        const Eigen::Vector3d e = p.reference - Tq;
        // J = -[I, -Tq^] (= -point2fs(Tq), as P2PErrorEvaluator)
        J.rightCols<3>() = lgmath::so3::hat(Tq);
        if (rm_ori_) J.col(5).setZero();
        const Eigen::Matrix<double, 6, 3> JtW = J.transpose() * p.W;
        H.noalias() += JtW * J;
        grad.noalias() += JtW * e;
      }

      // Jacobians of the pose perturbation w.r.t. the state variables
      steam::Jacobians jacs;
      g.T_ms->backward(Eigen::Matrix<double, 6, 6>::Identity(), node, jacs);
      const auto& jac_map = jacs.get();
      std::vector<std::pair<unsigned int, const Eigen::MatrixXd*>> blocks;
      blocks.reserve(jac_map.size());
      for (const auto& kv : jac_map)
        blocks.emplace_back(state_vec.getStateBlockIndex(kv.first), &kv.second);

      for (size_t i = 0; i < blocks.size(); ++i) {
        const auto& [idx1, Jx1] = blocks[i];
        const Eigen::MatrixXd new_b = -(Jx1->transpose() * grad);
        auto it_b = b_blocks.find(idx1);
        if (it_b == b_blocks.end()) b_blocks.emplace(idx1, new_b);
        else it_b->second += new_b;

        for (size_t j = 0; j < blocks.size(); ++j) {
          const auto& [idx2, Jx2] = blocks[j];
          if (idx1 > idx2) continue;  // upper half only, as WeightedLeastSq
          const Eigen::MatrixXd new_A = Jx1->transpose() * H * (*Jx2);
          auto key = std::make_pair(idx1, idx2);
          auto it_A = A_blocks.find(key);
          if (it_A == A_blocks.end()) A_blocks.emplace(key, new_A);
          else it_A->second += new_A;
        }
      }
    }

    // write once per block (thread critical, as WeightedLeastSqCostTerm)
    for (const auto& kv : b_blocks) {
#pragma omp critical(b_update)
      { gradient_vector->mapAt(kv.first) += kv.second; }
    }
    for (const auto& kv : A_blocks) {
      steam::BlockSparseMatrix::BlockRowEntry& entry =
          approximate_hessian->rowEntryAt(kv.first.first, kv.first.second, true);
      omp_set_lock(&entry.lock);
      entry.data += kv.second;
      omp_unset_lock(&entry.lock);
    }
  }

 private:
  std::vector<Group> groups_;
  bool rm_ori_ = false;
};

}  // namespace lidar
}  // namespace vtr
