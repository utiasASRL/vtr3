/**
 * \file test_p2plane_batch.cpp
 * \author Wenda Zhao, Autonomous Space Robotics Lab (ASRL)
 *
 * The batched point-to-plane cost term must give the same cost and
 * Gauss-Newton system as one WeightedLeastSqCostTerm(p2pError) per point,
 * for a rigid scan (all points at one knot) and with continuous-time
 * interpolation (points at their own times between two knots).
 */

#include <gmock/gmock.h>

#include <random>

#include "steam.hpp"
#include "vtr_lidar/features/p2plane_batch_cost_term.hpp"
#include "vtr_logging/logging_init.hpp"

using namespace ::testing;
using namespace steam;
using namespace steam::se3;
using namespace steam::traj;
using namespace vtr::lidar;

namespace {

struct Setup {
  const_vel::Interface::Ptr trajectory;
  std::vector<StateVarBase::Ptr> state_vars;
  SE3StateVar::Ptr T_s_r_var;
  int64_t t0 = 1'000'000'000, t1 = 1'100'000'000;

  Setup() {
    Eigen::Matrix<double, 6, 1> qc;
    qc << 1.0, 0.1, 0.1, 0.1, 0.1, 1.0;
    trajectory = const_vel::Interface::MakeShared(qc);
    Eigen::Matrix<double, 6, 1> xi0, xi1, w0, w1;
    xi0 << 0.3, -0.2, 0.1, 0.02, -0.01, 0.3;
    xi1 << 0.1, -0.1, 0.12, 0.03, -0.02, 0.35;
    w0 << -1.5, 0.1, 0.0, 0.01, 0.02, 0.4;
    w1 << -1.4, 0.2, 0.05, 0.0, 0.03, 0.5;
    const auto T0 = SE3StateVar::MakeShared(lgmath::se3::Transformation(xi0));
    const auto T1 = SE3StateVar::MakeShared(lgmath::se3::Transformation(xi1));
    const auto v0 = vspace::VSpaceStateVar<6>::MakeShared(w0);
    const auto v1 = vspace::VSpaceStateVar<6>::MakeShared(w1);
    trajectory->add(Time(t0), T0, v0);
    trajectory->add(Time(t1), T1, v1);
    state_vars = {T0, v0, T1, v1};
    Eigen::Matrix<double, 6, 1> xi_sr;
    xi_sr << 0.2, 0.0, 0.5, 0.0, 0.0, 3.14;
    T_s_r_var = SE3StateVar::MakeShared(lgmath::se3::Transformation(xi_sr));
    T_s_r_var->locked() = true;
  }

  Evaluable<lgmath::se3::Transformation>::ConstPtr T_ms(const int64_t t) const {
    return inverse(compose(T_s_r_var, trajectory->getPoseInterpolator(Time(t))));
  }
};

struct Corr {
  int64_t t;
  P2PlaneBatchCostTerm::Point p;
};

std::vector<Corr> randomCorrs(const Setup& s, const int n, const bool rigid) {
  std::mt19937 rng(5);
  std::uniform_real_distribution<double> u(-20.0, 20.0), ut(0.0, 1.0);
  std::vector<Corr> out;
  for (int i = 0; i < n; ++i) {
    Corr c;
    // a few distinct firing times (as the columns of a scan)
    c.t = rigid ? s.t1 : s.t0 + static_cast<int64_t>((1 + i % 7) * 12'000'000);
    c.p.query = Eigen::Vector3d(u(rng), u(rng), 0.1 * u(rng));
    c.p.reference = Eigen::Vector3d(u(rng), u(rng), 0.1 * u(rng));
    Eigen::Vector3d nrm(u(rng), u(rng), u(rng));
    nrm.normalize();
    c.p.W = ut(rng) * nrm * nrm.transpose() + 1e-5 * Eigen::Matrix3d::Identity();
    out.push_back(c);
  }
  return out;
}

void compare(const bool rigid) {
  Setup s;
  const auto corrs = randomCorrs(s, 300, rigid);

  // reference: one weighted least-squares term per point
  OptimizationProblem per_point;
  for (const auto& v : s.state_vars) per_point.addStateVariable(v);
  const auto loss = L2LossFunc::MakeShared();
  for (const auto& c : corrs) {
    const auto noise = StaticNoiseModel<3>::MakeShared(c.p.W, NoiseType::INFORMATION);
    const auto err = p2p::p2pError(s.T_ms(c.t), c.p.reference, c.p.query);
    per_point.addCostTerm(WeightedLeastSqCostTerm<3>::MakeShared(err, noise, loss));
  }

  // batched: three terms, points grouped by time
  OptimizationProblem batch;
  for (const auto& v : s.state_vars) batch.addStateVariable(v);
  const size_t chunk = 100;
  for (size_t start = 0; start < corrs.size(); start += chunk) {
    std::map<int64_t, P2PlaneBatchCostTerm::Group> by_time;
    for (size_t k = start; k < std::min(corrs.size(), start + chunk); ++k) {
      auto& g = by_time[corrs[k].t];
      if (!g.T_ms) g.T_ms = s.T_ms(corrs[k].t);
      g.points.push_back(corrs[k].p);
    }
    std::vector<P2PlaneBatchCostTerm::Group> groups;
    for (auto& kv : by_time) groups.push_back(kv.second);
    batch.addCostTerm(P2PlaneBatchCostTerm::MakeShared(groups));
  }

  EXPECT_NEAR(batch.cost(), per_point.cost(), 1e-9 * per_point.cost());

  Eigen::SparseMatrix<double> A_ref, A_batch;
  Eigen::VectorXd b_ref, b_batch;
  per_point.getStateVector();  // as a solver does before building the system
  batch.getStateVector();
  per_point.buildGaussNewtonTerms(A_ref, b_ref);
  batch.buildGaussNewtonTerms(A_batch, b_batch);
  const Eigen::MatrixXd Ar(A_ref), Ab(A_batch);
  ASSERT_EQ(Ar.rows(), Ab.rows());
  EXPECT_LT((Ar - Ab).norm(), 1e-9 * Ar.norm());
  EXPECT_LT((b_ref - b_batch).norm(), 1e-9 * b_ref.norm());
  // the rigid scan only constrains the last pose
  if (rigid) EXPECT_EQ(Ar.block(0, 0, 12, 12).norm(), 0.0);
}

}  // namespace

TEST(LIDAR, p2plane_batch_matches_per_point_rigid) { compare(true); }

TEST(LIDAR, p2plane_batch_matches_per_point_continuous_time) { compare(false); }

int main(int argc, char** argv) {
  vtr::logging::configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
