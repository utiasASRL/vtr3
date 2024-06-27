#include <gtest/gtest.h>
#include <random>

#include "vtr_logging/logging_init.hpp"

#include "lgmath.hpp"
#include "vtr_path_planning/mpc/mpc_path_planner.hpp"
#include "vtr_tactic/cache.hpp"

using namespace ::testing;
using namespace vtr;
using namespace vtr::tactic;
using namespace vtr::pose_graph;



void print(const tactic::LocalizationChain& chain) {
  CLOG(INFO, "test") << "trunk sid: " << chain.trunkSequenceId()
                     << ", trunk vid: " << chain.trunkVertexId();
  CLOG(INFO, "test") << "T_branch_trunk: "
                     << chain.T_branch_trunk().vec().transpose();
  CLOG(INFO, "test") << "branch sid: " << chain.branchSequenceId()
                     << ", branch vid: " << chain.branchVertexId();
  CLOG(INFO, "test") << "T_twig_branch: "
                     << chain.T_twig_branch().vec().transpose();
  CLOG(INFO, "test") << "twig vid: " << chain.twigVertexId();
  CLOG(INFO, "test") << "T_petiole_twig: "
                     << chain.T_petiole_twig().vec().transpose();
  CLOG(INFO, "test") << "petiole vid: " << chain.petioleVertexId();
  CLOG(INFO, "test") << "T_leaf_petiole: "
                     << chain.T_leaf_petiole().vec().transpose();
}

EdgeTransform tf_from_global(double x, double y,
                           double theta) {
    auto rotm = lgmath::so3::vec2rot({0, 0, theta});
    Eigen::Vector3d final_pose{x, y, 0};
    return EdgeTransform(rotm, -rotm.transpose() * final_pose);
}

class ChainTest : public Test {
 public:
  ChainTest() : graph_(new RCGraph("temp/testing", false)), chain_(new tactic::LocalizationChain(graph_)) {
  P_tran << 1, 0,
            0, 0,
            0, 0,
            0, 0,
            0, 0,
            0, 1;
  }
  ~ChainTest() override {}

  void createConstantCurve(const EdgeTransform& T_wr0, const EdgeTransform& T_wrf, const Timestamp& t1, const Timestamp& t2, unsigned nodes=2, bool is_teach=true) {
    ASSERT_GE(nodes, 2);
    ASSERT_GT(t2, t1);

    EdgeTransform edge = T_wr0.inverse() * T_wrf;
    Timestamp delta_t = t2 - t2;
      
    auto start_v = graph_->addVertex(t1);
    uint64_t run_id = start_v->id().majorId();
    uint64_t minor_offset = start_v->id().minorId();
    for(unsigned i = 1; i < nodes; ++i) {
      double interp = -1.0 / (nodes - 1);
      auto interp_tf = EdgeTransform(interp * edge.vec(), 0);
      graph_->addVertex(t1 + interp * delta_t);
      interp_tf.setZeroCovariance();
      graph_->addEdge(VertexId(run_id, i - 1 + minor_offset), VertexId(run_id, i + minor_offset), EdgeType::Temporal,
                      is_teach, interp_tf);
    }

  }

  void SetUp() override {
    graph_->addRun();

    createConstantCurve(tf_from_global(0, 0, 0), tf_from_global(20.0, 0.0, 0), 0, 10*1e9, 10);
    createConstantCurve(tf_from_global(20.5, 0, 0), tf_from_global(25.5, 5.0, M_PI_2), 10*1e9, 20*1e9, 10);
    graph_->addEdge(VertexId(0, 9), VertexId(0, 10), EdgeType::Temporal,
                      true, tf_from_global(0.5, 0.0, 0).inverse());


    using PrivEvaluator = eval::mask::privileged::CachedEval<RCGraph>;
    auto eval = std::make_shared<PrivEvaluator>(*graph_);
    auto path = graph_->getSubgraph(0ul, eval);
    VertexId::Vector sequence;
    for (auto it = path->begin(0ul); it != path->end(); ++it)
      sequence.push_back(it->v()->id());
    chain_->setSequence(sequence);
    chain_->expand();
  }

  void TearDown() override {}

 protected:
  size_t num_vertices_ = 20;
  RCGraph::Ptr graph_;
  tactic::LocalizationChain::Ptr chain_;
  uint64_t t_offset_ = 4*60*1e9; 
  Eigen::Matrix<double, 6, 2> P_tran;


};

template<int dim>
Eigen::Matrix<double, dim, 1> normalVector(double mu, double sigma) {
  static std::random_device rd{};
  static std::mt19937 gen{rd()};
 
    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
  std::normal_distribution d{mu, sigma};
  Eigen::Matrix<double, dim, 1> out;
  for(unsigned i = 0; i < dim; ++i) {
    out(i) = d(gen);
  }
  return out;
}

TEST_F(ChainTest, solve_mpc_constant_curve) {
  //Init localization
  uint64_t repeat_run = graph_->addRun();

  EdgeTransform T_init = tf_from_global(0, 0.0, 0.0);
  graph_->addVertex(t_offset_);
  graph_->addEdge(VertexId(repeat_run, 0), VertexId(0, 0), EdgeType::Spatial,
                      false, T_init);


  // initialize the localization chain
  chain_->setPetiole(VertexId(repeat_run, 0));
  auto live_id = chain_->petioleVertexId();
  auto map_id = chain_->trunkVertexId();
  auto map_sid = chain_->trunkSequenceId();
  chain_->updateBranchToTwigTransform(live_id, map_id, map_sid,
                                      T_init.inverse(), true, false);

  // CLOG(DEBUG, "test") << chain_->pose(19).r_ab_inb().transpose().head<2>()
  //             << " " << chain_->pose(19).vec().tail<1>();

  print(*chain_);


  constexpr unsigned NUM_STEPS = 10;


  Eigen::Vector2d last_velo = {0.8, 0.0};

  constexpr double DT_MPC = 0.12;

  MPCConfig config;
  config.VF = 0.8;
  config.DT = 0.1;
  config.K = NUM_STEPS;
  config.vel_max = {1.5, 0.5};
  config.acc_max = {10.0, 10.0};
  config.previous_vel = last_velo;
  config.verbosity = true;
  config.T0 = T_init;
  config.vel_warm_start = {last_velo};

  config.pose_noise_cov = Eigen::Matrix<double, 6, 6>::Identity(); 
  config.pose_noise_cov.diagonal() << 1.0, 1.0, 1e5, 1e5, 1e5, 5e2;

  config.vel_noise_cov = Eigen::Matrix<double, 2, 2>::Identity(); 
  config.vel_noise_cov.diagonal() << 1.0, 100.0;

  config.alpha = Eigen::Matrix<double, 2, 2>::Zero(); 
  // config.alpha.diagonal() << 0.5, 0.5;

  Eigen::Matrix2d alpha = Eigen::Matrix2d::Zero();
  // alpha.diagonal() << 0.5, 0.5;
  const Eigen::Matrix2d one_m_alpha = Eigen::Matrix2d::Identity() - alpha;

  auto mpc_res = SolveMPC(config, chain_);

  try {
    for(unsigned i = 0; i < 20*NUM_STEPS; ++i) {
      auto mpc_res = SolveMPC(config, chain_);
      auto applied_vel = mpc_res.applied_vels.front();
      mpc_res.applied_vels.pop_front();
      EXPECT_TRUE(applied_vel.allFinite());
      auto new_velo = alpha * last_velo + one_m_alpha * applied_vel;

      EdgeTransform delta_TF = EdgeTransform{-config.DT * P_tran * new_velo, 0};
      delta_TF.setZeroCovariance();
      auto new_vertex = graph_->addVertex(t_offset_ + (i + 1) * config.DT * 1e9);
      graph_->addEdge(new_vertex->id() - 1, new_vertex->id(), EdgeType::Temporal,
                        false, delta_TF);

      chain_->updatePetioleToLeafTransform(t_offset_ + (i + 1) * config.DT * 1e9, P_tran * new_velo,
                                        delta_TF, false);

      last_velo = new_velo;

      chain_->setPetiole(new_vertex->id());
      auto live_id = chain_->petioleVertexId();
      auto map_id = chain_->trunkVertexId();
      auto map_sid = chain_->trunkSequenceId();
      chain_->updateBranchToTwigTransform(live_id, map_id, map_sid,
                                        chain_->T_leaf_trunk(), true, false);
      
      config.previous_vel = last_velo + normalVector<2>(0, 0.001);
      config.T0 = lgmath::se3::Transformation(normalVector<6>(0, 0.1))*chain_->T_start_leaf();
      config.vel_warm_start = mpc_res.applied_vels;
      CLOG(DEBUG, "test") << chain_->T_start_leaf().r_ab_inb().transpose().head<2>()
                << " " << chain_->T_start_leaf().vec().tail<1>();
    }
  } catch (std::runtime_error& e){

    uint64_t mpc_rollout_run = graph_->addRun();
    graph_->addVertex(t_offset_);
    graph_->addEdge(VertexId(mpc_rollout_run, 0), VertexId(0, 0), EdgeType::Spatial,
                        false, T_init);

    mpc_res.mpc_poses.insert(mpc_res.mpc_poses.begin(), T_init);
    for(unsigned i = 0; i < NUM_STEPS; ++i) {
      auto new_vertex = graph_->addVertex(t_offset_ + (i + 1) * config.DT * 1e9);      
      graph_->addEdge(new_vertex->id() - 1, new_vertex->id(), EdgeType::Temporal,
                        false, mpc_res.mpc_poses[i+1].inverse()*mpc_res.mpc_poses[i]);
    }


    throw e;
  }

  uint64_t mpc_rollout_run = graph_->addRun();
    graph_->addVertex(t_offset_);
    graph_->addEdge(VertexId(mpc_rollout_run, 0), VertexId(0, 0), EdgeType::Spatial,
                        false, T_init);

    mpc_res.mpc_poses.insert(mpc_res.mpc_poses.begin(), T_init);
    for(unsigned i = 0; i < NUM_STEPS; ++i) {
      auto new_vertex = graph_->addVertex(t_offset_ + (i + 1) * config.DT * 1e9);      
      graph_->addEdge(new_vertex->id() - 1, new_vertex->id(), EdgeType::Temporal,
                        false, mpc_res.mpc_poses[i+1].inverse()*mpc_res.mpc_poses[i]);
    }
  
}


//Variations:
//Motion model (add low pass filter)
//Noise to applied and measured velocity and position
//Sampling frequency 

int main(int argc, char** argv) {
  logging::configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
