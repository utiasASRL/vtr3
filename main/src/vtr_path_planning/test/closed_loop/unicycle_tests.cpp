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
using namespace vtr::path_planning;

constexpr unsigned NUM_STEPS = 15;


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



using SE2Pose = std::tuple<double, double, double>;

class ChainTest : public TestWithParam<SE2Pose>  {
 public:
  ChainTest() : graph_(new RCGraph(std::string("temp/") + std::string(UnitTest::GetInstance()->current_test_info()->name()), false)), chain_(new tactic::LocalizationChain(graph_)) {
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
    start_v->SetTerrainType(5);
    for(unsigned i = 1; i < nodes; ++i) {
      double interp = -1.0 / (nodes - 1);
      auto interp_tf = EdgeTransform(interp * edge.vec(), 0);
      auto v = graph_->addVertex(t1 + interp * delta_t);
      v->SetTerrainType(5);

      interp_tf.setZeroCovariance();
      graph_->addEdge(VertexId(run_id, i - 1 + minor_offset), VertexId(run_id, i + minor_offset), EdgeType::Temporal,
                      is_teach, interp_tf);
    }

  }

  void SetUp() override {
    graph_->addRun();

    createConstantCurve(tf_from_global(0, 0, 0), tf_from_global(20.0, 0.0, 0), 0, 10*1e9, 20);
    createConstantCurve(tf_from_global(20.5, 0, 0), tf_from_global(25.5, 5.0, M_PI_2), 10*1e9, 20*1e9, 20);
    graph_->addEdge(VertexId(0, 19), VertexId(0, 20), EdgeType::Temporal,
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


class RealChainTest : public TestWithParam<std::string>  {
 public:
  RealChainTest() : graph_(new RCGraph(GetParam(), true)), chain_(new tactic::LocalizationChain(graph_)) {
    P_tran << 1, 0,
              0, 0,
              0, 0,
              0, 0,
              0, 0,
              0, 1;
  }
  ~RealChainTest() override {}


  void SetUp() override {
    CLOG(DEBUG, "test") << "Starting set up";

    using PrivEvaluator = eval::mask::privileged::CachedEval<RCGraph>;
    using ForwardEvaluator = eval::mask::direction_from_vertex::Eval;
    auto teachEval = std::make_shared<PrivEvaluator>(*graph_);
    auto fwdEval = std::make_shared<ForwardEvaluator>(2ul);
    auto path = graph_->getSubgraph(0ul, teachEval);
    CLOG(DEBUG, "test") << "Got subgraph";

    VertexId::Vector sequence;
    for (auto it = path->begin(2ul, 0, fwdEval); it != path->end(); ++it)
      sequence.push_back(it->v()->id());
    chain_->setSequence(sequence);
    chain_->expand();
  }

  void TearDown() override {}

 protected:
  RCGraph::Ptr graph_;
  tactic::LocalizationChain::Ptr chain_;
  uint64_t t_offset_ = 4*60*1e9; 
  Eigen::Matrix<double, 6, 2> P_tran;

};

template<int dim>
Eigen::Matrix<double, dim, 1> normalVector(double mu, Eigen::Matrix<double, dim, 1> sigmas) {
  static std::random_device rd{};
  static std::mt19937 gen{rd()};
 
    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
  Eigen::Matrix<double, dim, 1> out;
  for(unsigned i = 0; i < dim; ++i) {
    std::normal_distribution d{mu, sigmas(i)};
    out(i) = d(gen);
  }
  return out;
}

TEST_P(ChainTest, NoiseFreeMPC) {
  //Init localization
  uint64_t repeat_run = graph_->addRun();

  SE2Pose init_pose = GetParam();
  auto x_init = std::get<0>(init_pose);
  auto y_init = std::get<1>(init_pose);
  auto theta_init = std::get<2>(init_pose);

  EdgeTransform T_init = tf_from_global(x_init, y_init, theta_init);
  graph_->addVertex(t_offset_);
  graph_->addEdge(VertexId(repeat_run, 0), VertexId(0, 0), EdgeType::Spatial,
                      false, T_init);

  ASSERT_GT(chain_->p(chain_->size() - 1), 0);

  // initialize the localization chain
  chain_->setPetiole(VertexId(repeat_run, 0));
  auto live_id = chain_->petioleVertexId();
  auto map_id = chain_->trunkVertexId();
  auto map_sid = chain_->trunkSequenceId();
  chain_->updateBranchToTwigTransform(live_id, map_id, map_sid,
                                      T_init.inverse(), true, false);

  print(*chain_);


  CasadiUnicycleMPC solver;
  CasadiUnicycleMPC::Config config;
  config.VF = 1.0;
  config.vel_max = {1.5, 0.5};

  Eigen::Vector2d last_velo = {0.0, 0.0};
  config.previous_vel = {0.0, 0.0};
  config.T0 = {x_init, y_init, theta_init};

  double state_p = findRobotP(chain_->T_start_leaf(), chain_);

  for(int i = 1; i < config.N+1; i++){
    config.reference_poses.push_back({i*config.VF*config.DT, 0, 0});
  }

  Eigen::Matrix2d alpha = Eigen::Matrix2d::Zero();
  // alpha.diagonal() << 0, config.alpha;
  const Eigen::Matrix2d one_m_alpha = Eigen::Matrix2d::Identity() - alpha;

  auto mpc_res = solver.solve(config);
  CLOG(DEBUG, "test") << "Shape: (" << mpc_res["pose"].rows() << ", " << mpc_res["pose"].columns() << ")";

  std::vector<EdgeTransform> mpc_poses;
  for(int i = 0; i < mpc_res["pose"].columns(); i++) {
    const auto& pose_i = mpc_res["pose"](casadi::Slice(), i).get_elements();
    mpc_poses.push_back(tf_from_global(pose_i[0], pose_i[1], pose_i[2]));
  }

  for(unsigned i = 0; i < 20*NUM_STEPS; ++i) {
      auto mpc_res = solver.solve(config);
      const auto& mpc_vel = mpc_res["vel"](casadi::Slice(), 0).get_elements();

      Eigen::Vector2d applied_vel;
      applied_vel << mpc_vel[0], mpc_vel[1];

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

      state_p = findRobotP(chain_->T_start_leaf(), chain_);     
      std::vector<double> p_rollout;
      for(int j = 1; j < config.N+1; j++){
        p_rollout.push_back(state_p + j*config.VF*config.DT);
      }

      config.reference_poses.clear();
      auto referenceInfo = generateHomotopyReference(p_rollout, chain_);
      for(const auto& Tf : referenceInfo.poses) {
        config.reference_poses.push_back(tf_to_global(Tf));
        CLOG(DEBUG, "test") << "Target " << tf_to_global(Tf);
      }

      config.up_barrier_q = referenceInfo.barrier_q_max;
      config.low_barrier_q = referenceInfo.barrier_q_min;

      config.previous_vel = {last_velo(0, 0), last_velo(1, 0)};
      config.T0 = tf_to_global(chain_->T_start_leaf());
    }

  uint64_t mpc_rollout_run = graph_->addRun();
  graph_->addVertex(t_offset_);
  graph_->addEdge(VertexId(mpc_rollout_run, 0), VertexId(0, 0), EdgeType::Spatial,
                      false, T_init);

  for(unsigned i = 0; i < config.N; ++i) {
    auto new_vertex = graph_->addVertex(t_offset_ + (i + 1) * config.DT * 1e9);      
    graph_->addEdge(new_vertex->id() - 1, new_vertex->id(), EdgeType::Temporal,
                      false, mpc_poses[i+1].inverse()*mpc_poses[i]);
  }
  
}


TEST_P(ChainTest, NoisyMPC) {

  //Init localization
  uint64_t repeat_run = graph_->addRun();

  SE2Pose init_pose = GetParam();
  auto x_init = std::get<0>(init_pose);
  auto y_init = std::get<1>(init_pose);
  auto theta_init = std::get<2>(init_pose);

  EdgeTransform T_init = tf_from_global(x_init, y_init, theta_init);
  graph_->addVertex(t_offset_);
  graph_->addEdge(VertexId(repeat_run, 0), VertexId(0, 0), EdgeType::Spatial,
                      false, T_init);

  ASSERT_GT(chain_->p(chain_->size() - 1), 0);

  // initialize the localization chain
  chain_->setPetiole(VertexId(repeat_run, 0));
  auto live_id = chain_->petioleVertexId();
  auto map_id = chain_->trunkVertexId();
  auto map_sid = chain_->trunkSequenceId();
  chain_->updateBranchToTwigTransform(live_id, map_id, map_sid,
                                      T_init.inverse(), true, false);

  print(*chain_);


  CasadiUnicycleMPC solver;
  CasadiUnicycleMPC::Config config;
  config.VF = 1.0;
  config.vel_max = {1.5, 0.5};

  Eigen::Vector2d last_velo = {0.0, 0.0};
  config.previous_vel = {0.0, 0.0};
  config.T0 = {x_init, y_init, theta_init};

  double state_p = findRobotP(chain_->T_start_leaf(), chain_);

  for(int i = 1; i < config.N+1; i++){
    config.reference_poses.push_back({i*config.VF*config.DT, 0, 0});
  }

  Eigen::Matrix2d alpha = Eigen::Matrix2d::Zero();
  // alpha.diagonal() << 0, config.alpha;
  const Eigen::Matrix2d one_m_alpha = Eigen::Matrix2d::Identity() - alpha;

  auto mpc_res = solver.solve(config);
  CLOG(DEBUG, "test") << "Shape: (" << mpc_res["pose"].rows() << ", " << mpc_res["pose"].columns() << ")";

  std::vector<EdgeTransform> mpc_poses;
  for(int i = 0; i < mpc_res["pose"].columns(); i++) {
    const auto& pose_i = mpc_res["pose"](casadi::Slice(), i).get_elements();
    mpc_poses.push_back(tf_from_global(pose_i[0], pose_i[1], pose_i[2]));
  }

  for(unsigned i = 0; i < 20*NUM_STEPS; ++i) {
      auto mpc_res = solver.solve(config);
      const auto& mpc_vel = mpc_res["vel"](casadi::Slice(), 0).get_elements();

      Eigen::Vector2d applied_vel;
      applied_vel << mpc_vel[0], mpc_vel[1];

      EXPECT_TRUE(applied_vel.allFinite());
      auto new_velo = alpha * last_velo + one_m_alpha * applied_vel;

      EdgeTransform delta_TF = EdgeTransform{-config.DT * P_tran * new_velo + normalVector<6>(0, {0.1, 0.1, 0, 0.001, 0.001, 0.01}), 0};
      delta_TF.setZeroCovariance();
      auto new_vertex = graph_->addVertex(t_offset_ + (i + 1) * config.DT * 1e9);
      graph_->addEdge(new_vertex->id() - 1, new_vertex->id(), EdgeType::Temporal,
                        false, delta_TF);

      chain_->updatePetioleToLeafTransform(t_offset_ + (i + 1) * config.DT * 1e9, P_tran * new_velo,
                                        delta_TF, false);

      last_velo = new_velo + normalVector<2>(0, {0.1, 0.1});

      chain_->setPetiole(new_vertex->id());
      auto live_id = chain_->petioleVertexId();
      auto map_id = chain_->trunkVertexId();
      auto map_sid = chain_->trunkSequenceId();
      chain_->updateBranchToTwigTransform(live_id, map_id, map_sid,
                                        chain_->T_leaf_trunk(), true, false);

      state_p = findRobotP(chain_->T_start_leaf(), chain_);     
      std::vector<double> p_rollout;
      for(int j = 1; j < config.N+1; j++){
        p_rollout.push_back(state_p + j*config.VF*config.DT);
      }

      config.reference_poses.clear();
      auto referenceInfo = generateHomotopyReference(p_rollout, chain_);
      for(const auto& Tf : referenceInfo.poses) {
        config.reference_poses.push_back(tf_to_global(Tf));
        CLOG(DEBUG, "test") << "Target " << tf_to_global(Tf);
      }

      config.up_barrier_q = referenceInfo.barrier_q_max;
      config.low_barrier_q = referenceInfo.barrier_q_min;

      config.previous_vel = {last_velo(0, 0), last_velo(1, 0)};
      config.T0 = tf_to_global(chain_->T_start_leaf());
    }

  uint64_t mpc_rollout_run = graph_->addRun();
  graph_->addVertex(t_offset_);
  graph_->addEdge(VertexId(mpc_rollout_run, 0), VertexId(0, 0), EdgeType::Spatial,
                      false, T_init);

  for(unsigned i = 0; i < config.N; ++i) {
    auto new_vertex = graph_->addVertex(t_offset_ + (i + 1) * config.DT * 1e9);      
    graph_->addEdge(new_vertex->id() - 1, new_vertex->id(), EdgeType::Temporal,
                      false, mpc_poses[i+1].inverse()*mpc_poses[i]);
  }
  
}


// INSTANTIATE_TEST_SUITE_P(MPCSet, ChainTest, Values(std::make_tuple(0.0, 0.0, 0.0), std::make_tuple(0.0, 0.5, 0.0), std::make_tuple(0.0, -0.5, 0.0)));



TEST_P(RealChainTest, NoiseFreeRealMPC) {
  //Init localization
  uint64_t repeat_run = graph_->addRun();

  double x_init = 0;
  double y_init = -0.2;
  double theta_init = 0;

  EdgeTransform T_init = tf_from_global(x_init, y_init, theta_init);
  graph_->addVertex(t_offset_);
  graph_->addEdge(VertexId(repeat_run, 0), VertexId(0, 2), EdgeType::Spatial,
                      false, T_init);

  ASSERT_GT(chain_->p(chain_->size() - 1), 0);

  // initialize the localization chain
  chain_->setPetiole(VertexId(repeat_run, 0));
  auto live_id = chain_->petioleVertexId();
  auto map_id = chain_->trunkVertexId();
  auto map_sid = chain_->trunkSequenceId();
  chain_->updateBranchToTwigTransform(live_id, map_id, map_sid,
                                      T_init.inverse(), true, false);


  CasadiUnicycleMPC solver;
  CasadiUnicycleMPC::Config config;
  config.VF = 1.0;
  config.vel_max = {1.5, 0.5};

  Eigen::Vector2d last_velo = {0.0, 0.0};
  config.previous_vel = {0.0, 0.0};
  config.T0 = {x_init, y_init, theta_init};

  std::vector<tactic::EdgeTransform> target_poses_0;

  double state_p = findRobotP(chain_->T_start_leaf(), chain_);     
  std::vector<double> p_rollout;
  for(int j = 1; j < config.N+1; j++){
    p_rollout.push_back(state_p + j*config.VF*config.DT);
  }

  config.reference_poses.clear();
  auto referenceInfo = generateHomotopyReference(p_rollout, chain_);
  target_poses_0.push_back(referenceInfo.poses.front());
  for(const auto& Tf : referenceInfo.poses) {
    config.reference_poses.push_back(tf_to_global(chain_->T_start_trunk().inverse() * Tf));
    CLOG(DEBUG, "test") << "Target " << tf_to_global(Tf);
  }
  
  config.up_barrier_q = referenceInfo.barrier_q_max;
  config.low_barrier_q = referenceInfo.barrier_q_min;

  Eigen::Matrix2d alpha = Eigen::Matrix2d::Zero();
  alpha.diagonal() << 0, config.alpha;
  const Eigen::Matrix2d one_m_alpha = Eigen::Matrix2d::Identity() - alpha;

  auto mpc_res = solver.solve(config);
  CLOG(DEBUG, "test") << "Shape: (" << mpc_res["pose"].rows() << ", " << mpc_res["pose"].columns() << ")";

  std::vector<EdgeTransform> mpc_poses;
  for(int i = 0; i < mpc_res["pose"].columns(); i++) {
    const auto& pose_i = mpc_res["pose"](casadi::Slice(), i).get_elements();
    mpc_poses.push_back(tf_from_global(pose_i[0], pose_i[1], pose_i[2]));
  }



  for(unsigned i = 0; i < 250 / config.DT; ++i) {
      auto mpc_res = solver.solve(config);
      const auto& mpc_vel = mpc_res["vel"](casadi::Slice(), 0).get_elements();

      Eigen::Vector2d applied_vel;
      applied_vel << mpc_vel[0], mpc_vel[1];

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

      state_p = findRobotP(chain_->T_start_leaf(), chain_);     
      std::vector<double> p_rollout;
      for(int j = 1; j < config.N+1; j++){
        p_rollout.push_back(state_p + j*config.VF*config.DT);
      }

      config.reference_poses.clear();
      auto referenceInfo = generateHomotopyReference(p_rollout, chain_);
      target_poses_0.push_back(referenceInfo.poses.front());
      for(const auto& Tf : referenceInfo.poses) {
        config.reference_poses.push_back(tf_to_global(chain_->T_start_trunk().inverse() * Tf));
        CLOG(DEBUG, "test") << "Target " << config.reference_poses.back();
      }

      config.up_barrier_q = referenceInfo.barrier_q_max;
      config.low_barrier_q = referenceInfo.barrier_q_min;

      config.previous_vel = {last_velo(0, 0), last_velo(1, 0)};
      config.T0 = tf_to_global(chain_->T_leaf_trunk().inverse());

      if (map_sid == chain_->size() - 1)
        break;
    }

  uint64_t mpc_rollout_run = graph_->addRun();
  graph_->addVertex(t_offset_);
  graph_->addEdge(VertexId(mpc_rollout_run, 0), VertexId(0, 2), EdgeType::Spatial,
                      false, T_init);

  for(unsigned i = 0; i < config.N; ++i) {
    auto new_vertex = graph_->addVertex(t_offset_ + (i + 1) * config.DT * 1e9);      
    graph_->addEdge(new_vertex->id() - 1, new_vertex->id(), EdgeType::Temporal,
                      false, mpc_poses[i+1].inverse()*mpc_poses[i]);
  }


  uint64_t target_pose_run = graph_->addRun();

  auto new_vertex = graph_->addVertex(t_offset_);
  auto start_vertex = graph_->addEdge(new_vertex->id(), VertexId(0, 2), EdgeType::Spatial,
                      false, target_poses_0.front());
  for(size_t i = 1; i < target_poses_0.size(); i++) {
    auto new_vertex = graph_->addVertex(t_offset_ + i * config.DT * 1e9);
          
    graph_->addEdge(new_vertex->id() - 1, new_vertex->id(), EdgeType::Temporal,
                      false, target_poses_0[i].inverse()*target_poses_0[i-1]);
  }

  
}

INSTANTIATE_TEST_SUITE_P(RealMPC, RealChainTest, Values("temp/dome4eva"));


//Variations:
//Motion model (add low pass filter)
//Noise to applied and measured velocity and position
//Sampling frequency 

int main(int argc, char** argv) {
  logging::configureLogging("", true);
  InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
