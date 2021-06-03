#include <gtest/gtest.h>

#include <random>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/evaluator/accumulators.hpp>
#include <vtr_pose_graph/evaluator/mask_evaluator.hpp>
#include <vtr_pose_graph/evaluator/weight_evaluator.hpp>
#include <vtr_pose_graph/index/graph.hpp>
#include <vtr_pose_graph/relaxation/pose_graph_relaxation.hpp>

#define GRAPH_SIZE 10
#define ANGLE_NOISE -M_PI / 4.f / 6.f
#define LINEAR_NOISE 1.f / 6.f

#define RUN_LENGTH 1000
#define NUM_RUNS 20

// Trace out a spiral
#define PATH_ANG_Z M_PI / 72.f
#define PATH_LIN_X 1.f
#define PATH_LIN_Z 0.16
#define ST_ANG_NOISE -M_PI / 12.f / 6.f
#define ST_LIN_NOISE 0.25 / 6.f

using namespace vtr::pose_graph;

using DoubleRandType =
    decltype(std::bind(std::normal_distribution<double>{-1.f, 1.f},
                       std::mt19937(std::random_device{}())));

class RelaxationTest : public ::testing::Test {
 protected:
  RelaxationTest()
      : graph_(new BasicGraph(0)),
        arnd_(std::bind(std::normal_distribution<double>{0.f, ANGLE_NOISE},
                        std::mt19937(std::random_device{}()))),
        rrnd_(std::bind(std::normal_distribution<double>{0.f, LINEAR_NOISE},
                        std::mt19937(std::random_device{}()))) {}

  void SetUp() override {
    /* Create the following othogonal graph:
     *
     * R0: 000 - 001 - 002 - 003 - ...
     *      |     |     |     |
     *     010 - 011 - 012 - 013 - ...
     *      |     |     |     |
     *     020 - 021 - 022 - 023 - ...
     *      |     |     |     |
     *     ...   ...   ...   ...
     *
     * R1: 100 - 101 - 102 - 103 - ...
     *      |     |     |     |
     *     110 - 111 - 112 - 113 - ...
     *      |     |     |     |
     *     120 - 121 - 122 - 123 - ...
     *      |     |     |     |
     *     ...   ...   ...   ...
     *
     * Where any two vertices ixy and jxy are connected across runs
     */
    Eigen::Vector3d vx(2, 0, 0), vy(0, 3, 0), vz(0, 0, 5);
    lgmath::se3::Transformation Tx(Eigen::Matrix3d::Zero(), vx);
    lgmath::se3::Transformation Ty(Eigen::Matrix3d::Zero(), vy);
    lgmath::se3::Transformation Tz(Eigen::Matrix3d::Zero(), vz);

    // Create a GRAPH_SIZE x GRAPH_SIZE cube, where each vertex is a run
    for (int idx = 0; idx < GRAPH_SIZE; ++idx) {
      (void)graph_->addRun();

      graph_->addVertex();
      for (int idz = 1; idz < GRAPH_SIZE; ++idz) {
        graph_->addVertex();
        auto xEdge =
            graph_->addEdge(VertexId(idx, idz), VertexId(idx, idz - 1));
        xEdge->setTransform(randomTf() * Tx);
      }

      for (int idy = 1; idy < GRAPH_SIZE; ++idy) {
        graph_->addVertex();
        auto yEdge = graph_->addEdge(VertexId(idx, idy * GRAPH_SIZE),
                                     VertexId(idx, (idy - 1) * GRAPH_SIZE));
        yEdge->setTransform(randomTf() * Ty);

        for (int idz = 1; idz < GRAPH_SIZE; ++idz) {
          graph_->addVertex();
          auto yEdge =
              graph_->addEdge(VertexId(idx, idy * GRAPH_SIZE + idz),
                              VertexId(idx, (idy - 1) * GRAPH_SIZE + idz));
          auto xEdge =
              graph_->addEdge(VertexId(idx, idy * GRAPH_SIZE + idz),
                              VertexId(idx, idy * GRAPH_SIZE + idz - 1));

          yEdge->setTransform(randomTf() * Ty);
          xEdge->setTransform(randomTf() * Tx);
        }
      }
    }

    // Cross link the runs
    for (int idx = 1; idx < GRAPH_SIZE; ++idx) {
      for (int idy = 0; idy < GRAPH_SIZE * GRAPH_SIZE; ++idy) {
        auto zEdge =
            graph_->addEdge(VertexId(idx, idy), VertexId(idx - 1, idy));
        zEdge->setTransform(randomTf() * Tz);
      }
    }
  }

  void TearDown() override {}

  ~RelaxationTest() override {}

  lgmath::se3::Transformation randomTf() {
    Eigen::Matrix<double, 6, 1> xi_tmp;
    xi_tmp << rrnd_(), rrnd_(), rrnd_(), arnd_(), arnd_(), arnd_();
    return lgmath::se3::Transformation(xi_tmp);
  }

  BasicGraph::Ptr graph_;
  DoubleRandType arnd_;
  DoubleRandType rrnd_;
};

class STRelaxationTest : public ::testing::Test {
 protected:
  STRelaxationTest(int numRuns = NUM_RUNS, int runLen = RUN_LENGTH)
      : graph_(new BasicGraph(0)),
        numRuns_(numRuns),
        runLen_(runLen),
        arnd_(std::bind(std::normal_distribution<double>{0.f, ST_ANG_NOISE},
                        std::mt19937(std::random_device{}()))),
        rrnd_(std::bind(std::normal_distribution<double>{0.f, ST_LIN_NOISE},
                        std::mt19937(std::random_device{}()))) {}

  void SetUp() override {
    /* Create the following othogonal graph:
     *
     * R0: 00 - 01 - 02 - 03 - ...
     *      |    |    |    |
     * R1: 10 - 11 - 12 - 13 - ...
     *      |    |    |    |
     * R2: 20 - 21 - 22 - 23 - ...
     *      |    |    |    |
     *     ...   ...   ...   ...
     */

    // Nominal edge transformations to make a rectangular prism of ratio 2:3:5
    // (because coprimality)
    Eigen::Vector3d r(PATH_LIN_X, 0, PATH_LIN_Z), phi(0.f, 0.f, PATH_ANG_Z);
    lgmath::se3::Transformation T(lgmath::so3::vec2rot(phi), r);

    (void)graph_->addRun();
    graph_->addVertex();

    for (int idy = 1; idy < runLen_; ++idy) {
      graph_->addVertex();
      auto tmpEdge = graph_->addEdge(VertexId(0, idy), VertexId(0, idy - 1));
      tmpEdge->setTransform(T);
    }

    // Create a GRAPH_SIZE x GRAPH_SIZE cube, where each vertex is a run
    for (int idx = 1; idx < numRuns_; ++idx) {
      (void)graph_->addRun();

      graph_->addVertex();
      for (int idy = 1; idy < runLen_; ++idy) {
        graph_->addVertex();
        auto tmpEdge =
            graph_->addEdge(VertexId(idx, idy), VertexId(idx, idy - 1));
        tmpEdge->setTransform(randomTf() * T);

        tmpEdge = graph_->addEdge(VertexId(idx, idy), VertexId(0, idy));
        tmpEdge->setTransform(randomTf());
      }
    }
  }

  void TearDown() override {}

  ~STRelaxationTest() {}

  lgmath::se3::Transformation randomTf() {
    Eigen::Matrix<double, 6, 1> xi_tmp;
    xi_tmp << rrnd_(), rrnd_(), rrnd_(), arnd_(), arnd_(), arnd_();
    return lgmath::se3::Transformation(xi_tmp);
  }

  BasicGraph::Ptr graph_;
  int numRuns_;
  int runLen_;
  DoubleRandType arnd_;
  DoubleRandType rrnd_;
};

TEST_F(RelaxationTest, CubeRelaxationTest) {
  GraphOptimizationProblem<BasicGraph> relaxer(graph_, VertexId(0, 0));

  // We know the covariance perfectly in this case, because added noise
  // artificially
  Eigen::Matrix<double, 6, 6> cov(Eigen::Matrix<double, 6, 6>::Identity());
  cov.topLeftCorner<3, 3>() *= LINEAR_NOISE * LINEAR_NOISE;
  cov.bottomRightCorner<3, 3>() *= ANGLE_NOISE * ANGLE_NOISE;

  steam::BaseNoiseModel<6>::Ptr covPtr(new steam::StaticNoiseModel<6>(cov));
  steam::L2LossFunc::Ptr sharedLossFunc(new steam::L2LossFunc());

  relaxer.registerComponent(
      PoseGraphRelaxation<BasicGraph>::MakeShared(covPtr, sharedLossFunc));

  std::cout << "Solving..." << std::endl;

  using SolverType = steam::LevMarqGaussNewtonSolver;
  SolverType::Params params;
  params.verbose = true;
  relaxer.optimize<SolverType>(params);
}

TEST_F(STRelaxationTest, STGPRelaxationTest) {
  std::cout << "Building test fixture..." << std::endl;
  GraphOptimizationProblem<BasicGraph> relaxer(graph_, VertexId(0, 0));

  // We know the covariance perfectly in this case, because added noise
  // artificially
  Eigen::Matrix<double, 6, 6> cov(Eigen::Matrix<double, 6, 6>::Identity());
  cov.topLeftCorner<3, 3>() *= ST_LIN_NOISE * ST_LIN_NOISE;
  cov.bottomRightCorner<3, 3>() *= ST_ANG_NOISE * ST_ANG_NOISE;

  steam::BaseNoiseModel<6>::Ptr covPtr(new steam::StaticNoiseModel<6>(cov));
  steam::L2LossFunc::Ptr sharedLossFunc(new steam::L2LossFunc());

  relaxer.registerComponent(
      PoseGraphRelaxation<BasicGraph>::MakeShared(covPtr, sharedLossFunc));

  std::cout << "Solving..." << std::endl;

  using SolverType = steam::LevMarqGaussNewtonSolver;
  SolverType::Params params;
  params.verbose = true;
  relaxer.optimize<SolverType>(params);
}

TEST_F(STRelaxationTest, PoseAccumulator) {
  std::cout << "Building test fixture..." << std::endl;
  const VertexId begin_id(0, 0),
      end_id(GRAPH_SIZE - 1, GRAPH_SIZE * GRAPH_SIZE - 1);
  BasicGraphBase::Ptr chain = graph_->breadthFirstSearch(begin_id, end_id);
  GraphOptimizationProblem<BasicGraphBase> relaxer(chain, begin_id);

  // We know the covariance perfectly in this case, because added noise
  // artificially
  Eigen::Matrix<double, 6, 6> cov(Eigen::Matrix<double, 6, 6>::Identity());
  cov.topLeftCorner<3, 3>() *= ST_LIN_NOISE * ST_LIN_NOISE;
  cov.bottomRightCorner<3, 3>() *= ST_ANG_NOISE * ST_ANG_NOISE;

  steam::BaseNoiseModel<6>::Ptr covPtr(new steam::StaticNoiseModel<6>(cov));
  steam::L2LossFunc::Ptr sharedLossFunc(new steam::L2LossFunc());

  relaxer.registerComponent(
      PoseGraphRelaxation<BasicGraphBase>::MakeShared(covPtr, sharedLossFunc));

  std::cout << "Solving..." << std::endl;

  using SolverType = steam::LevMarqGaussNewtonSolver;
  SolverType::Params params;
  params.maxIterations = 0;
  params.verbose = true;
  relaxer.optimize<SolverType>(params);

  // Because it was reduced to a linear chain, accumulation and relaxation
  // should be the same
  using tf_t = lgmath::se3::Transformation;
  // tf_t T_begin_end = accumulator(chain->begin(begin_id), chain->end(),
  // tf_t(), Eval::ComposeTf<BasicGraphBase>());
  tf_t T_begin_end =
      eval::ComposeTfAccumulator(chain->begin(begin_id), chain->end(), tf_t());
  EXPECT_TRUE(relaxer.at(end_id).vec().isApprox(T_begin_end.inverse().vec()));

  /* For debug, the whole chain incrementally
  for (auto iter = ++chain->begin(begin_id); iter != chain->end();) {
    VertexId to_id = iter->to();
    tf_t T_begin_to = accumulator(chain->begin(begin_id), ++iter, tf_t(),
  Eval::ComposeTf<BasicGraphBase>()); INFO("Id: " << to_id);
    CHECK(relaxer.at(to_id).vec() == T_begin_to.inverse().vec());
  }
  */
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}