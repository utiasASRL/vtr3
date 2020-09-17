#include <gtest/gtest.h>

#include <filesystem>
#include <iostream>
#include <random>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/evaluator/composite.hpp>
#include <vtr_pose_graph/evaluator/mask_evaluator.hpp>
#include <vtr_pose_graph/evaluator/time.hpp>
#include <vtr_pose_graph/evaluator/weight_evaluator.hpp>
#include <vtr_pose_graph/index/composite_graph.hpp>
#include <vtr_pose_graph/index/graph.hpp>
#include <vtr_pose_graph/index/rc_graph/rc_graph.hpp>

namespace fs = std::filesystem;
using namespace vtr::pose_graph;

using SimpleVertex = uint64_t;
using SimpleEdge = std::pair<uint64_t, uint64_t>;

using IntRandType =
    decltype(std::bind(std::uniform_int_distribution<int64_t>{0, 1000},
                       std::mt19937(std::random_device{}())));
using DoubleRandType =
    decltype(std::bind(std::uniform_real_distribution<double>{0.f, 100.f},
                       std::mt19937(std::random_device{}())));

class CompositeTest : public ::testing::Test {
 public:
  CompositeTest()
      : graph_(
            new RCGraph(fs::temp_directory_path() / "vtr_pose_graph_test", 0)),
        irnd_(std::bind(std::uniform_int_distribution<int64_t>{0, 1000},
                        std::mt19937(std::random_device{}()))),
        drnd_(std::bind(std::uniform_real_distribution<double>{0.f, 100.f},
                        std::mt19937(std::random_device{}()))),
        robot_id_(0) {}

  ~CompositeTest() override {}

  void SetUp() override {
#if 0
    /* Create the following graph
     * R0: 0 --- 1 --- 2
     *       \
     *        \
     *         \
     * R1: 0 --- 1 --- 2
     *     |           |
     * R2: 0 --- 1 --- 2
     *           |
     * R3: 0 --- 1 --- 2
     *                 |
     * R4: 0 --- 1 --- 2
     */

    using namespace vtr::common;
    auto stamp = timing::clock::now();

    // add a graph with 5 runs and 3 vertices per run.
    for (int idx = 0; idx < 5; ++idx) {
      // Create the robochunk directories

      (void)graph_->addRun(robot_id_);

      graph_->addVertex(timing::toRobochunk(stamp - timing::hours(idx)));
      graph_->addVertex(
          timing::toRobochunk(stamp - timing::hours(idx) + timing::seconds(5)));
      graph_->addVertex(timing::toRobochunk(stamp - timing::hours(idx) +
                                            timing::seconds(10)));
      graph_->addEdge(VertexId(idx, 0), VertexId(idx, 1));
      graph_->addEdge(VertexId(idx, 1), VertexId(idx, 2));
    }

    graph_->addEdge(VertexId(1, 1), VertexId(0, 0), Spatial);
    graph_->addEdge(VertexId(2, 0), VertexId(1, 0), Spatial);
    graph_->addEdge(VertexId(2, 2), VertexId(1, 2), Spatial);
    graph_->addEdge(VertexId(3, 1), VertexId(2, 1), Spatial);
    graph_->addEdge(VertexId(4, 2), VertexId(3, 2), Spatial);

    // set the edge's transform to something special;
    auto edge_map = graph_->edges();
    for (auto itr = edge_map->begin(); itr != edge_map->end(); ++itr) {
      Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
      transform(0, 3) = itr->second->id().majorId();
      transform(1, 3) = itr->second->id().minorId();
      transform(2, 3) = itr->second->id().type();
      itr->second->setTransform(lgmath::se3::Transformation(transform));
    }

    cgraph_.reset(new CompositeGraph<RCGraph>(graph_));
#endif
  }

  void TearDown() override {}

  SimpleVertex rndSimpleVertex() { return SimpleVertex(irnd_()); }

  SimpleEdge rndSimpleEdge() { return SimpleEdge(irnd_(), irnd_()); }

  VertexId rndVertexId() { return VertexId(irnd_(), irnd_()); }

  EdgeId rndEdgeId(const EdgeId::Type& type) {
    if (type == EdgeId::Type::Temporal) {
      auto run_id = irnd_();
      return EdgeId(SimpleVertex(VertexId(run_id, irnd_())),
                    SimpleVertex(VertexId(run_id, irnd_())), type);
    } else {
      return EdgeId(SimpleVertex(rndVertexId()), SimpleVertex(rndVertexId()),
                    type);
    }
  }

 public:
  RCGraph::Ptr graph_;
  typename CompositeGraph<RCGraph>::Ptr cgraph_;
  IntRandType irnd_;
  DoubleRandType drnd_;
  int robot_id_;
};

#if 0
TEST_CASE_METHOD(CompositeTest, "Weight Evaluator", "[weight]") {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100},
                        std::mt19937(std::random_device{}()));

  SECTION("Constant", "[const]") {
    double eweight = drnd(), vweight = drnd();
    auto eval1 = Eval::Weight::Const::MakeShared(eweight, vweight);
    auto eval2 = Eval::Composite<Eval::Weight::Const>::MakeDirect(
        cgraph_.get(), eweight, vweight);

    CHECK_NOTHROW(eval1->setGraph(graph_.get()));

    auto res1 = graph_->dijkstraSearch(VertexId(0, 2), VertexId(4, 0),
    eval1); auto res2 = cgraph_->dijkstraSearch(VertexId(0, 2), VertexId(4,
    0), eval2)
                    ->flatten();

    auto it1 = res1->begin(VertexId(0, 2));
    auto it2 = res2->begin(VertexId(0, 2));

    while (it1 != res1->end() && it2 != res2->end()) {
      CHECK((it2->v()->id() == it1->v()->id()));
      ++it1;
      ++it2;
    }

    CHECK((it1 == res1->end() && it2 == res2->end()));
  }

  SECTION("Time", "[time]") {
    using namespace asrl::common;

    auto evalTime = timing::clock::now() - timing::minutes(30);
    auto eval1 = Eval::Weight::TimeDelta<RCGraph>::Direct::MakeShared(
        Eval::Weight::TimeDeltaConfig(evalTime));
    auto eval2 =
        Eval::Composite<Eval::Weight::TimeDelta<RCGraph>::Direct>::MakeDirect(
            cgraph_.get());

    CHECK_NOTHROW(eval1->setGraph(graph_.get()));

    auto res1 = graph_->dijkstraSearch(VertexId(0, 2), VertexId(4, 0),
    eval1); auto res2 = cgraph_->dijkstraSearch(VertexId(0, 2), VertexId(4,
    0), eval2)
                    ->flatten();

    auto it1 = res1->begin(VertexId(0, 2));
    auto it2 = res2->begin(VertexId(0, 2));

    while (it1 != res1->end() && it2 != res2->end()) {
      LOG(INFO) << it1->v()->id() << ", " << it2->v()->id();
      CHECK((it2->v()->id() == it1->v()->id()));
      ++it1;
      ++it2;
    }

    CHECK((it1 == res1->end() && it2 == res2->end()));

    LOG(INFO) << VertexId(0, 0) << ": " << eval1->at(VertexId(0, 0));
    LOG(INFO) << VertexId(0, 1) << ": " << eval1->at(VertexId(0, 1));
    LOG(INFO) << VertexId(0, 2) << ": " << eval1->at(VertexId(0, 2));
  }
}
#endif
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
