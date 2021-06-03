#include <gtest/gtest.h>

#include <iostream>
#include <random>
#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/evaluator/mask_evaluator.hpp>
#include <vtr_pose_graph/evaluator/weight_evaluator.hpp>
#include <vtr_pose_graph/index/graph.hpp>

using namespace vtr::pose_graph;
using namespace vtr::common;

using SimpleVertex = uint64_t;
using SimpleEdge = std::pair<uint64_t, uint64_t>;

using IntRandType =
    decltype(std::bind(std::uniform_int_distribution<int64_t>{0, 1000},
                       std::mt19937(std::random_device{}())));
using DoubleRandType =
    decltype(std::bind(std::uniform_real_distribution<double>{0.f, 100.f},
                       std::mt19937(std::random_device{}())));

class EvaluatorTest : public ::testing::Test {
 protected:
  EvaluatorTest()
      : graph_(new BasicGraph(0)),
        irnd_(std::bind(std::uniform_int_distribution<int64_t>{0, 1000},
                        std::mt19937(std::random_device{}()))),
        drnd_(std::bind(std::uniform_real_distribution<double>{0.f, 100.f},
                        std::mt19937(std::random_device{}()))) {
  }

  ~EvaluatorTest() override {
  }

  void SetUp() override {
    /* Create the following graph
     * R0: 0 --- 1 --- 2
     *       \
     *        \
     *         \
     * R1: 0 --- 1 --- 2
     *                 |
     * R2: 0 --- 1 --- 2
     *           |
     * R3: 0 --- 1 --- 2
     *                 |
     * R4: 0 --- 1 --- 2
     */

    // Add a graph with 5 runs and 3 vertices per run.
    for (int idx = 0; idx < 5; ++idx) {
      graph_->addRun();
      graph_->addVertex();
      graph_->addVertex();
      graph_->addVertex();
      graph_->addEdge(VertexId(idx, 0), VertexId(idx, 1));
      graph_->addEdge(VertexId(idx, 1), VertexId(idx, 2));
    }
    // Add spatial edges across runs.
    graph_->addEdge(VertexId(1, 1), VertexId(0, 0), Spatial);
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
  }

  void TearDown() override {
  }

  SimpleVertex rndSimpleVertex() {
    return SimpleVertex(irnd_());
  }

  SimpleEdge rndSimpleEdge() {
    return SimpleEdge(irnd_(), irnd_());
  }

  VertexId rndVertexId() {
    return VertexId(irnd_(), irnd_());
  }

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

  BasicGraph::UniquePtr graph_;
  IntRandType irnd_;
  DoubleRandType drnd_;
};

TEST_F(EvaluatorTest, WeightConstant) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100},
                        std::mt19937(std::random_device{}()));

  double eweight = drnd(), vweight = drnd();
  auto eval = eval::Weight::Const::MakeShared(eweight, vweight);

  eval->setGraph(graph_.get());

  EXPECT_EQ(eval->at(rndSimpleEdge()), eweight);
  EXPECT_EQ(eval->at(rndSimpleVertex()), vweight);

  EXPECT_EQ(eval->at(rndEdgeId(EdgeId::Type::Spatial)), eweight);
  EXPECT_EQ(eval->at(rndEdgeId(EdgeId::Type::Temporal)), eweight);
  EXPECT_EQ(eval->at(rndEdgeId(EdgeId::Type::UNDEFINED)), eweight);
  EXPECT_EQ(eval->at(rndVertexId()), vweight);

  EXPECT_EQ((*eval)[rndSimpleEdge()], eweight);
  EXPECT_EQ((*eval)[rndSimpleVertex()], vweight);

  EXPECT_EQ((*eval)[rndEdgeId(EdgeId::Type::Spatial)], eweight);
  EXPECT_EQ((*eval)[rndEdgeId(EdgeId::Type::Temporal)], eweight);
  EXPECT_EQ((*eval)[rndEdgeId(EdgeId::Type::UNDEFINED)], eweight);
  EXPECT_EQ((*eval)[rndVertexId()], vweight);
}

TEST_F(EvaluatorTest, WeightMap) {
  auto eval = eval::Weight::Map::MakeShared();

  eval->setGraph(graph_.get());

  // Assign weights based on a pattern
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    eval->ref(it->first) =
        it->second->id().minorId() + 10.f * it->second->id().majorId();
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    eval->ref(it->first) = it->second->id().minorId1() +
                           it->second->id().minorId2() +
                           10.f * it->second->id().majorId();
  }

  // Check using both GraphId and SimpleId
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    EXPECT_EQ(eval->at(it->first),
              it->second->id().minorId() + 10.f * it->second->id().majorId());
    EXPECT_EQ(eval->at(it->second->id()),
              it->second->id().minorId() + 10.f * it->second->id().majorId());
    EXPECT_EQ((*eval)[it->first],
              it->second->id().minorId() + 10.f * it->second->id().majorId());
    EXPECT_EQ((*eval)[it->second->id()],
              it->second->id().minorId() + 10.f * it->second->id().majorId());
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    EXPECT_EQ(eval->at(it->first),
              (it->second->id().minorId1() + it->second->id().minorId2() +
               10.f * it->second->id().majorId()));
    EXPECT_EQ(eval->at(it->second->id()),
              (it->second->id().minorId1() + it->second->id().minorId2() +
               10.f * it->second->id().majorId()));
    EXPECT_EQ((*eval)[it->first],
              (it->second->id().minorId1() + it->second->id().minorId2() +
               10.f * it->second->id().majorId()));
    EXPECT_EQ((*eval)[it->second->id()],
              (it->second->id().minorId1() + it->second->id().minorId2() +
               10.f * it->second->id().majorId()));
  }

  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    eval->ref(it->second->id()) =
        17.f * (it->second->id().minorId() + 10.f * it->second->id().majorId());
  }

  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    eval->ref(it->second->id()) =
        17.f * (it->second->id().minorId1() + it->second->id().minorId2() +
                10.f * it->second->id().majorId());
  }

  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    EXPECT_EQ(eval->at(it->first),
              (17.f * (it->second->id().minorId() +
                       10.f * it->second->id().majorId())));
    EXPECT_EQ(eval->at(it->second->id()),
              (17.f * (it->second->id().minorId() +
                       10.f * it->second->id().majorId())));
    EXPECT_EQ((*eval)[it->first], (17.f * (it->second->id().minorId() +
                                           10.f * it->second->id().majorId())));
    EXPECT_EQ((*eval)[it->second->id()],
              (17.f * (it->second->id().minorId() +
                       10.f * it->second->id().majorId())));
  }

  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    EXPECT_EQ(
        eval->at(it->first),
        (17.f * (it->second->id().minorId1() + it->second->id().minorId2() +
                 10.f * it->second->id().majorId())));
    EXPECT_EQ(
        eval->at(it->second->id()),
        (17.f * (it->second->id().minorId1() + it->second->id().minorId2() +
                 10.f * it->second->id().majorId())));
    EXPECT_EQ((*eval)[it->first], (17.f * (it->second->id().minorId1() +
                                           it->second->id().minorId2() +
                                           10.f * it->second->id().majorId())));
    EXPECT_EQ(
        (*eval)[it->second->id()],
        (17.f * (it->second->id().minorId1() + it->second->id().minorId2() +
                 10.f * it->second->id().majorId())));
  }
}

TEST_F(EvaluatorTest, WeightBasicMath) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100},
                        std::mt19937(std::random_device{}()));

  auto eval = eval::Weight::Map::MakeShared();
  // Assign some random weights
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it)
    eval->ref(it->first) = drnd();
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it)
    eval->ref(it->first) = drnd();

  double scale = (drnd() - 50.f) / 25.f;  // Scale factor in the range [-2,2]
  double offset =
      (drnd() - 50.f) / 5.f;  // Additive offset in the range [-10, 10]

  eval::Weight::Ptr a1 = Add(eval, offset);
  eval::Weight::Ptr a2 = Add(offset, eval);
  eval::Weight::Ptr a3 = Add(eval, eval);

  eval::Weight::Ptr s1 = Sub(eval, offset);
  eval::Weight::Ptr s2 = Sub(offset, eval);
  eval::Weight::Ptr s3 = Sub(eval, eval);

  eval::Weight::Ptr m1 = Mul(eval, scale);
  eval::Weight::Ptr m2 = Mul(scale, eval);
  eval::Weight::Ptr m3 = Mul(eval, eval);

  eval::Weight::Ptr d1 = Div(eval, scale);
  eval::Weight::Ptr d2 = Div(scale, eval);
  eval::Weight::Ptr d3 = Div(eval, eval);

  eval::Weight::Ptr n1 = Neg(eval);

  // One complicated test to see if it breaks: -(a + x) / (a - x) * x
  eval::Weight::Ptr QQ =
      Mul(Neg(Div(Add(offset, eval), Sub(offset, eval))), eval);

  // Ensure we can call setGraph on combined evaluators
  a1->setGraph(graph_.get());
  a2->setGraph(graph_.get());
  a3->setGraph(graph_.get());

  s1->setGraph(graph_.get());
  s2->setGraph(graph_.get());
  s3->setGraph(graph_.get());

  m1->setGraph(graph_.get());
  m2->setGraph(graph_.get());
  m3->setGraph(graph_.get());

  d1->setGraph(graph_.get());
  d2->setGraph(graph_.get());
  d3->setGraph(graph_.get());

  n1->setGraph(graph_.get());
  QQ->setGraph(graph_.get());

  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    auto res = eval->at(it->first);

    EXPECT_NEAR(a1->at(it->first), res + offset, 1e-10);
    EXPECT_NEAR(a2->at(it->first), offset + res, 1e-10);
    EXPECT_NEAR(a3->at(it->first), res + res, 1e-10);
    EXPECT_NEAR((*a1)[it->first], res + offset, 1e-10);
    EXPECT_NEAR((*a2)[it->first], offset + res, 1e-10);
    EXPECT_NEAR((*a3)[it->first], res + res, 1e-10);

    EXPECT_NEAR(s1->at(it->first), res - offset, 1e-10);
    EXPECT_NEAR(s2->at(it->first), offset - res, 1e-10);
    EXPECT_NEAR(s3->at(it->first), res - res, 1e-10);
    EXPECT_NEAR((*s1)[it->first], res - offset, 1e-10);
    EXPECT_NEAR((*s2)[it->first], offset - res, 1e-10);
    EXPECT_NEAR((*s3)[it->first], res - res, 1e-10);

    EXPECT_NEAR(m1->at(it->first), res * scale, 1e-10);
    EXPECT_NEAR(m2->at(it->first), scale * res, 1e-10);
    EXPECT_NEAR(m3->at(it->first), res * res, 1e-10);
    EXPECT_NEAR((*m1)[it->first], res * scale, 1e-10);
    EXPECT_NEAR((*m2)[it->first], scale * res, 1e-10);
    EXPECT_NEAR((*m3)[it->first], res * res, 1e-10);

    EXPECT_NEAR(d1->at(it->first), res / scale, 1e-10);
    EXPECT_NEAR((*d1)[it->first], res / scale, 1e-10);

    // Division special case: we can't divide by zero
    if (res != 0.f) {
      EXPECT_NEAR(d2->at(it->first), scale / res, 1e-10);
      EXPECT_NEAR(d3->at(it->first), 1.f, 1e-10);
      EXPECT_NEAR((*d2)[it->first], scale / res, 1e-10);
      EXPECT_NEAR((*d3)[it->first], 1.f, 1e-10);
    }

    EXPECT_NEAR(n1->at(it->first), -res, 1e-10);
    EXPECT_NEAR(QQ->at(it->first), -res * (offset + res) / (offset - res),
                1e-6);
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    auto res = eval->at(it->first);

    EXPECT_NEAR(a1->at(it->first), res + offset, 1e-10);
    EXPECT_NEAR(a2->at(it->first), offset + res, 1e-10);
    EXPECT_NEAR(a3->at(it->first), res + res, 1e-10);
    EXPECT_NEAR((*a1)[it->first], res + offset, 1e-10);
    EXPECT_NEAR((*a2)[it->first], offset + res, 1e-10);
    EXPECT_NEAR((*a3)[it->first], res + res, 1e-10);

    EXPECT_NEAR(s1->at(it->first), res - offset, 1e-10);
    EXPECT_NEAR(s2->at(it->first), offset - res, 1e-10);
    EXPECT_NEAR(s3->at(it->first), res - res, 1e-10);
    EXPECT_NEAR((*s1)[it->first], res - offset, 1e-10);
    EXPECT_NEAR((*s2)[it->first], offset - res, 1e-10);
    EXPECT_NEAR((*s3)[it->first], res - res, 1e-10);

    EXPECT_NEAR(m1->at(it->first), res * scale, 1e-10);
    EXPECT_NEAR(m2->at(it->first), scale * res, 1e-10);
    EXPECT_NEAR(m3->at(it->first), res * res, 1e-10);
    EXPECT_NEAR((*m1)[it->first], res * scale, 1e-10);
    EXPECT_NEAR((*m2)[it->first], scale * res, 1e-10);
    EXPECT_NEAR((*m3)[it->first], res * res, 1e-10);

    EXPECT_NEAR(d1->at(it->first), res / scale, 1e-10);
    EXPECT_NEAR((*d1)[it->first], res / scale, 1e-10);

    // Division special case: we can't divide by zero
    if (res != 0.f) {
      EXPECT_NEAR(d2->at(it->first), scale / res, 1e-10);
      EXPECT_NEAR(d3->at(it->first), 1.f, 1e-10);
      EXPECT_NEAR((*d2)[it->first], scale / res, 1e-10);
      EXPECT_NEAR((*d3)[it->first], 1.f, 1e-10);
    }

    EXPECT_NEAR(n1->at(it->first), -res, 1e-10);
    EXPECT_NEAR(QQ->at(it->first), -res * (offset + res) / (offset - res),
                1e-6);
  }
}

TEST_F(EvaluatorTest, WeightScalarMath) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100},
                        std::mt19937(std::random_device{}()));

  auto eval = eval::Weight::Map::MakeShared();

  // Assign some random weights
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it)
    eval->ref(it->first) = (drnd() - 50.f) / 10.f;
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it)
    eval->ref(it->first) = (drnd() - 50.f) / 10.f;

  double exponent = drnd() / 20.f;  // Exponent in the range [0,5]

  eval::Weight::Ptr abs = Abs(eval);
  eval::Weight::Ptr exp = Exp(eval);
  eval::Weight::Ptr log = Log(abs);  // We need to keep this positive

  eval::Weight::Ptr sgm = Sgm(eval);
  eval::Weight::Ptr erf = Erf(eval);
  eval::Weight::Ptr erfc = Erfc(eval);

  eval::Weight::Ptr pow1 = Pow(abs, exponent);
  eval::Weight::Ptr pow2 = Pow(exponent, eval);
  eval::Weight::Ptr pow3 = Pow(abs, eval);

  // Ensure we can call setGraph on combined evaluators
  pow1->setGraph(graph_.get());
  pow2->setGraph(graph_.get());
  pow3->setGraph(graph_.get());

  abs->setGraph(graph_.get());
  exp->setGraph(graph_.get());
  log->setGraph(graph_.get());
  sgm->setGraph(graph_.get());
  erf->setGraph(graph_.get());
  erfc->setGraph(graph_.get());

  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    auto res = eval->at(it->first);

    EXPECT_NEAR(abs->at(it->first), std::abs(res), 1e-10);
    EXPECT_NEAR(exp->at(it->first), std::exp(res), 1e-10);
    EXPECT_NEAR(log->at(it->first), std::log(std::abs(res)), 1e-10);
    EXPECT_NEAR(sgm->at(it->first), 1.f / (1.f + std::exp(-res)), 1e-10);
    EXPECT_NEAR(erf->at(it->first), std::erf(res), 1e-10);
    EXPECT_NEAR(erfc->at(it->first), std::erfc(res), 1e-10);

    EXPECT_NEAR((*abs)[it->first], std::abs(res), 1e-10);
    EXPECT_NEAR((*exp)[it->first], std::exp(res), 1e-10);
    EXPECT_NEAR((*log)[it->first], std::log(std::abs(res)), 1e-10);
    EXPECT_NEAR((*sgm)[it->first], 1.f / (1.f + std::exp(-res)), 1e-10);
    EXPECT_NEAR((*erf)[it->first], std::erf(res), 1e-10);
    EXPECT_NEAR((*erfc)[it->first], std::erfc(res), 1e-10);

    EXPECT_NEAR(pow1->at(it->first), std::pow(std::abs(res), exponent), 1e-10);
    EXPECT_NEAR(pow2->at(it->first), std::pow(exponent, res), 1e-10);
    EXPECT_NEAR(pow3->at(it->first), std::pow(std::abs(res), res), 1e-10);

    EXPECT_NEAR((*pow1)[it->first], std::pow(std::abs(res), exponent), 1e-10);
    EXPECT_NEAR((*pow2)[it->first], std::pow(exponent, res), 1e-10);
    EXPECT_NEAR((*pow3)[it->first], std::pow(std::abs(res), res), 1e-10);
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    auto res = eval->at(it->first);

    EXPECT_NEAR(abs->at(it->first), std::abs(res), 1e-10);
    EXPECT_NEAR(exp->at(it->first), std::exp(res), 1e-10);
    EXPECT_NEAR(log->at(it->first), std::log(std::abs(res)), 1e-10);
    EXPECT_NEAR(sgm->at(it->first), 1.f / (1.f + std::exp(-res)), 1e-10);
    EXPECT_NEAR(erf->at(it->first), std::erf(res), 1e-10);
    EXPECT_NEAR(erfc->at(it->first), std::erfc(res), 1e-10);

    EXPECT_NEAR((*abs)[it->first], std::abs(res), 1e-10);
    EXPECT_NEAR((*exp)[it->first], std::exp(res), 1e-10);
    EXPECT_NEAR((*log)[it->first], std::log(std::abs(res)), 1e-10);
    EXPECT_NEAR((*sgm)[it->first], 1.f / (1.f + std::exp(-res)), 1e-10);
    EXPECT_NEAR((*erf)[it->first], std::erf(res), 1e-10);
    EXPECT_NEAR((*erfc)[it->first], std::erfc(res), 1e-10);

    EXPECT_NEAR(pow1->at(it->first), std::pow(std::abs(res), exponent), 1e-10);
    EXPECT_NEAR(pow2->at(it->first), std::pow(exponent, res), 1e-10);
    EXPECT_NEAR(pow3->at(it->first), std::pow(std::abs(res), res), 1e-10);

    EXPECT_NEAR((*pow1)[it->first], std::pow(std::abs(res), exponent), 1e-10);
    EXPECT_NEAR((*pow2)[it->first], std::pow(exponent, res), 1e-10);
    EXPECT_NEAR((*pow3)[it->first], std::pow(std::abs(res), res), 1e-10);
  }
}

TEST_F(EvaluatorTest, WeightComparison) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100},
                        std::mt19937(std::random_device{}()));

  auto eval1 = eval::Weight::Map::MakeShared();  // [-1, 1]
  auto eval2 = eval::Weight::Map::MakeShared();  // [-1, 1]

  // Assign some random weights in the range [-pi, pi]
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    eval1->ref(it->first) = (drnd() - 50.f) / 50.f;
    eval2->ref(it->first) = (drnd() - 50.f) / 50.f;
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    eval1->ref(it->first) = (drnd() - 50.f) / 50.f;
    eval2->ref(it->first) = (drnd() - 50.f) / 50.f;
  }

  double v11 = eval1->at(graph_->vertices()->begin()->first);
  double e11 = eval1->at(graph_->edges()->begin()->first);

  eval::Mask::Ptr eq11 = Equal(eval1, eval1);
  eval::Mask::Ptr eq22 = Equal(eval2, eval2);
  eval::Mask::Ptr eq12 = Equal(eval1, eval2);
  eval::Mask::Ptr eq21 = Equal(eval2, eval1);
  eval::Mask::Ptr eq1cv = Equal(eval1, v11);
  eval::Mask::Ptr eqc1v = Equal(v11, eval1);
  eval::Mask::Ptr eq1ce = Equal(eval1, e11);
  eval::Mask::Ptr eqc1e = Equal(e11, eval1);

  eval::Mask::Ptr ne11 = NEqual(eval1, eval1);
  eval::Mask::Ptr ne22 = NEqual(eval2, eval2);
  eval::Mask::Ptr ne12 = NEqual(eval1, eval2);
  eval::Mask::Ptr ne21 = NEqual(eval2, eval1);
  eval::Mask::Ptr ne1cv = NEqual(eval1, v11);
  eval::Mask::Ptr nec1v = NEqual(v11, eval1);
  eval::Mask::Ptr ne1ce = NEqual(eval1, e11);
  eval::Mask::Ptr nec1e = NEqual(e11, eval1);

  eval::Mask::Ptr gt1 = Greater(eval1, eval2);
  eval::Mask::Ptr gt2 = Greater(eval1, 0);
  eval::Mask::Ptr gt3 = Greater(0, eval1);

  eval::Mask::Ptr lt1 = Less(eval1, eval2);
  eval::Mask::Ptr lt2 = Less(eval1, 0);
  eval::Mask::Ptr lt3 = Less(0, eval1);

  eval::Mask::Ptr ge1 = GEqual(eval1, eval2);
  eval::Mask::Ptr ge2 = GEqual(eval1, 0);
  eval::Mask::Ptr ge3 = GEqual(0, eval1);

  eval::Mask::Ptr le1 = LEqual(eval1, eval2);
  eval::Mask::Ptr le2 = LEqual(eval1, 0);
  eval::Mask::Ptr le3 = LEqual(0, eval1);

  // Ensure we can call setGraph on combined evaluators
  eq11->setGraph(graph_.get());
  eq22->setGraph(graph_.get());
  eq12->setGraph(graph_.get());
  eq21->setGraph(graph_.get());
  eq1cv->setGraph(graph_.get());
  eqc1v->setGraph(graph_.get());
  eq1ce->setGraph(graph_.get());
  eqc1e->setGraph(graph_.get());

  ne11->setGraph(graph_.get());
  ne22->setGraph(graph_.get());
  ne12->setGraph(graph_.get());
  ne21->setGraph(graph_.get());
  ne1cv->setGraph(graph_.get());
  nec1v->setGraph(graph_.get());
  ne1ce->setGraph(graph_.get());
  nec1e->setGraph(graph_.get());

  gt1->setGraph(graph_.get());
  gt2->setGraph(graph_.get());
  gt3->setGraph(graph_.get());

  lt1->setGraph(graph_.get());
  lt2->setGraph(graph_.get());
  lt3->setGraph(graph_.get());

  ge1->setGraph(graph_.get());
  ge2->setGraph(graph_.get());
  ge3->setGraph(graph_.get());

  le1->setGraph(graph_.get());
  le2->setGraph(graph_.get());
  le3->setGraph(graph_.get());

  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    auto res1 = eval1->at(it->first);
    auto res2 = eval2->at(it->first);

    EXPECT_TRUE(eq11->at(it->first));
    EXPECT_TRUE(eq22->at(it->first));
    EXPECT_EQ(eq12->at(it->first), (res1 == res2));
    EXPECT_EQ(eq21->at(it->first), (res1 == res2));
    EXPECT_EQ(eq1cv->at(it->first), (res1 == v11));
    EXPECT_EQ(eqc1v->at(it->first), (res1 == v11));

    EXPECT_TRUE((*eq11)[it->first]);
    EXPECT_TRUE((*eq22)[it->first]);
    EXPECT_EQ((*eq12)[it->first], (res1 == res2));
    EXPECT_EQ((*eq21)[it->first], (res1 == res2));
    EXPECT_EQ((*eq1cv)[it->first], (res1 == v11));
    EXPECT_EQ((*eqc1v)[it->first], (res1 == v11));

    EXPECT_TRUE(!ne11->at(it->first));
    EXPECT_TRUE(!ne22->at(it->first));
    EXPECT_EQ(ne12->at(it->first), (res1 != res2));
    EXPECT_EQ(ne21->at(it->first), (res1 != res2));
    EXPECT_EQ(ne1cv->at(it->first), (res1 != v11));
    EXPECT_EQ(nec1v->at(it->first), (res1 != v11));

    EXPECT_TRUE(!(*ne11)[it->first]);
    EXPECT_TRUE(!(*ne22)[it->first]);
    EXPECT_EQ((*ne12)[it->first], (res1 != res2));
    EXPECT_EQ((*ne21)[it->first], (res1 != res2));
    EXPECT_EQ((*ne1cv)[it->first], (res1 != v11));
    EXPECT_EQ((*nec1v)[it->first], (res1 != v11));

    EXPECT_EQ(gt1->at(it->first), (res1 > res2));
    EXPECT_EQ(gt2->at(it->first), (res1 > 0));
    EXPECT_EQ(gt3->at(it->first), (0 > res1));

    EXPECT_EQ((*gt1)[it->first], (res1 > res2));
    EXPECT_EQ((*gt2)[it->first], (res1 > 0));
    EXPECT_EQ((*gt3)[it->first], (0 > res1));

    EXPECT_EQ(lt1->at(it->first), (res1 < res2));
    EXPECT_EQ(lt2->at(it->first), (res1 < 0));
    EXPECT_EQ(lt3->at(it->first), (0 < res1));

    EXPECT_EQ((*lt1)[it->first], (res1 < res2));
    EXPECT_EQ((*lt2)[it->first], (res1 < 0));
    EXPECT_EQ((*lt3)[it->first], (0 < res1));

    EXPECT_EQ(ge1->at(it->first), (res1 >= res2));
    EXPECT_EQ(ge2->at(it->first), (res1 >= 0));
    EXPECT_EQ(ge3->at(it->first), (0 >= res1));

    EXPECT_EQ((*ge1)[it->first], (res1 >= res2));
    EXPECT_EQ((*ge2)[it->first], (res1 >= 0));
    EXPECT_EQ((*ge3)[it->first], (0 >= res1));

    EXPECT_EQ(le1->at(it->first), (res1 <= res2));
    EXPECT_EQ(le2->at(it->first), (res1 <= 0));
    EXPECT_EQ(le3->at(it->first), (0 <= res1));

    EXPECT_EQ((*le1)[it->first], (res1 <= res2));
    EXPECT_EQ((*le2)[it->first], (res1 <= 0));
    EXPECT_EQ((*le3)[it->first], (0 <= res1));
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    auto res1 = eval1->at(it->first);
    auto res2 = eval2->at(it->first);

    EXPECT_TRUE(eq11->at(it->first));
    EXPECT_TRUE(eq22->at(it->first));
    EXPECT_EQ(eq12->at(it->first), (res1 == res2));
    EXPECT_EQ(eq21->at(it->first), (res1 == res2));
    EXPECT_EQ(eq1ce->at(it->first), (res1 == e11));
    EXPECT_EQ(eqc1e->at(it->first), (res1 == e11));

    EXPECT_TRUE((*eq11)[it->first]);
    EXPECT_TRUE((*eq22)[it->first]);
    EXPECT_EQ((*eq12)[it->first], (res1 == res2));
    EXPECT_EQ((*eq21)[it->first], (res1 == res2));
    EXPECT_EQ((*eq1ce)[it->first], (res1 == e11));
    EXPECT_EQ((*eqc1e)[it->first], (res1 == e11));

    EXPECT_TRUE(!ne11->at(it->first));
    EXPECT_TRUE(!ne22->at(it->first));
    EXPECT_EQ(ne12->at(it->first), (res1 != res2));
    EXPECT_EQ(ne21->at(it->first), (res1 != res2));
    EXPECT_EQ(ne1ce->at(it->first), (res1 != e11));
    EXPECT_EQ(nec1e->at(it->first), (res1 != e11));

    EXPECT_TRUE(!(*ne11)[it->first]);
    EXPECT_TRUE(!(*ne22)[it->first]);
    EXPECT_EQ((*ne12)[it->first], (res1 != res2));
    EXPECT_EQ((*ne21)[it->first], (res1 != res2));
    EXPECT_EQ((*ne1ce)[it->first], (res1 != e11));
    EXPECT_EQ((*nec1e)[it->first], (res1 != e11));

    EXPECT_EQ(gt1->at(it->first), (res1 > res2));
    EXPECT_EQ(gt2->at(it->first), (res1 > 0));
    EXPECT_EQ(gt3->at(it->first), (0 > res1));

    EXPECT_EQ((*gt1)[it->first], (res1 > res2));
    EXPECT_EQ((*gt2)[it->first], (res1 > 0));
    EXPECT_EQ((*gt3)[it->first], (0 > res1));

    EXPECT_EQ(lt1->at(it->first), (res1 < res2));
    EXPECT_EQ(lt2->at(it->first), (res1 < 0));
    EXPECT_EQ(lt3->at(it->first), (0 < res1));

    EXPECT_EQ((*lt1)[it->first], (res1 < res2));
    EXPECT_EQ((*lt2)[it->first], (res1 < 0));
    EXPECT_EQ((*lt3)[it->first], (0 < res1));

    EXPECT_EQ(ge1->at(it->first), (res1 >= res2));
    EXPECT_EQ(ge2->at(it->first), (res1 >= 0));
    EXPECT_EQ(ge3->at(it->first), (0 >= res1));

    EXPECT_EQ((*ge1)[it->first], (res1 >= res2));
    EXPECT_EQ((*ge2)[it->first], (res1 >= 0));
    EXPECT_EQ((*ge3)[it->first], (0 >= res1));

    EXPECT_EQ(le1->at(it->first), (res1 <= res2));
    EXPECT_EQ(le2->at(it->first), (res1 <= 0));
    EXPECT_EQ(le3->at(it->first), (0 <= res1));

    EXPECT_EQ((*le1)[it->first], (res1 <= res2));
    EXPECT_EQ((*le2)[it->first], (res1 <= 0));
    EXPECT_EQ((*le3)[it->first], (0 <= res1));
  }
}

TEST_F(EvaluatorTest, WeightTrigonometry) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100},
                        std::mt19937(std::random_device{}()));

  auto eval = eval::Weight::Map::MakeShared();
  auto inveval = eval::Weight::Map::MakeShared();

  // Assign some random weights in the range [-pi, pi]
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    eval->ref(it->first) = (drnd() - 50.f) / 50.f * M_PI;
    inveval->ref(it->first) = (drnd() - 50.f) / 50.f;
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    eval->ref(it->first) = (drnd() - 50.f) / 50.f * M_PI;
    inveval->ref(it->first) = (drnd() - 50.f) / 50.f;
  }

  eval::Weight::Ptr sin = Sin(eval);
  eval::Weight::Ptr cos = Cos(eval);
  eval::Weight::Ptr tan = Tan(eval);

  eval::Weight::Ptr asin = ASin(inveval);
  eval::Weight::Ptr acos = ACos(inveval);
  eval::Weight::Ptr atan = ATan(inveval);

  eval::Weight::Ptr atan2 = ATan2(Div(eval, M_PI), inveval);

  // Ensure we can call setGraph on combined evaluators
  sin->setGraph(graph_.get());
  cos->setGraph(graph_.get());
  tan->setGraph(graph_.get());
  acos->setGraph(graph_.get());
  asin->setGraph(graph_.get());
  atan->setGraph(graph_.get());
  atan2->setGraph(graph_.get());

  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    auto res = eval->at(it->first);
    auto invres = inveval->at(it->first);

    EXPECT_EQ(sin->at(it->first), std::sin(res));
    EXPECT_EQ(cos->at(it->first), std::cos(res));
    EXPECT_EQ(tan->at(it->first), std::tan(res));
    EXPECT_EQ(asin->at(it->first), std::asin(invres));
    EXPECT_EQ(acos->at(it->first), std::acos(invres));
    EXPECT_EQ(atan->at(it->first), std::atan(invres));

    EXPECT_EQ((*sin)[it->first], std::sin(res));
    EXPECT_EQ((*cos)[it->first], std::cos(res));
    EXPECT_EQ((*tan)[it->first], std::tan(res));
    EXPECT_EQ((*asin)[it->first], std::asin(invres));
    EXPECT_EQ((*acos)[it->first], std::acos(invres));
    EXPECT_EQ((*atan)[it->first], std::atan(invres));

    EXPECT_EQ((*atan2)[it->first], std::atan2(res / M_PI, invres));
  }

  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    auto res = eval->at(it->first);
    auto invres = inveval->at(it->first);

    EXPECT_EQ(sin->at(it->first), std::sin(res));
    EXPECT_EQ(cos->at(it->first), std::cos(res));
    EXPECT_EQ(tan->at(it->first), std::tan(res));
    EXPECT_EQ(asin->at(it->first), std::asin(invres));
    EXPECT_EQ(acos->at(it->first), std::acos(invres));
    EXPECT_EQ(atan->at(it->first), std::atan(invres));

    EXPECT_EQ((*sin)[it->first], std::sin(res));
    EXPECT_EQ((*cos)[it->first], std::cos(res));
    EXPECT_EQ((*tan)[it->first], std::tan(res));
    EXPECT_EQ((*asin)[it->first], std::asin(invres));
    EXPECT_EQ((*acos)[it->first], std::acos(invres));
    EXPECT_EQ((*atan)[it->first], std::atan(invres));

    EXPECT_EQ((*atan2)[it->first], std::atan2(res / M_PI, invres));
  }
}

TEST_F(EvaluatorTest, WeightHyperbolicTrigonometry) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100},
                        std::mt19937(std::random_device{}()));

  auto eval1 = eval::Weight::Map::MakeShared();  // [-50, 50]
  auto eval2 = eval::Weight::Map::MakeShared();  // [1, 50]
  auto eval3 = eval::Weight::Map::MakeShared();  // [-1, 1]

  // Assign some random weights in the range [-pi, pi]
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    eval1->ref(it->first) = (drnd() - 50.f);
    eval2->ref(it->first) = (drnd()) / 2.0 + 1.f;
    eval3->ref(it->first) = (drnd() - 50.f) / 50.f;
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    eval1->ref(it->first) = (drnd() - 50.f);
    eval2->ref(it->first) = (drnd()) / 2.0 + 1.f;
    eval3->ref(it->first) = (drnd() - 50.f) / 50.f;
  }

  eval::Weight::Ptr sinh = Sinh(eval1);
  eval::Weight::Ptr cosh = Cosh(eval1);
  eval::Weight::Ptr tanh = Tanh(eval1);

  eval::Weight::Ptr asinh = ASinh(eval1);
  eval::Weight::Ptr acosh = ACosh(eval2);
  eval::Weight::Ptr atanh = ATanh(eval3);

  // Ensure we can call setGraph on combined evaluators
  sinh->setGraph(graph_.get());
  cosh->setGraph(graph_.get());
  tanh->setGraph(graph_.get());
  acosh->setGraph(graph_.get());
  asinh->setGraph(graph_.get());
  atanh->setGraph(graph_.get());

  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    auto res1 = eval1->at(it->first);
    auto res2 = eval2->at(it->first);
    auto res3 = eval3->at(it->first);

    EXPECT_EQ(sinh->at(it->first), std::sinh(res1));
    EXPECT_EQ(cosh->at(it->first), std::cosh(res1));
    EXPECT_EQ(tanh->at(it->first), std::tanh(res1));
    EXPECT_EQ(asinh->at(it->first), std::asinh(res1));
    EXPECT_EQ(acosh->at(it->first), std::acosh(res2));
    EXPECT_EQ(atanh->at(it->first), std::atanh(res3));

    EXPECT_EQ((*sinh)[it->first], std::sinh(res1));
    EXPECT_EQ((*cosh)[it->first], std::cosh(res1));
    EXPECT_EQ((*tanh)[it->first], std::tanh(res1));
    EXPECT_EQ((*asinh)[it->first], std::asinh(res1));
    EXPECT_EQ((*acosh)[it->first], std::acosh(res2));
    EXPECT_EQ((*atanh)[it->first], std::atanh(res3));
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    auto res1 = eval1->at(it->first);
    auto res2 = eval2->at(it->first);
    auto res3 = eval3->at(it->first);

    EXPECT_EQ(sinh->at(it->first), std::sinh(res1));
    EXPECT_EQ(cosh->at(it->first), std::cosh(res1));
    EXPECT_EQ(tanh->at(it->first), std::tanh(res1));
    EXPECT_EQ(asinh->at(it->first), std::asinh(res1));
    EXPECT_EQ(acosh->at(it->first), std::acosh(res2));
    EXPECT_EQ(atanh->at(it->first), std::atanh(res3));

    EXPECT_EQ((*sinh)[it->first], std::sinh(res1));
    EXPECT_EQ((*cosh)[it->first], std::cosh(res1));
    EXPECT_EQ((*tanh)[it->first], std::tanh(res1));
    EXPECT_EQ((*asinh)[it->first], std::asinh(res1));
    EXPECT_EQ((*acosh)[it->first], std::acosh(res2));
    EXPECT_EQ((*atanh)[it->first], std::atanh(res3));
  }
}

TEST_F(EvaluatorTest, MaskConstant) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100},
                        std::mt19937(std::random_device{}()));

  bool vmask = drnd() > 50.0, emask = drnd() > 50.0;
  auto eval = eval::Mask::Const::MakeShared(emask, vmask);

  eval->setGraph(graph_.get());

  EXPECT_EQ(eval->at(rndSimpleEdge()), emask);
  EXPECT_EQ(eval->at(rndSimpleVertex()), vmask);

  EXPECT_EQ(eval->at(rndEdgeId(EdgeId::Type::Spatial)), emask);
  EXPECT_EQ(eval->at(rndEdgeId(EdgeId::Type::Temporal)), emask);
  EXPECT_EQ(eval->at(rndEdgeId(EdgeId::Type::UNDEFINED)), emask);
  EXPECT_EQ(eval->at(rndVertexId()), vmask);

  EXPECT_EQ((*eval)[rndSimpleEdge()], emask);
  EXPECT_EQ((*eval)[rndSimpleVertex()], vmask);

  EXPECT_EQ((*eval)[rndEdgeId(EdgeId::Type::Spatial)], emask);
  EXPECT_EQ((*eval)[rndEdgeId(EdgeId::Type::Temporal)], emask);
  EXPECT_EQ((*eval)[rndEdgeId(EdgeId::Type::UNDEFINED)], emask);
  EXPECT_EQ((*eval)[rndVertexId()], vmask);
}

TEST_F(EvaluatorTest, MaskMap) {
  auto eval = eval::Mask::Map::MakeShared();

  eval->setGraph(graph_.get());

  // Assign masks based on a pattern: true if sum of minor and major ids is
  // EVEN
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    eval->ref(it->first) =
        (it->second->id().minorId() + it->second->id().majorId()) % 2 == 0;
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    eval->ref(it->first) =
        (((it->second->id().minorId1() + it->second->id().minorId2() +
           it->second->id().majorId()) %
          2) == 0);
  }

  // Check using both GraphId and SimpleId
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    EXPECT_EQ(
        eval->at(it->first),
        ((it->second->id().minorId() + it->second->id().majorId()) % 2 == 0));
    EXPECT_EQ(
        eval->at(it->second->id()),
        ((it->second->id().minorId() + it->second->id().majorId()) % 2 == 0));

    EXPECT_EQ(
        (*eval)[it->first],
        ((it->second->id().minorId() + it->second->id().majorId()) % 2 == 0));
    EXPECT_EQ(
        (*eval)[it->second->id()],
        ((it->second->id().minorId() + it->second->id().majorId()) % 2 == 0));
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    EXPECT_EQ(eval->at(it->first),
              ((it->second->id().minorId1() + it->second->id().minorId2() +
                it->second->id().majorId()) %
                   2 ==
               0));
    EXPECT_EQ(eval->at(it->second->id()),
              ((it->second->id().minorId1() + it->second->id().minorId2() +
                it->second->id().majorId()) %
                   2 ==
               0));

    EXPECT_EQ((*eval)[it->first],
              ((it->second->id().minorId1() + it->second->id().minorId2() +
                it->second->id().majorId()) %
                   2 ==
               0));
    EXPECT_EQ((*eval)[it->second->id()],
              ((it->second->id().minorId1() + it->second->id().minorId2() +
                it->second->id().majorId()) %
                   2 ==
               0));
  }

  // Assign masks based on a pattern, using VertexId/EdgeId: true if sum of
  // minor and major ids is ODD
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    eval->ref(it->second->id()) =
        (it->second->id().minorId() + it->second->id().majorId()) % 2 != 0;
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    eval->ref(it->second->id()) =
        ((it->second->id().minorId1() + it->second->id().minorId2() +
          it->second->id().majorId()) %
         2) != 0;
  }

  // Check using both GraphId and SimpleId
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    EXPECT_EQ(
        eval->at(it->first),
        ((it->second->id().minorId() + it->second->id().majorId()) % 2 != 0));
    EXPECT_EQ(
        eval->at(it->second->id()),
        ((it->second->id().minorId() + it->second->id().majorId()) % 2 != 0));

    EXPECT_EQ(
        (*eval)[it->first],
        ((it->second->id().minorId() + it->second->id().majorId()) % 2 != 0));
    EXPECT_EQ(
        (*eval)[it->second->id()],
        ((it->second->id().minorId() + it->second->id().majorId()) % 2 != 0));
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    EXPECT_EQ(eval->at(it->first),
              (((it->second->id().minorId1() + it->second->id().minorId2() +
                 it->second->id().majorId()) %
                2) != 0));
    EXPECT_EQ(eval->at(it->second->id()),
              (((it->second->id().minorId1() + it->second->id().minorId2() +
                 it->second->id().majorId()) %
                2) != 0));

    EXPECT_EQ((*eval)[it->first],
              (((it->second->id().minorId1() + it->second->id().minorId2() +
                 it->second->id().majorId()) %
                2) != 0));
    EXPECT_EQ((*eval)[it->second->id()],
              (((it->second->id().minorId1() + it->second->id().minorId2() +
                 it->second->id().majorId()) %
                2) != 0));
  }
}

TEST_F(EvaluatorTest, MaskBooleanLogic) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100},
                        std::mt19937(std::random_device{}()));

  auto eval1 = eval::Mask::Map::MakeShared();  // 1:1 true:false
  auto eval2 = eval::Mask::Map::MakeShared();  // 1:1 true:false

  // Assign some random weights in the range [-pi, pi]
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    eval1->ref(it->first) = drnd() > 50.f;
    eval2->ref(it->first) = drnd() > 50.f;
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    eval1->ref(it->first) = drnd() > 50.f;
    eval2->ref(it->first) = drnd() > 50.f;
  }

  eval::Mask::Ptr and11 = And(eval1, eval1);  // A & A = A
  eval::Mask::Ptr and22 = And(eval2, eval2);  // B & B = B
  eval::Mask::Ptr and12 = And(eval1, eval2);  // A & B = A & B
  eval::Mask::Ptr and21 = And(eval2, eval1);  // B & A = B & A = A & B
  eval::Mask::Ptr and1t = And(eval1, true);   // A & 1 = A
  eval::Mask::Ptr andt1 = And(true, eval1);   // 1 & A = A
  eval::Mask::Ptr and1f = And(eval1, false);  // A & 0 = 0
  eval::Mask::Ptr andf1 = And(false, eval1);  // 0 & A = 0

  eval::Mask::Ptr or11 = Or(eval1, eval1);  // A | A = A
  eval::Mask::Ptr or22 = Or(eval2, eval2);  // B | B = B
  eval::Mask::Ptr or12 = Or(eval1, eval2);  // A | B = A | B
  eval::Mask::Ptr or21 = Or(eval2, eval1);  // B | A = B | A = A | B
  eval::Mask::Ptr or1t = Or(eval1, true);   // A | 1 = 1
  eval::Mask::Ptr ort1 = Or(true, eval1);   // 1 | A = 1
  eval::Mask::Ptr or1f = Or(eval1, false);  // A | 0 = A
  eval::Mask::Ptr orf1 = Or(false, eval1);  // 0 | A = A

  eval::Mask::Ptr xor11 = Xor(eval1, eval1);  // A ^ A = 0
  eval::Mask::Ptr xor22 = Xor(eval2, eval2);  // B ^ B = 0
  eval::Mask::Ptr xor12 = Xor(eval1, eval2);  // A ^ B = A ^ B
  eval::Mask::Ptr xor21 = Xor(eval2, eval1);  // B ^ A = B ^ A = A ^ B
  eval::Mask::Ptr xor1t = Xor(eval1, true);   // A ^ 1 = ~A
  eval::Mask::Ptr xort1 = Xor(true, eval1);   // 1 ^ A = ~A
  eval::Mask::Ptr xor1f = Xor(eval1, false);  // A ^ 0 = A
  eval::Mask::Ptr xorf1 = Xor(false, eval1);  // 0 ^ A = A

  eval::Mask::Ptr not1 = Not(eval1);  // ~A = ~A

  // Ensure we can call setGraph on combined evaluators
  and11->setGraph(graph_.get());
  and22->setGraph(graph_.get());
  and12->setGraph(graph_.get());
  and21->setGraph(graph_.get());
  and1t->setGraph(graph_.get());
  andt1->setGraph(graph_.get());
  and1f->setGraph(graph_.get());
  andf1->setGraph(graph_.get());

  or11->setGraph(graph_.get());
  or22->setGraph(graph_.get());
  or12->setGraph(graph_.get());
  or21->setGraph(graph_.get());
  or1t->setGraph(graph_.get());
  ort1->setGraph(graph_.get());
  or1f->setGraph(graph_.get());
  orf1->setGraph(graph_.get());

  xor11->setGraph(graph_.get());
  xor22->setGraph(graph_.get());
  xor12->setGraph(graph_.get());
  xor21->setGraph(graph_.get());
  xor1t->setGraph(graph_.get());
  xort1->setGraph(graph_.get());
  xor1f->setGraph(graph_.get());
  xorf1->setGraph(graph_.get());

  not1->setGraph(graph_.get());

  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    auto res1 = eval1->at(it->first);
    auto res2 = eval2->at(it->first);

    EXPECT_EQ(and11->at(it->first), res1);
    EXPECT_EQ(and22->at(it->first), res2);
    EXPECT_EQ(and12->at(it->first), (res1 && res2));
    EXPECT_EQ(and21->at(it->first), (res1 && res2));
    EXPECT_EQ(and1t->at(it->first), res1);
    EXPECT_EQ(andt1->at(it->first), res1);
    EXPECT_TRUE(!and1f->at(it->first));
    EXPECT_TRUE(!andf1->at(it->first));

    EXPECT_EQ((*and11)[it->first], res1);
    EXPECT_EQ((*and22)[it->first], res2);
    EXPECT_EQ((*and12)[it->first], (res1 && res2));
    EXPECT_EQ((*and21)[it->first], (res1 && res2));
    EXPECT_EQ((*and1t)[it->first], res1);
    EXPECT_EQ((*andt1)[it->first], res1);
    EXPECT_FALSE((*and1f)[it->first]);
    EXPECT_FALSE((*andf1)[it->first]);

    EXPECT_EQ(or11->at(it->first), res1);
    EXPECT_EQ(or22->at(it->first), res2);
    EXPECT_EQ(or12->at(it->first), (res1 || res2));
    EXPECT_EQ(or21->at(it->first), (res1 || res2));
    EXPECT_TRUE(or1t->at(it->first));
    EXPECT_TRUE(ort1->at(it->first));
    EXPECT_EQ(or1f->at(it->first), res1);
    EXPECT_EQ(orf1->at(it->first), res1);

    EXPECT_EQ((*or11)[it->first], res1);
    EXPECT_EQ((*or22)[it->first], res2);
    EXPECT_EQ((*or12)[it->first], (res1 || res2));
    EXPECT_EQ((*or21)[it->first], (res1 || res2));
    EXPECT_TRUE((*or1t)[it->first]);
    EXPECT_TRUE((*ort1)[it->first]);
    EXPECT_EQ((*or1f)[it->first], res1);
    EXPECT_EQ((*orf1)[it->first], res1);

    EXPECT_FALSE(xor11->at(it->first));
    EXPECT_FALSE(xor22->at(it->first));
    EXPECT_EQ(xor12->at(it->first), (res1 != res2));
    EXPECT_EQ(xor21->at(it->first), (res1 != res2));
    EXPECT_NE(xor1t->at(it->first), res1);
    EXPECT_NE(xort1->at(it->first), res1);
    EXPECT_EQ(xor1f->at(it->first), res1);
    EXPECT_EQ(xorf1->at(it->first), res1);

    EXPECT_TRUE(!(*xor11)[it->first]);
    EXPECT_TRUE(!(*xor22)[it->first]);
    EXPECT_EQ((*xor12)[it->first], (res1 != res2));
    EXPECT_EQ((*xor21)[it->first], (res1 != res2));
    EXPECT_NE((*xor1t)[it->first], res1);
    EXPECT_NE((*xort1)[it->first], res1);
    EXPECT_EQ((*xor1f)[it->first], res1);
    EXPECT_EQ((*xorf1)[it->first], res1);

    EXPECT_NE(not1->at(it->first), res1);
    EXPECT_NE((*not1)[it->first], res1);
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    auto res1 = eval1->at(it->first);
    auto res2 = eval2->at(it->first);

    EXPECT_EQ(and11->at(it->first), res1);
    EXPECT_EQ(and22->at(it->first), res2);
    EXPECT_EQ(and12->at(it->first), (res1 && res2));
    EXPECT_EQ(and21->at(it->first), (res1 && res2));
    EXPECT_EQ(and1t->at(it->first), res1);
    EXPECT_EQ(andt1->at(it->first), res1);
    EXPECT_FALSE(and1f->at(it->first));
    EXPECT_FALSE(andf1->at(it->first));

    EXPECT_EQ((*and11)[it->first], res1);
    EXPECT_EQ((*and22)[it->first], res2);
    EXPECT_EQ((*and12)[it->first], (res1 && res2));
    EXPECT_EQ((*and21)[it->first], (res1 && res2));
    EXPECT_EQ((*and1t)[it->first], res1);
    EXPECT_EQ((*andt1)[it->first], res1);
    EXPECT_FALSE((*and1f)[it->first]);
    EXPECT_FALSE((*andf1)[it->first]);

    EXPECT_EQ(or11->at(it->first), res1);
    EXPECT_EQ(or22->at(it->first), res2);
    EXPECT_EQ(or12->at(it->first), (res1 || res2));
    EXPECT_EQ(or21->at(it->first), (res1 || res2));
    EXPECT_TRUE(or1t->at(it->first));
    EXPECT_TRUE(ort1->at(it->first));
    EXPECT_EQ(or1f->at(it->first), res1);
    EXPECT_EQ(orf1->at(it->first), res1);

    EXPECT_EQ((*or11)[it->first], res1);
    EXPECT_EQ((*or22)[it->first], res2);
    EXPECT_EQ((*or12)[it->first], (res1 || res2));
    EXPECT_EQ((*or21)[it->first], (res1 || res2));
    EXPECT_TRUE((*or1t)[it->first]);
    EXPECT_TRUE((*ort1)[it->first]);
    EXPECT_EQ((*or1f)[it->first], res1);
    EXPECT_EQ((*orf1)[it->first], res1);

    EXPECT_FALSE(xor11->at(it->first));
    EXPECT_FALSE(xor22->at(it->first));
    EXPECT_EQ(xor12->at(it->first), (res1 != res2));
    EXPECT_EQ(xor21->at(it->first), (res1 != res2));
    EXPECT_NE(xor1t->at(it->first), res1);
    EXPECT_NE(xort1->at(it->first), res1);
    EXPECT_EQ(xor1f->at(it->first), res1);
    EXPECT_EQ(xorf1->at(it->first), res1);

    EXPECT_FALSE((*xor11)[it->first]);
    EXPECT_FALSE((*xor22)[it->first]);
    EXPECT_EQ((*xor12)[it->first], (res1 != res2));
    EXPECT_EQ((*xor21)[it->first], (res1 != res2));
    EXPECT_NE((*xor1t)[it->first], res1);
    EXPECT_NE((*xort1)[it->first], res1);
    EXPECT_EQ((*xor1f)[it->first], res1);
    EXPECT_EQ((*xorf1)[it->first], res1);

    EXPECT_NE(not1->at(it->first), res1);
    EXPECT_NE((*not1)[it->first], res1);
  }
}

TEST_F(EvaluatorTest, MaskComparison) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100},
                        std::mt19937(std::random_device{}()));

  auto eval1 = eval::Mask::Map::MakeShared();  // 1:1 true:false
  auto eval2 = eval::Mask::Map::MakeShared();  // 1:1 true:false

  // Assign some random weights in the range [-pi, pi]
  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    eval1->ref(it->first) = drnd() > 50.f;
    eval2->ref(it->first) = drnd() > 50.f;
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    eval1->ref(it->first) = drnd() > 50.f;
    eval2->ref(it->first) = drnd() > 50.f;
  }

  bool cval = drnd() > 50.0;

  eval::Mask::Ptr eq11 = Equal(eval1, eval1);
  eval::Mask::Ptr eq22 = Equal(eval2, eval2);
  eval::Mask::Ptr eq12 = Equal(eval1, eval2);
  eval::Mask::Ptr eq21 = Equal(eval2, eval1);
  eval::Mask::Ptr eq1c = Equal(eval1, cval);
  eval::Mask::Ptr eqc1 = Equal(cval, eval1);

  eval::Mask::Ptr ne11 = NEqual(eval1, eval1);
  eval::Mask::Ptr ne22 = NEqual(eval2, eval2);
  eval::Mask::Ptr ne12 = NEqual(eval1, eval2);
  eval::Mask::Ptr ne21 = NEqual(eval2, eval1);
  eval::Mask::Ptr ne1c = NEqual(eval1, cval);
  eval::Mask::Ptr nec1 = NEqual(cval, eval1);

  eval::Mask::Ptr gt1 = Greater(eval1, eval2);
  eval::Mask::Ptr gt2 = Greater(eval1, true);   // bool  > true = false
  eval::Mask::Ptr gt3 = Greater(false, eval1);  // false > bool = false

  eval::Mask::Ptr lt1 = Less(eval1, eval2);
  eval::Mask::Ptr lt2 = Less(true, eval1);   // true < bool  = false
  eval::Mask::Ptr lt3 = Less(eval1, false);  // bool < false = false

  eval::Mask::Ptr ge1 = GEqual(eval1, eval2);
  eval::Mask::Ptr ge2 = GEqual(eval1, false);  // bool >= false = true
  eval::Mask::Ptr ge3 = GEqual(true, eval1);   // true >= bool  = true

  eval::Mask::Ptr le1 = LEqual(eval1, eval2);
  eval::Mask::Ptr le2 = LEqual(eval1, true);   // bool  <= true = true
  eval::Mask::Ptr le3 = LEqual(false, eval1);  // false <= bool = true

  // Ensure we can call setGraph on combined evaluators
  eq11->setGraph(graph_.get());
  eq22->setGraph(graph_.get());
  eq12->setGraph(graph_.get());
  eq21->setGraph(graph_.get());
  eq1c->setGraph(graph_.get());
  eqc1->setGraph(graph_.get());

  ne11->setGraph(graph_.get());
  ne22->setGraph(graph_.get());
  ne12->setGraph(graph_.get());
  ne21->setGraph(graph_.get());
  ne1c->setGraph(graph_.get());
  nec1->setGraph(graph_.get());

  gt1->setGraph(graph_.get());
  gt2->setGraph(graph_.get());
  gt3->setGraph(graph_.get());

  lt1->setGraph(graph_.get());
  lt2->setGraph(graph_.get());
  lt3->setGraph(graph_.get());

  ge1->setGraph(graph_.get());
  ge2->setGraph(graph_.get());
  ge3->setGraph(graph_.get());

  le1->setGraph(graph_.get());
  le2->setGraph(graph_.get());
  le3->setGraph(graph_.get());

  for (auto it = graph_->vertices()->begin(); it != graph_->vertices()->end();
       ++it) {
    auto res1 = eval1->at(it->first);
    auto res2 = eval2->at(it->first);

    EXPECT_TRUE(eq11->at(it->first));
    EXPECT_TRUE(eq22->at(it->first));
    EXPECT_EQ(eq12->at(it->first), (res1 == res2));
    EXPECT_EQ(eq21->at(it->first), (res1 == res2));
    EXPECT_EQ(eq1c->at(it->first), (res1 == cval));
    EXPECT_EQ(eqc1->at(it->first), (res1 == cval));

    EXPECT_TRUE((*eq11)[it->first]);
    EXPECT_TRUE((*eq22)[it->first]);
    EXPECT_EQ((*eq12)[it->first], (res1 == res2));
    EXPECT_EQ((*eq21)[it->first], (res1 == res2));
    EXPECT_EQ((*eq1c)[it->first], (res1 == cval));
    EXPECT_EQ((*eqc1)[it->first], (res1 == cval));

    EXPECT_TRUE(!ne11->at(it->first));
    EXPECT_TRUE(!ne22->at(it->first));
    EXPECT_EQ(ne12->at(it->first), (res1 != res2));
    EXPECT_EQ(ne21->at(it->first), (res1 != res2));
    EXPECT_EQ(ne1c->at(it->first), (res1 != cval));
    EXPECT_EQ(nec1->at(it->first), (res1 != cval));

    EXPECT_FALSE((*ne11)[it->first]);
    EXPECT_FALSE((*ne22)[it->first]);
    EXPECT_EQ((*ne12)[it->first], (res1 != res2));
    EXPECT_EQ((*ne21)[it->first], (res1 != res2));
    EXPECT_EQ((*ne1c)[it->first], (res1 != cval));
    EXPECT_EQ((*nec1)[it->first], (res1 != cval));

    EXPECT_EQ(gt1->at(it->first), (res1 > res2));
    EXPECT_FALSE(gt2->at(it->first));
    EXPECT_FALSE(gt3->at(it->first));

    EXPECT_EQ((*gt1)[it->first], (res1 > res2));
    EXPECT_FALSE((*gt2)[it->first]);
    EXPECT_FALSE((*gt3)[it->first]);

    EXPECT_EQ(lt1->at(it->first), (res1 < res2));
    EXPECT_FALSE(lt2->at(it->first));
    EXPECT_FALSE(lt3->at(it->first));

    EXPECT_EQ((*lt1)[it->first], (res1 < res2));
    EXPECT_FALSE((*lt2)[it->first]);
    EXPECT_FALSE((*lt3)[it->first]);

    EXPECT_EQ(ge1->at(it->first), (res1 >= res2));
    EXPECT_TRUE(ge2->at(it->first));
    EXPECT_TRUE(ge3->at(it->first));

    EXPECT_EQ((*ge1)[it->first], (res1 >= res2));
    EXPECT_TRUE((*ge2)[it->first]);
    EXPECT_TRUE((*ge3)[it->first]);

    EXPECT_EQ(le1->at(it->first), (res1 <= res2));
    EXPECT_TRUE(le2->at(it->first));
    EXPECT_TRUE(le3->at(it->first));

    EXPECT_EQ((*le1)[it->first], (res1 <= res2));
    EXPECT_TRUE((*le2)[it->first]);
    EXPECT_TRUE((*le3)[it->first]);
  }
  for (auto it = graph_->edges()->begin(); it != graph_->edges()->end(); ++it) {
    auto res1 = eval1->at(it->first);
    auto res2 = eval2->at(it->first);

    EXPECT_TRUE(eq11->at(it->first));
    EXPECT_TRUE(eq22->at(it->first));
    EXPECT_EQ(eq12->at(it->first), (res1 == res2));
    EXPECT_EQ(eq21->at(it->first), (res1 == res2));
    EXPECT_EQ(eq1c->at(it->first), (res1 == cval));
    EXPECT_EQ(eqc1->at(it->first), (res1 == cval));

    EXPECT_TRUE((*eq11)[it->first]);
    EXPECT_TRUE((*eq22)[it->first]);
    EXPECT_EQ((*eq12)[it->first], (res1 == res2));
    EXPECT_EQ((*eq21)[it->first], (res1 == res2));
    EXPECT_EQ((*eq1c)[it->first], (res1 == cval));
    EXPECT_EQ((*eqc1)[it->first], (res1 == cval));

    EXPECT_FALSE(ne11->at(it->first));
    EXPECT_FALSE(ne22->at(it->first));
    EXPECT_EQ(ne12->at(it->first), (res1 != res2));
    EXPECT_EQ(ne21->at(it->first), (res1 != res2));
    EXPECT_EQ(ne1c->at(it->first), (res1 != cval));
    EXPECT_EQ(nec1->at(it->first), (res1 != cval));

    EXPECT_FALSE((*ne11)[it->first]);
    EXPECT_FALSE((*ne22)[it->first]);
    EXPECT_EQ((*ne12)[it->first], (res1 != res2));
    EXPECT_EQ((*ne21)[it->first], (res1 != res2));
    EXPECT_EQ((*ne1c)[it->first], (res1 != cval));
    EXPECT_EQ((*nec1)[it->first], (res1 != cval));

    EXPECT_EQ(gt1->at(it->first), (res1 > res2));
    EXPECT_FALSE(gt2->at(it->first));
    EXPECT_FALSE(gt3->at(it->first));

    EXPECT_EQ((*gt1)[it->first], (res1 > res2));
    EXPECT_FALSE((*gt2)[it->first]);
    EXPECT_FALSE((*gt3)[it->first]);

    EXPECT_EQ(lt1->at(it->first), (res1 < res2));
    EXPECT_FALSE(lt2->at(it->first));
    EXPECT_FALSE(lt3->at(it->first));

    EXPECT_EQ((*lt1)[it->first], (res1 < res2));
    EXPECT_FALSE((*lt2)[it->first]);
    EXPECT_FALSE((*lt3)[it->first]);

    EXPECT_EQ(ge1->at(it->first), (res1 >= res2));
    EXPECT_TRUE(ge2->at(it->first));
    EXPECT_TRUE(ge3->at(it->first));

    EXPECT_EQ((*ge1)[it->first], (res1 >= res2));
    EXPECT_TRUE((*ge2)[it->first]);
    EXPECT_TRUE((*ge3)[it->first]);

    EXPECT_EQ(le1->at(it->first), (res1 <= res2));
    EXPECT_TRUE(le2->at(it->first));
    EXPECT_TRUE(le3->at(it->first));

    EXPECT_EQ((*le1)[it->first], (res1 <= res2));
    EXPECT_TRUE((*le2)[it->first]);
    EXPECT_TRUE((*le3)[it->first]);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
