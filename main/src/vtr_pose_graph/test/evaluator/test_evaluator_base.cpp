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
 * \file evaluator_tests.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include <random>

#include "vtr_logging/logging_init.hpp"
#include "vtr_pose_graph/evaluator/evaluators.hpp"
#include "vtr_pose_graph/index/graph.hpp"

using namespace ::testing;
using namespace vtr::logging;
using namespace vtr::pose_graph;

using IntRandType =
    decltype(std::bind(std::uniform_int_distribution<int64_t>{0, 1000},
                       std::mt19937(std::random_device{}())));
using DoubleRandType =
    decltype(std::bind(std::uniform_real_distribution<double>{0.f, 100.f},
                       std::mt19937(std::random_device{}())));

class EvaluatorTestFixture : public Test {
 protected:
  EvaluatorTestFixture()
      : irnd_(std::bind(std::uniform_int_distribution<int64_t>{0, 1000},
                        std::mt19937(std::random_device{}()))),
        drnd_(std::bind(std::uniform_real_distribution<double>{0.f, 100.f},
                        std::mt19937(std::random_device{}()))) {}

  ~EvaluatorTestFixture() override {}

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

    // clang-format off
    for (int idx = 0; idx < 5; ++idx) {
      graph_->addRun();
      graph_->addVertex();
      graph_->addVertex();
      graph_->addVertex();
      graph_->addEdge(VertexId(idx, 0), VertexId(idx, 1), EdgeType::Temporal, false, EdgeTransform(true));
      graph_->addEdge(VertexId(idx, 1), VertexId(idx, 2), EdgeType::Temporal, false, EdgeTransform(true));
    }
    graph_->addEdge(VertexId(1, 1), VertexId(0, 0), EdgeType::Spatial, false, EdgeTransform(true));
    graph_->addEdge(VertexId(2, 2), VertexId(1, 2), EdgeType::Spatial, false, EdgeTransform(true));
    graph_->addEdge(VertexId(3, 1), VertexId(2, 1), EdgeType::Spatial, false, EdgeTransform(true));
    graph_->addEdge(VertexId(4, 2), VertexId(3, 2), EdgeType::Spatial, false, EdgeTransform(true));
    // clang-format on
  }

  void TearDown() override {}

  VertexId rndVertexId() { return VertexId(irnd_(), irnd_()); }
  EdgeId rndEdgeId() { return EdgeId(rndVertexId(), rndVertexId()); }

  BasicGraph::Ptr graph_ = std::make_shared<BasicGraph>();
  IntRandType irnd_;
  DoubleRandType drnd_;
};

// clang-format off

TEST_F(EvaluatorTestFixture, WeightConstant) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100},
                        std::mt19937(std::random_device{}()));

  double eweight = drnd(), vweight = drnd();
  auto eval = std::make_shared<eval::weight::ConstEval>(eweight, vweight);

  EXPECT_EQ((*eval)[rndEdgeId()], eweight);
  EXPECT_EQ((*eval)[rndEdgeId()], eweight);
  EXPECT_EQ((*eval)[rndEdgeId()], eweight);
  EXPECT_EQ((*eval)[rndVertexId()], vweight);
}

TEST_F(EvaluatorTestFixture, WeightMap) {
  auto eval = std::make_shared<eval::weight::MapEval>();

  // Assign weights based on a pattern
  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    eval->ref(it->id()) = it->id().minorId() + 10.f * it->id().majorId();
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    eval->ref(it->id()) = it->id().minorId1() + it->id().minorId2() + 10.f * it->id().majorId2();
  }

  // Check using both GraphId and SimpleId
  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    EXPECT_EQ((*eval)[it->id()], it->id().minorId() + 10.f * it->id().majorId());
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    EXPECT_EQ((*eval)[it->id()], (it->id().minorId1() + it->id().minorId2() + 10.f * it->id().majorId2()));
  }

  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    eval->ref(it->id()) = 17.f * (it->id().minorId() + 10.f * it->id().majorId());
  }

  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    eval->ref(it->id()) = 17.f * (it->id().minorId1() + it->id().minorId2() + 10.f * it->id().majorId2());
  }

  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    EXPECT_EQ((*eval)[it->id()], (17.f * (it->id().minorId() + 10.f * it->id().majorId())));
  }

  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    EXPECT_EQ((*eval)[it->id()], (17.f * (it->id().minorId1() + it->id().minorId2() + 10.f * it->id().majorId2())));
  }
}

TEST_F(EvaluatorTestFixture, WeightBasicMath) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100}, std::mt19937(std::random_device{}()));

  auto eval = std::make_shared<eval::weight::MapEval>();

  // Assign some random weights
  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it)
    eval->ref(it->id()) = drnd();

  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it)
    eval->ref(it->id()) = drnd();

  double scale = (drnd() - 50.f) / 25.f;  // Scale factor in the range [-2,2]
  double offset = (drnd() - 50.f) / 5.f;  // Additive offset in the range [-10, 10]

  eval::weight::Ptr a1 = Add(eval, offset);
  eval::weight::Ptr a2 = Add(offset, eval);
  eval::weight::Ptr a3 = Add(eval, eval);

  eval::weight::Ptr s1 = Sub(eval, offset);
  eval::weight::Ptr s2 = Sub(offset, eval);
  eval::weight::Ptr s3 = Sub(eval, eval);

  eval::weight::Ptr m1 = Mul(eval, scale);
  eval::weight::Ptr m2 = Mul(scale, eval);
  eval::weight::Ptr m3 = Mul(eval, eval);

  eval::weight::Ptr d1 = Div(eval, scale);
  eval::weight::Ptr d2 = Div(scale, eval);
  eval::weight::Ptr d3 = Div(eval, eval);

  eval::weight::Ptr n1 = Neg(eval);

  // One complicated test to see if it breaks: -(a + x) / (a - x) * x
  eval::weight::Ptr QQ = Mul(Neg(Div(Add(offset, eval), Sub(offset, eval))), eval);

  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    auto res = (*eval)[it->id()];

    EXPECT_NEAR((*a1)[it->id()], res + offset, 1e-10);
    EXPECT_NEAR((*a2)[it->id()], offset + res, 1e-10);
    EXPECT_NEAR((*a3)[it->id()], res + res, 1e-10);

    EXPECT_NEAR((*s1)[it->id()], res - offset, 1e-10);
    EXPECT_NEAR((*s2)[it->id()], offset - res, 1e-10);
    EXPECT_NEAR((*s3)[it->id()], res - res, 1e-10);

    EXPECT_NEAR((*m1)[it->id()], res * scale, 1e-10);
    EXPECT_NEAR((*m2)[it->id()], scale * res, 1e-10);
    EXPECT_NEAR((*m3)[it->id()], res * res, 1e-10);

    EXPECT_NEAR((*d1)[it->id()], res / scale, 1e-10);

    // Division special case: we can't divide by zero
    if (res != 0.f) {
      EXPECT_NEAR((*d2)[it->id()], scale / res, 1e-10);
      EXPECT_NEAR((*d3)[it->id()], 1.f, 1e-10);
    }
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    auto res = (*eval)[it->id()];

    EXPECT_NEAR((*a1)[it->id()], res + offset, 1e-10);
    EXPECT_NEAR((*a2)[it->id()], offset + res, 1e-10);
    EXPECT_NEAR((*a3)[it->id()], res + res, 1e-10);

    EXPECT_NEAR((*s1)[it->id()], res - offset, 1e-10);
    EXPECT_NEAR((*s2)[it->id()], offset - res, 1e-10);
    EXPECT_NEAR((*s3)[it->id()], res - res, 1e-10);

    EXPECT_NEAR((*m1)[it->id()], res * scale, 1e-10);
    EXPECT_NEAR((*m2)[it->id()], scale * res, 1e-10);
    EXPECT_NEAR((*m3)[it->id()], res * res, 1e-10);

    EXPECT_NEAR((*d1)[it->id()], res / scale, 1e-10);

    // Division special case: we can't divide by zero
    if (res != 0.f) {
      EXPECT_NEAR((*d2)[it->id()], scale / res, 1e-10);
      EXPECT_NEAR((*d3)[it->id()], 1.f, 1e-10);
    }
  }
}

TEST_F(EvaluatorTestFixture, WeightScalarMath) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100}, std::mt19937(std::random_device{}()));

  auto eval = std::make_shared<eval::weight::MapEval>();

  // Assign some random weights
  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it)
    eval->ref(it->id()) = (drnd() - 50.f) / 10.f;
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it)
    eval->ref(it->id()) = (drnd() - 50.f) / 10.f;

  double exponent = drnd() / 20.f;  // Exponent in the range [0,5]

  eval::weight::Ptr abs = Abs(eval);
  eval::weight::Ptr exp = Exp(eval);
  eval::weight::Ptr log = Log(abs);  // We need to keep this positive

  eval::weight::Ptr sgm = Sgm(eval);
  eval::weight::Ptr erf = Erf(eval);
  eval::weight::Ptr erfc = Erfc(eval);

  eval::weight::Ptr pow1 = Pow(abs, exponent);
  eval::weight::Ptr pow2 = Pow(exponent, eval);
  eval::weight::Ptr pow3 = Pow(abs, eval);

  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    auto res = (*eval)[it->id()];

    EXPECT_NEAR((*abs)[it->id()], std::abs(res), 1e-10);
    EXPECT_NEAR((*exp)[it->id()], std::exp(res), 1e-10);
    EXPECT_NEAR((*log)[it->id()], std::log(std::abs(res)), 1e-10);
    EXPECT_NEAR((*sgm)[it->id()], 1.f / (1.f + std::exp(-res)), 1e-10);
    EXPECT_NEAR((*erf)[it->id()], std::erf(res), 1e-10);
    EXPECT_NEAR((*erfc)[it->id()], std::erfc(res), 1e-10);

    EXPECT_NEAR((*pow1)[it->id()], std::pow(std::abs(res), exponent), 1e-10);
    EXPECT_NEAR((*pow2)[it->id()], std::pow(exponent, res), 1e-10);
    EXPECT_NEAR((*pow3)[it->id()], std::pow(std::abs(res), res), 1e-10);
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    auto res = (*eval)[it->id()];

    EXPECT_NEAR((*abs)[it->id()], std::abs(res), 1e-10);
    EXPECT_NEAR((*exp)[it->id()], std::exp(res), 1e-10);
    EXPECT_NEAR((*log)[it->id()], std::log(std::abs(res)), 1e-10);
    EXPECT_NEAR((*sgm)[it->id()], 1.f / (1.f + std::exp(-res)), 1e-10);
    EXPECT_NEAR((*erf)[it->id()], std::erf(res), 1e-10);
    EXPECT_NEAR((*erfc)[it->id()], std::erfc(res), 1e-10);

    EXPECT_NEAR((*pow1)[it->id()], std::pow(std::abs(res), exponent), 1e-10);
    EXPECT_NEAR((*pow2)[it->id()], std::pow(exponent, res), 1e-10);
    EXPECT_NEAR((*pow3)[it->id()], std::pow(std::abs(res), res), 1e-10);
  }
}

TEST_F(EvaluatorTestFixture, WeightComparison) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100}, std::mt19937(std::random_device{}()));

  auto eval1 = std::make_shared<eval::weight::MapEval>();  // [-1, 1]
  auto eval2 = std::make_shared<eval::weight::MapEval>();  // [-1, 1]

  // Assign some random weights in the range [-pi, pi]
  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    eval1->ref(it->id()) = (drnd() - 50.f) / 50.f;
    eval2->ref(it->id()) = (drnd() - 50.f) / 50.f;
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    eval1->ref(it->id()) = (drnd() - 50.f) / 50.f;
    eval2->ref(it->id()) = (drnd() - 50.f) / 50.f;
  }

  double v11 = (*eval1)[graph_->beginVertex()->id()];
  double e11 = (*eval1)[graph_->beginEdge()->id()];

  eval::mask::Ptr eq11 = Equal(eval1, eval1);
  eval::mask::Ptr eq22 = Equal(eval2, eval2);
  eval::mask::Ptr eq12 = Equal(eval1, eval2);
  eval::mask::Ptr eq21 = Equal(eval2, eval1);
  eval::mask::Ptr eq1cv = Equal(eval1, v11);
  eval::mask::Ptr eqc1v = Equal(v11, eval1);
  eval::mask::Ptr eq1ce = Equal(eval1, e11);
  eval::mask::Ptr eqc1e = Equal(e11, eval1);

  eval::mask::Ptr ne11 = NEqual(eval1, eval1);
  eval::mask::Ptr ne22 = NEqual(eval2, eval2);
  eval::mask::Ptr ne12 = NEqual(eval1, eval2);
  eval::mask::Ptr ne21 = NEqual(eval2, eval1);
  eval::mask::Ptr ne1cv = NEqual(eval1, v11);
  eval::mask::Ptr nec1v = NEqual(v11, eval1);
  eval::mask::Ptr ne1ce = NEqual(eval1, e11);
  eval::mask::Ptr nec1e = NEqual(e11, eval1);

  eval::mask::Ptr gt1 = Greater(eval1, eval2);
  eval::mask::Ptr gt2 = Greater(eval1, 0);
  eval::mask::Ptr gt3 = Greater(0, eval1);

  eval::mask::Ptr lt1 = Less(eval1, eval2);
  eval::mask::Ptr lt2 = Less(eval1, 0);
  eval::mask::Ptr lt3 = Less(0, eval1);

  eval::mask::Ptr ge1 = GEqual(eval1, eval2);
  eval::mask::Ptr ge2 = GEqual(eval1, 0);
  eval::mask::Ptr ge3 = GEqual(0, eval1);

  eval::mask::Ptr le1 = LEqual(eval1, eval2);
  eval::mask::Ptr le2 = LEqual(eval1, 0);
  eval::mask::Ptr le3 = LEqual(0, eval1);

  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    auto res1 = (*eval1)[it->id()];
    auto res2 = (*eval2)[it->id()];

    EXPECT_TRUE((*eq11)[it->id()]);
    EXPECT_TRUE((*eq22)[it->id()]);
    EXPECT_EQ((*eq12)[it->id()], (res1 == res2));
    EXPECT_EQ((*eq21)[it->id()], (res1 == res2));
    EXPECT_EQ((*eq1cv)[it->id()], (res1 == v11));
    EXPECT_EQ((*eqc1v)[it->id()], (res1 == v11));

    EXPECT_TRUE(!(*ne11)[it->id()]);
    EXPECT_TRUE(!(*ne22)[it->id()]);
    EXPECT_EQ((*ne12)[it->id()], (res1 != res2));
    EXPECT_EQ((*ne21)[it->id()], (res1 != res2));
    EXPECT_EQ((*ne1cv)[it->id()], (res1 != v11));
    EXPECT_EQ((*nec1v)[it->id()], (res1 != v11));

    EXPECT_EQ((*gt1)[it->id()], (res1 > res2));
    EXPECT_EQ((*gt2)[it->id()], (res1 > 0));
    EXPECT_EQ((*gt3)[it->id()], (0 > res1));

    EXPECT_EQ((*lt1)[it->id()], (res1 < res2));
    EXPECT_EQ((*lt2)[it->id()], (res1 < 0));
    EXPECT_EQ((*lt3)[it->id()], (0 < res1));

    EXPECT_EQ((*ge1)[it->id()], (res1 >= res2));
    EXPECT_EQ((*ge2)[it->id()], (res1 >= 0));
    EXPECT_EQ((*ge3)[it->id()], (0 >= res1));

    EXPECT_EQ((*le1)[it->id()], (res1 <= res2));
    EXPECT_EQ((*le2)[it->id()], (res1 <= 0));
    EXPECT_EQ((*le3)[it->id()], (0 <= res1));
  }

  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    auto res1 = (*eval1)[it->id()];
    auto res2 = (*eval2)[it->id()];

    EXPECT_TRUE((*eq11)[it->id()]);
    EXPECT_TRUE((*eq22)[it->id()]);
    EXPECT_EQ((*eq12)[it->id()], (res1 == res2));
    EXPECT_EQ((*eq21)[it->id()], (res1 == res2));
    EXPECT_EQ((*eq1ce)[it->id()], (res1 == e11));
    EXPECT_EQ((*eqc1e)[it->id()], (res1 == e11));

    EXPECT_TRUE(!(*ne11)[it->id()]);
    EXPECT_TRUE(!(*ne22)[it->id()]);
    EXPECT_EQ((*ne12)[it->id()], (res1 != res2));
    EXPECT_EQ((*ne21)[it->id()], (res1 != res2));
    EXPECT_EQ((*ne1ce)[it->id()], (res1 != e11));
    EXPECT_EQ((*nec1e)[it->id()], (res1 != e11));

    EXPECT_EQ((*gt1)[it->id()], (res1 > res2));
    EXPECT_EQ((*gt2)[it->id()], (res1 > 0));
    EXPECT_EQ((*gt3)[it->id()], (0 > res1));

    EXPECT_EQ((*lt1)[it->id()], (res1 < res2));
    EXPECT_EQ((*lt2)[it->id()], (res1 < 0));
    EXPECT_EQ((*lt3)[it->id()], (0 < res1));

    EXPECT_EQ((*ge1)[it->id()], (res1 >= res2));
    EXPECT_EQ((*ge2)[it->id()], (res1 >= 0));
    EXPECT_EQ((*ge3)[it->id()], (0 >= res1));

    EXPECT_EQ((*le1)[it->id()], (res1 <= res2));
    EXPECT_EQ((*le2)[it->id()], (res1 <= 0));
    EXPECT_EQ((*le3)[it->id()], (0 <= res1));
  }
}

TEST_F(EvaluatorTestFixture, WeightTrigonometry) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100}, std::mt19937(std::random_device{}()));

  auto eval = std::make_shared<eval::weight::MapEval>();
  auto inveval = std::make_shared<eval::weight::MapEval>();

  // Assign some random weights in the range [-pi, pi]
  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    eval->ref(it->id()) = (drnd() - 50.f) / 50.f * M_PI;
    inveval->ref(it->id()) = (drnd() - 50.f) / 50.f;
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    eval->ref(it->id()) = (drnd() - 50.f) / 50.f * M_PI;
    inveval->ref(it->id()) = (drnd() - 50.f) / 50.f;
  }

  eval::weight::Ptr sin = Sin(eval);
  eval::weight::Ptr cos = Cos(eval);
  eval::weight::Ptr tan = Tan(eval);

  eval::weight::Ptr asin = ASin(inveval);
  eval::weight::Ptr acos = ACos(inveval);
  eval::weight::Ptr atan = ATan(inveval);

  eval::weight::Ptr atan2 = ATan2(Div(eval, M_PI), inveval);

  for (auto it = graph_->beginVertex(); it != graph_->endVertex();
       ++it) {
    auto res = (*eval)[it->id()];
    auto invres = (*inveval)[it->id()];

    EXPECT_EQ((*sin)[it->id()], std::sin(res));
    EXPECT_EQ((*cos)[it->id()], std::cos(res));
    EXPECT_EQ((*tan)[it->id()], std::tan(res));
    EXPECT_EQ((*asin)[it->id()], std::asin(invres));
    EXPECT_EQ((*acos)[it->id()], std::acos(invres));
    EXPECT_EQ((*atan)[it->id()], std::atan(invres));

    EXPECT_EQ((*atan2)[it->id()], std::atan2(res / M_PI, invres));
  }

  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    auto res = (*eval)[it->id()];
    auto invres = (*inveval)[it->id()];

    EXPECT_EQ((*sin)[it->id()], std::sin(res));
    EXPECT_EQ((*cos)[it->id()], std::cos(res));
    EXPECT_EQ((*tan)[it->id()], std::tan(res));
    EXPECT_EQ((*asin)[it->id()], std::asin(invres));
    EXPECT_EQ((*acos)[it->id()], std::acos(invres));
    EXPECT_EQ((*atan)[it->id()], std::atan(invres));

    EXPECT_EQ((*atan2)[it->id()], std::atan2(res / M_PI, invres));
  }
}

TEST_F(EvaluatorTestFixture, WeightHyperbolicTrigonometry) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100}, std::mt19937(std::random_device{}()));

  auto eval1 = std::make_shared<eval::weight::MapEval>();  // [-50, 50]
  auto eval2 = std::make_shared<eval::weight::MapEval>();  // [1, 50]
  auto eval3 = std::make_shared<eval::weight::MapEval>();  // [-1, 1]

  // Assign some random weights in the range [-pi, pi]
  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    eval1->ref(it->id()) = (drnd() - 50.f);
    eval2->ref(it->id()) = (drnd()) / 2.0 + 1.f;
    eval3->ref(it->id()) = (drnd() - 50.f) / 50.f;
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    eval1->ref(it->id()) = (drnd() - 50.f);
    eval2->ref(it->id()) = (drnd()) / 2.0 + 1.f;
    eval3->ref(it->id()) = (drnd() - 50.f) / 50.f;
  }

  eval::weight::Ptr sinh = Sinh(eval1);
  eval::weight::Ptr cosh = Cosh(eval1);
  eval::weight::Ptr tanh = Tanh(eval1);

  eval::weight::Ptr asinh = ASinh(eval1);
  eval::weight::Ptr acosh = ACosh(eval2);
  eval::weight::Ptr atanh = ATanh(eval3);

  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    auto res1 = (*eval1)[it->id()];
    auto res2 = (*eval2)[it->id()];
    auto res3 = (*eval3)[it->id()];

    EXPECT_EQ((*sinh)[it->id()], std::sinh(res1));
    EXPECT_EQ((*cosh)[it->id()], std::cosh(res1));
    EXPECT_EQ((*tanh)[it->id()], std::tanh(res1));
    EXPECT_EQ((*asinh)[it->id()], std::asinh(res1));
    EXPECT_EQ((*acosh)[it->id()], std::acosh(res2));
    EXPECT_EQ((*atanh)[it->id()], std::atanh(res3));
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    auto res1 = (*eval1)[it->id()];
    auto res2 = (*eval2)[it->id()];
    auto res3 = (*eval3)[it->id()];

    EXPECT_EQ((*sinh)[it->id()], std::sinh(res1));
    EXPECT_EQ((*cosh)[it->id()], std::cosh(res1));
    EXPECT_EQ((*tanh)[it->id()], std::tanh(res1));
    EXPECT_EQ((*asinh)[it->id()], std::asinh(res1));
    EXPECT_EQ((*acosh)[it->id()], std::acosh(res2));
    EXPECT_EQ((*atanh)[it->id()], std::atanh(res3));
  }
}

TEST_F(EvaluatorTestFixture, MaskConstant) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100}, std::mt19937(std::random_device{}()));

  bool vmask = drnd() > 50.0, emask = drnd() > 50.0;
  auto eval = std::make_shared<eval::mask::ConstEval>(emask, vmask);

  EXPECT_EQ((*eval)[rndEdgeId()], emask);
  EXPECT_EQ((*eval)[rndEdgeId()], emask);
  EXPECT_EQ((*eval)[rndEdgeId()], emask);
  EXPECT_EQ((*eval)[rndVertexId()], vmask);
}

TEST_F(EvaluatorTestFixture, MaskMap) {
  auto eval = std::make_shared<eval::mask::MapEval>();

  // Assign masks based on a pattern: true if sum of minor and major ids is
  // EVEN
  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    eval->ref(it->id()) = (it->id().minorId() + it->id().majorId()) % 2 == 0;
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    eval->ref(it->id()) = (((it->id().minorId1() + it->id().minorId2() + it->id().majorId2()) % 2) == 0);
  }

  // Check using both GraphId and SimpleId
  for (auto it = graph_->beginVertex(); it != graph_->endVertex();
       ++it) {
    EXPECT_EQ((*eval)[it->id()],((it->id().minorId() + it->id().majorId()) % 2 == 0));
    EXPECT_EQ((*eval)[it->id()],((it->id().minorId() + it->id().majorId()) % 2 == 0));
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    EXPECT_EQ((*eval)[it->id()], ((it->id().minorId1() + it->id().minorId2() + it->id().majorId2()) % 2 == 0));
    EXPECT_EQ((*eval)[it->id()], ((it->id().minorId1() + it->id().minorId2() + it->id().majorId2()) % 2 == 0));
  }

  // Assign masks based on a pattern, using VertexId/EdgeId: true if sum of
  // minor and major ids is ODD
  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    eval->ref(it->id()) = (it->id().minorId() + it->id().majorId()) % 2 != 0;
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    eval->ref(it->id()) = ((it->id().minorId1() + it->id().minorId2() + it->id().majorId2()) % 2) != 0;
  }

  // Check using both GraphId and SimpleId
  for (auto it = graph_->beginVertex(); it != graph_->endVertex();
       ++it) {
    EXPECT_EQ( (*eval)[it->id()], ((it->id().minorId() + it->id().majorId()) % 2 != 0));
    EXPECT_EQ( (*eval)[it->id()], ((it->id().minorId() + it->id().majorId()) % 2 != 0));
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    EXPECT_EQ((*eval)[it->id()], (((it->id().minorId1() + it->id().minorId2() + it->id().majorId2()) % 2) != 0));
    EXPECT_EQ((*eval)[it->id()], (((it->id().minorId1() + it->id().minorId2() + it->id().majorId2()) % 2) != 0));
  }
}

TEST_F(EvaluatorTestFixture, MaskBooleanLogic) {
  auto drnd = std::bind(std::uniform_real_distribution<double>{0, 100},
                        std::mt19937(std::random_device{}()));

  auto eval1 = std::make_shared<eval::mask::MapEval>();  // 1:1 true:false
  auto eval2 = std::make_shared<eval::mask::MapEval>();  // 1:1 true:false

  // Assign some random weights in the range [-pi, pi]
  for (auto it = graph_->beginVertex(); it != graph_->endVertex(); ++it) {
    eval1->ref(it->id()) = drnd() > 50.f;
    eval2->ref(it->id()) = drnd() > 50.f;
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    eval1->ref(it->id()) = drnd() > 50.f;
    eval2->ref(it->id()) = drnd() > 50.f;
  }

  eval::mask::Ptr and11 = And(eval1, eval1);  // A & A = A
  eval::mask::Ptr and22 = And(eval2, eval2);  // B & B = B
  eval::mask::Ptr and12 = And(eval1, eval2);  // A & B = A & B
  eval::mask::Ptr and21 = And(eval2, eval1);  // B & A = B & A = A & B
  eval::mask::Ptr and1t = And(eval1, true);   // A & 1 = A
  eval::mask::Ptr andt1 = And(true, eval1);   // 1 & A = A
  eval::mask::Ptr and1f = And(eval1, false);  // A & 0 = 0
  eval::mask::Ptr andf1 = And(false, eval1);  // 0 & A = 0

  eval::mask::Ptr or11 = Or(eval1, eval1);  // A | A = A
  eval::mask::Ptr or22 = Or(eval2, eval2);  // B | B = B
  eval::mask::Ptr or12 = Or(eval1, eval2);  // A | B = A | B
  eval::mask::Ptr or21 = Or(eval2, eval1);  // B | A = B | A = A | B
  eval::mask::Ptr or1t = Or(eval1, true);   // A | 1 = 1
  eval::mask::Ptr ort1 = Or(true, eval1);   // 1 | A = 1
  eval::mask::Ptr or1f = Or(eval1, false);  // A | 0 = A
  eval::mask::Ptr orf1 = Or(false, eval1);  // 0 | A = A

  eval::mask::Ptr xor11 = Xor(eval1, eval1);  // A ^ A = 0
  eval::mask::Ptr xor22 = Xor(eval2, eval2);  // B ^ B = 0
  eval::mask::Ptr xor12 = Xor(eval1, eval2);  // A ^ B = A ^ B
  eval::mask::Ptr xor21 = Xor(eval2, eval1);  // B ^ A = B ^ A = A ^ B
  eval::mask::Ptr xor1t = Xor(eval1, true);   // A ^ 1 = ~A
  eval::mask::Ptr xort1 = Xor(true, eval1);   // 1 ^ A = ~A
  eval::mask::Ptr xor1f = Xor(eval1, false);  // A ^ 0 = A
  eval::mask::Ptr xorf1 = Xor(false, eval1);  // 0 ^ A = A

  eval::mask::Ptr not1 = Not(eval1);  // ~A = ~A

  for (auto it = graph_->beginVertex(); it != graph_->endVertex();
       ++it) {
    auto res1 = (*eval1)[it->id()];
    auto res2 = (*eval2)[it->id()];

    EXPECT_EQ((*and11)[it->id()], res1);
    EXPECT_EQ((*and22)[it->id()], res2);
    EXPECT_EQ((*and12)[it->id()], (res1 && res2));
    EXPECT_EQ((*and21)[it->id()], (res1 && res2));
    EXPECT_EQ((*and1t)[it->id()], res1);
    EXPECT_EQ((*andt1)[it->id()], res1);
    EXPECT_FALSE((*and1f)[it->id()]);
    EXPECT_FALSE((*andf1)[it->id()]);

    EXPECT_EQ((*or11)[it->id()], res1);
    EXPECT_EQ((*or22)[it->id()], res2);
    EXPECT_EQ((*or12)[it->id()], (res1 || res2));
    EXPECT_EQ((*or21)[it->id()], (res1 || res2));
    EXPECT_TRUE((*or1t)[it->id()]);
    EXPECT_TRUE((*ort1)[it->id()]);
    EXPECT_EQ((*or1f)[it->id()], res1);
    EXPECT_EQ((*orf1)[it->id()], res1);

    EXPECT_TRUE(!(*xor11)[it->id()]);
    EXPECT_TRUE(!(*xor22)[it->id()]);
    EXPECT_EQ((*xor12)[it->id()], (res1 != res2));
    EXPECT_EQ((*xor21)[it->id()], (res1 != res2));
    EXPECT_NE((*xor1t)[it->id()], res1);
    EXPECT_NE((*xort1)[it->id()], res1);
    EXPECT_EQ((*xor1f)[it->id()], res1);
    EXPECT_EQ((*xorf1)[it->id()], res1);

    EXPECT_NE((*not1)[it->id()], res1);
  }
  for (auto it = graph_->beginEdge(); it != graph_->endEdge(); ++it) {
    auto res1 = (*eval1)[it->id()];
    auto res2 = (*eval2)[it->id()];

    EXPECT_EQ((*and11)[it->id()], res1);
    EXPECT_EQ((*and22)[it->id()], res2);
    EXPECT_EQ((*and12)[it->id()], (res1 && res2));
    EXPECT_EQ((*and21)[it->id()], (res1 && res2));
    EXPECT_EQ((*and1t)[it->id()], res1);
    EXPECT_EQ((*andt1)[it->id()], res1);
    EXPECT_FALSE((*and1f)[it->id()]);
    EXPECT_FALSE((*andf1)[it->id()]);

    EXPECT_EQ((*or11)[it->id()], res1);
    EXPECT_EQ((*or22)[it->id()], res2);
    EXPECT_EQ((*or12)[it->id()], (res1 || res2));
    EXPECT_EQ((*or21)[it->id()], (res1 || res2));
    EXPECT_TRUE((*or1t)[it->id()]);
    EXPECT_TRUE((*ort1)[it->id()]);
    EXPECT_EQ((*or1f)[it->id()], res1);
    EXPECT_EQ((*orf1)[it->id()], res1);

    EXPECT_FALSE((*xor11)[it->id()]);
    EXPECT_FALSE((*xor22)[it->id()]);
    EXPECT_EQ((*xor12)[it->id()], (res1 != res2));
    EXPECT_EQ((*xor21)[it->id()], (res1 != res2));
    EXPECT_NE((*xor1t)[it->id()], res1);
    EXPECT_NE((*xort1)[it->id()], res1);
    EXPECT_EQ((*xor1f)[it->id()], res1);
    EXPECT_EQ((*xorf1)[it->id()], res1);

    EXPECT_NE((*not1)[it->id()], res1);
  }
}
// clang-format on

int main(int argc, char** argv) {
  configureLogging("", true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
