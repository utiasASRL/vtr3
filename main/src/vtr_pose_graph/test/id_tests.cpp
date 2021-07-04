#include <gtest/gtest.h>

#include <iostream>

#include <vtr_logging/logging_init.hpp>
#include <vtr_pose_graph/id/graph_id.hpp>

TEST(PoseGraph, id) {
  using namespace vtr::pose_graph;

  VertexId vtmp1(2, 10);
  VertexId vtmp2(2, 11);
  VertexId vtmp3(2, 9);
  VertexId vtmp4(3, 9);
  VertexId vtmp5(1, 20);

  std::cout << std::endl << "Vertices" << std::endl;
  std::cout << vtmp1 << ", " << vtmp2 << ", " << vtmp3 << ", " << vtmp4 << ", "
            << vtmp5 << std::endl;

  EXPECT_TRUE(vtmp1 < vtmp2);
  EXPECT_FALSE(vtmp1 < vtmp3);
  EXPECT_TRUE(vtmp1 < vtmp4);
  EXPECT_FALSE(vtmp1 < vtmp5);

  vtmp5++;
  EXPECT_EQ(vtmp5.id(), VertexId::Base::BaseIdPairType(1, 21));

  ++vtmp5;
  EXPECT_EQ(vtmp5.id(), VertexId::Base::BaseIdPairType(1, 22));

  VertexId vtmp6(vtmp5);
  EXPECT_EQ(vtmp6.id(), VertexId::Base::BaseIdPairType(1, 22));

  VertexId vtmp7(++vtmp5);
  EXPECT_EQ(vtmp7.id(), VertexId::Base::BaseIdPairType(1, 23));

  vtmp2 = vtmp6;
  EXPECT_EQ(vtmp2.id(), VertexId::Base::BaseIdPairType(1, 22));

  EXPECT_FALSE(vtmp6 == vtmp5);

  EXPECT_EQ(vtmp1.majorId(), BaseIdType(2));
  EXPECT_EQ(vtmp1.minorId(), BaseIdType(10));

  EdgeId etmp1(2, 0, 10, EdgeId::Type::Spatial);
  EdgeId etmp2(2, 0, 11, EdgeId::Type::Spatial);
  EdgeId etmp3(2, 0, 9, EdgeId::Type::Spatial);
  EdgeId etmp4(3, 2, 9, EdgeId::Type::Spatial);
  EdgeId etmp5(1, 0, 20, EdgeId::Type::Spatial);
  EdgeId etmp6(2, 10, EdgeId::Type::Temporal);

  EdgeId::UnorderedSet tmp;
  tmp.insert(etmp1);

  std::cout << std::endl << "Edges" << std::endl;
  std::cout << etmp1 << ", " << etmp2 << ", " << etmp3 << ", " << etmp4 << ", "
            << etmp5 << ", " << etmp6 << std::endl;

  EXPECT_TRUE(etmp1 < etmp2);
  EXPECT_FALSE(etmp1 < etmp3);
  EXPECT_TRUE(etmp1 < etmp4);
  EXPECT_FALSE(etmp1 < etmp5);
  EXPECT_FALSE(etmp1 == etmp6);
  EXPECT_TRUE(etmp6 < etmp2);
  EXPECT_TRUE(etmp6 < etmp3);
  EXPECT_TRUE(etmp6 < etmp4);
  EXPECT_TRUE(etmp6 < etmp5);

  etmp5++;
  EXPECT_EQ(etmp5.majorId1(), BaseIdType(0));
  EXPECT_EQ(etmp5.minorId1(), BaseIdType(0));
  EXPECT_EQ(etmp5.majorId2(), BaseIdType(1));
  EXPECT_EQ(etmp5.minorId2(), BaseIdType(21));

  ++etmp5;
  EXPECT_EQ(etmp5.majorId1(), BaseIdType(0));
  EXPECT_EQ(etmp5.minorId1(), BaseIdType(1));
  EXPECT_EQ(etmp5.majorId2(), BaseIdType(1));
  EXPECT_EQ(etmp5.minorId2(), BaseIdType(21));

  EdgeId etmp7(etmp5);
  EXPECT_EQ(etmp7.majorId1(), BaseIdType(0));
  EXPECT_EQ(etmp7.minorId1(), BaseIdType(1));
  EXPECT_EQ(etmp7.majorId2(), BaseIdType(1));
  EXPECT_EQ(etmp7.minorId2(), BaseIdType(21));

  EdgeId etmp8(++etmp5);
  EXPECT_EQ(etmp5.majorId1(), BaseIdType(0));
  EXPECT_EQ(etmp5.minorId1(), BaseIdType(2));
  EXPECT_EQ(etmp5.majorId2(), BaseIdType(1));
  EXPECT_EQ(etmp5.minorId2(), BaseIdType(21));

  etmp2 = etmp7;
  EXPECT_EQ(etmp2.majorId1(), BaseIdType(0));
  EXPECT_EQ(etmp2.minorId1(), BaseIdType(1));
  EXPECT_EQ(etmp2.majorId2(), BaseIdType(1));
  EXPECT_EQ(etmp2.minorId2(), BaseIdType(21));

  EXPECT_FALSE(etmp7 == etmp5);

  /// \todo (yuchen) Test EdgeId majorId and minorId method.
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}