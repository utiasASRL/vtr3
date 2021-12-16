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
 * \file test_query_cache.cpp
 * \author Yuchen Wu, Autonomous Space Robotics Lab (ASRL)
 */
#include <gtest/gtest.h>

#include "vtr_tactic/cache.hpp"

using namespace vtr;
using namespace vtr::tactic;

TEST(QueryCache, query_cache_existence) {
  auto qdata = std::make_shared<QueryCache>();

  // existence check
  EXPECT_FALSE(qdata->stamp.valid());
  EXPECT_FALSE((bool)qdata->stamp);

  qdata->stamp.emplace(1);
  EXPECT_TRUE(qdata->stamp.valid());
  EXPECT_TRUE((bool)qdata->stamp);

  qdata->stamp.clear();
  EXPECT_FALSE(qdata->stamp.valid());
  EXPECT_FALSE((bool)qdata->stamp);
}

TEST(QueryCache, query_cache_modifier) {
  auto qdata = std::make_shared<QueryCache>();

  // emplace construct
  qdata->stamp.emplace(1);
  EXPECT_EQ(*qdata->stamp, 1);

  auto stamp = std::make_shared<storage::Timestamp>(2);

  // copy construct
  qdata->stamp = *stamp;
  EXPECT_NE(qdata->stamp.ptr(), stamp);  // different shared pointer
  EXPECT_EQ(*qdata->stamp, *stamp);
  (*stamp)++;                            // increment stamp to 3
  EXPECT_EQ(*qdata->stamp, *stamp - 1);  // qdata->stamp stays the same

  // pointer construct
  qdata->stamp = stamp;
  EXPECT_EQ(qdata->stamp.ptr(), stamp);  // same shared pointer
  EXPECT_EQ(*qdata->stamp, *stamp);      // same value
  (*stamp)++;                            // increment stamp to 4
  EXPECT_EQ(*qdata->stamp, *stamp);
}

TEST(QueryCache, query_cache_copy_move) {
  auto qdata = std::make_shared<QueryCache>();
  auto stamp = std::make_shared<storage::Timestamp>(1);

  // pointer construct
  qdata->stamp = stamp;

  // copy qdata
  // (all caches are copied, which contains shared pointers to the actual data)
  auto qdata2 = std::make_shared<QueryCache>(*qdata);
  EXPECT_EQ(qdata->stamp.ptr(), stamp);
  EXPECT_EQ(qdata2->stamp.ptr(), stamp);

  // changes will be in sync
  (*stamp)++;
  EXPECT_EQ(*qdata->stamp, *stamp);
  EXPECT_EQ(*qdata2->stamp, *stamp);

  auto stamp2 = std::make_shared<storage::Timestamp>(10);
  qdata2->stamp = *stamp2;  // changes the value of the shared pointer
  EXPECT_EQ(*stamp2, *stamp);
  EXPECT_EQ(*qdata->stamp, *stamp);
  EXPECT_EQ(*qdata2->stamp, *stamp);

  // copy the shared pointer, now qdata and qdata2 no longer shares the same
  // shared pointer
  qdata2->stamp = stamp2;
  EXPECT_EQ(qdata->stamp.ptr(), stamp);
  EXPECT_EQ(qdata2->stamp.ptr(), stamp2);

  // values are differenct now
  (*stamp)--;
  (*stamp2)++;
  EXPECT_EQ(*qdata->stamp, *stamp);
  EXPECT_EQ(*qdata2->stamp, *stamp2);
  EXPECT_NE(*qdata->stamp, *qdata2->stamp);
}