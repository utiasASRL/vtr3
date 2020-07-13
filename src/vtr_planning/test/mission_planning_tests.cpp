#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(TestCase, testName) { EXPECT_EQ(0, 0); }

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
#if 0
  // run `roscore` if you need the follwoing in your test.
  ros::init(argc, argv, "mission_planning_tests");
  ros::NodeHandle nh;
#endif
  return RUN_ALL_TESTS();
}