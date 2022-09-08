//
// Created by 4c3y (acey) on 08.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(roslog_LogPolicyFirstN, lpp_syntax) {
  //Should print 2 times [ INFO] [Rostime]: Test
  for (int i = 0; i < 5; i++) {
    LOG_FIRST(I, 3, "Test");
  }
}

TEST(roslog_LogPolicyFirstN, glog_syntax) {
  //Should print 2 times [ INFO] [Rostime]: Test
  for (int i = 0; i < 5; i++) {
    LOG_FIRST_N(INFO, 3) << "Test";
  }
}

TEST(roslog_LogPolicyFirstN, ros_info_once) {
  //Should print 1 time [ INFO] [Rostime]: Test123
  for (int i = 0; i < 5; i++) {
    ROS_INFO_ONCE("Test123");
  }
}

TEST(roslog_LogPolicyFirstN, ros_warn_once) {
  //Should print 1 time [ WARN] [Rostime]: Test123
  for (int i = 0; i < 5; i++) {
    ROS_INFO_ONCE("Test123");
  }
}

TEST(roslog_LogPolicyFirstN, ros_error_once) {
  //Should print 1 time [ERROR] [Rostime]: Test123
  for (int i = 0; i < 5; i++) {
    ROS_INFO_ONCE("Test123");
  }
}

