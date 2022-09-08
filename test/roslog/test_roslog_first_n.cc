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
