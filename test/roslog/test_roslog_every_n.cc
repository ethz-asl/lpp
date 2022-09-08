//
// Created by 4c3y (acey) on 08.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(roslog_LogEveryN, lpp_syntax) {
  //Should print 2 times [ INFO] [Rostime]: Test
  for (int i = 0; i < 5; i++) {
    LOG_EVERY(I, 3, "Test");
  }
}

TEST(roslog_LogEveryN, glog_syntax) {
  //Should print 2 times [ INFO] [Rostime]: Test
  for (int i = 0; i < 5; i++) {
    LOG_EVERY_N(INFO, 3) << "Test";
  }
}