//
// Created by 4c3y (acey) on 06.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(LPP_LogEveryN, lpp_syntax_severity_info) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(I, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, "INFO  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(LPP_LogEveryN, glog_syntax_severity_info) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY_N(INFO, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, "INFO  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}