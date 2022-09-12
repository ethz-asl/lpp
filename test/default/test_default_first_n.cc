//
// Created by 4c3y (acey) on 12.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(default_LogPolicyFirstN, glog_syntax) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_FIRST_N(INFO, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStderr();

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogPolicyFirstN, lpp_syntax) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(I, 3, "Test"));

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test"));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}