//
// Created by 4c3y (acey) on 08.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(glog_LogPolicyEveryN, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_EVERY(D, 3, "Test" << 123);
    std::string output = testing::internal::GetCapturedStderr();

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogPolicyEveryN, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_EVERY(I, 3, "Test" << 123);
    std::string output = testing::internal::GetCapturedStderr();

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogPolicyEveryN, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_EVERY_N(INFO, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStderr();

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogPolicyEveryN, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_EVERY_N(INFO, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStderr();

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}