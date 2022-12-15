//
// Created by 4c3y (acey) on 15.12.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>
#include "callback.h"

using namespace lpp::custom;

TEST(lpp_custom_if_every_n, glog_syntax_if_every_n_severity_debug) {
  LOG_INIT(*test_argv, logCallback);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    DLOG_IF_EVERY_N(INFO, i <= 3, 3) << "test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, debug + test123);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_if_every_n, glog_syntax_if_every_n_severity_info) {
  LOG_INIT(*test_argv, logCallback);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    LOG_IF_EVERY_N(INFO, i <= 3, 3) << "test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, info + test123);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_if_every_n, glog_syntax_if_every_n_severity_warning) {
  LOG_INIT(*test_argv, logCallback);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    LOG_IF_EVERY_N(WARNING, i <= 3, 3) << "test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, warning + test123);
    } else {
      ASSERT_EQ(output, "");
    }
  }

}

TEST(lpp_custom_if_every_n, glog_syntax_if_every_n_severity_error) {
  LOG_INIT(*test_argv, logCallback);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    LOG_IF_EVERY_N(ERROR, i <= 3, 3) << "test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, error + test123);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_if_every_n, glog_syntax_if_every_n_severity_fatal) {
  LOG_INIT(*test_argv, logCallback);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    LOG_IF_EVERY_N(FATAL, i <= 3, 3) << "test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, fatal + test123);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}