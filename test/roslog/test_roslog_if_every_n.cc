//
// Created by 4c3y (acey) on 15.12.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

using namespace lpp::rostest;

TEST(roslog_if_every_n, glog_syntax_if_every_n_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    DLOG_IF_EVERY_N(INFO, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(debug, removeNumbersFromString(output));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(roslog_if_every_n, glog_syntax_if_every_n_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    LOG_IF_EVERY_N(INFO, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(info, removeNumbersFromString(output));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(roslog_if_every_n, glog_syntax_if_every_n_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_IF_EVERY_N(WARNING, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStderr();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(warning, removeNumbersFromString(output));
    } else {
      ASSERT_EQ(output, "");
    }
  }

}

TEST(roslog_if_every_n, glog_syntax_if_every_n_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_IF_EVERY_N(ERROR, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStderr();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(error, removeNumbersFromString(output));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(roslog_if_every_n, glog_syntax_if_every_n_severity_fatal) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_IF_EVERY_N(FATAL, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStderr();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(fatal, removeNumbersFromString(output));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}