//
// Created by 4c3y (acey) on 15.12.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(lpp_if_every_n, glog_syntax_if_every_n_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    DLOG_IF_EVERY_N(INFO, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, "DEBUG Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_if_every_n, glog_syntax_if_every_n_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    LOG_IF_EVERY_N(INFO, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, "INFO  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_if_every_n, glog_syntax_if_every_n_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    LOG_IF_EVERY_N(WARNING, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, "WARN  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }

}

TEST(lpp_if_every_n, glog_syntax_if_every_n_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    LOG_IF_EVERY_N(ERROR, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, "ERROR Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_if_every_n, glog_syntax_if_every_n_severity_fatal) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    LOG_IF_EVERY_N(FATAL, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, "FATAL Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}