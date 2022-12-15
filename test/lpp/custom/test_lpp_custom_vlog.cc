//
// Created by 4c3y (acey) on 24.11.22.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>
#include "callback.h"

using namespace lpp::custom;

TEST(lpp_vlog, glog_syntax_severity_v1) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG(1) << "test" << 123);
  ASSERT_EQ(output, debug + test123);
}

TEST(lpp_vlog, glog_syntax_severity_v3) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG(3) << "test123");
  ASSERT_EQ(output, debug + test123);
}

TEST(lpp_vlog, glog_syntax_severity_v5) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG(5) << "test123");
  ASSERT_EQ(output, "");
}

TEST(lpp_vlog, glog_syntax_severity_if_v1) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(1, true) << "test123");
  ASSERT_EQ(output, debug + test123);
}

TEST(lpp_vlog, glog_syntax_severity_if_v3) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(1, true) << "test123");
  ASSERT_EQ(output, debug + test123);
}

TEST(lpp_vlog, glog_syntax_severity_if_v5) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(5, true) << "test123");
  ASSERT_EQ(output, "");
}

TEST(lpp_vlog, glog_syntax_severity_ifnot_v1) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(1, false) << "test123");
  ASSERT_EQ(output, "");
}

TEST(lpp_vlog, glog_syntax_severity_ifnot_v3) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(3, false) << "test123");
  ASSERT_EQ(output, "");
}

TEST(lpp_vlog, glog_syntax_severity_ifnot_v5) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(5, false) << "test123");
  ASSERT_EQ(output, "");
}

TEST(lpp_vlog, glog_syntax_every_n_severity_v1) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(VLOG_EVERY_N(1, 3) << "test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, debug + test123);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_vlog, glog_syntax_every_n_severity_v3) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(VLOG_EVERY_N(3, 3) << "test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, debug + test123);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_vlog, glog_syntax_every_n_severity_v5) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(VLOG_EVERY_N(5, 3) << "test" << 123);
    ASSERT_EQ(output, "");
  }
}

TEST(lpp_vlog, glog_syntax_if_every_n_severity_v1) {
  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    VLOG_IF_EVERY_N(1, i <= 3, 3) << "test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, debug + test123);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_vlog, glog_syntax_if_every_n_severity_v3) {
  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    VLOG_IF_EVERY_N(3, i <= 3, 3) << "test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, debug + test123);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_vlog, glog_syntax_if_every_n_severity_v5) {
  LOG_INIT(*test_argv, logCallback);
  FLAGS_v = 3;

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    VLOG_IF_EVERY_N(5, i <= 3, 3) << "test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    ASSERT_EQ(output, "");
  }
}