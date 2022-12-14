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
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(1, true) << "test123");
  ASSERT_EQ(output, debug + test123);
}

TEST(lpp_vlog, glog_syntax_severity_if_v3) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(1, true) << "test123");
  ASSERT_EQ(output, debug + test123);
}

TEST(lpp_vlog, glog_syntax_severity_if_v5) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(5, true) << "test123");
  ASSERT_EQ(output, "");
}

TEST(lpp_vlog, glog_syntax_severity_ifnot_v1) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(1, false) << "test123");
  ASSERT_EQ(output, "");
}

TEST(lpp_vlog, glog_syntax_severity_ifnot_v3) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(3, false) << "test123");
  ASSERT_EQ(output, "");
}

TEST(lpp_vlog, glog_syntax_severity_ifnot_v5) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(5, false) << "test123");
  ASSERT_EQ(output, "");
}