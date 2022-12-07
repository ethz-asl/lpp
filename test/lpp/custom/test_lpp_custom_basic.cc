//
// Created by 4c3y (acey) on 24.11.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>
#include "callback.h"

using namespace lpp::custom;

//! LPP syntax

TEST(lpp_custom_basic, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(D, "test" << 123));

  ASSERT_EQ(output, debug + test123);
}

TEST(lpp_custom_basic, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(I, "test" << 123));

  ASSERT_EQ(output, info + test123);
}

TEST(lpp_custom_basic, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(W, "test" << 123));

  ASSERT_EQ(output, warning + test123);
}

TEST(lpp_custom_basic, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(E, "test" << 123));

  ASSERT_EQ(output, error + test123);
}

TEST(lpp_custom_basic, lpp_syntax_severity_fatal) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(F, "test" << 123));

  ASSERT_EQ(output, fatal + test123);
}


//! Glog syntax
TEST(lpp_custom_basic, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(DLOG(INFO) << "test" << 123;);

  ASSERT_EQ(output, debug + test123);
}

TEST(lpp_custom_basic, glog_syntax_severity_info) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(INFO) << "test" << 123;);

  ASSERT_EQ(output, info + test123);
}

TEST(lpp_custom_basic, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(WARNING) << "test" << 123;);

  ASSERT_EQ(output, warning + test123);
}

TEST(lpp_custom_basic, glog_syntax_severity_error) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(ERROR) << "test" << 123;);

  ASSERT_EQ(output, error + test123);
}

TEST(lpp_custom_basic, glog_syntax_severity_fatal) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(FATAL) << "test" << 123;);

  ASSERT_EQ(output, fatal + test123);
}


//! Roslog syntax
TEST(lpp_custom_basic, roslog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG("test123"));
  ASSERT_EQ(output, debug + test123);
}

TEST(lpp_custom_basic, roslog_syntax_severity_debug_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_STREAM("test" << 123));
  ASSERT_EQ(output, debug + test123);
}

TEST(lpp_custom_basic, roslog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO("test123"));
  ASSERT_EQ(output, info + test123);
}

TEST(lpp_custom_basic, roslog_syntax_severity_info_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_STREAM("test" << 123));
  ASSERT_EQ(output, info + test123);
}

TEST(lpp_custom_basic, roslog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_WARN("test123"));
  ASSERT_EQ(output, warning + test123);
}

TEST(lpp_custom_basic, roslog_syntax_severity_warning_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_WARN_STREAM("test" << 123));
  ASSERT_EQ(output, warning + test123);
}

TEST(lpp_custom_basic, roslog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_ERROR("test123"));
  ASSERT_EQ(output, error + test123);
}

TEST(lpp_custom_basic, roslog_syntax_severity_error_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_ERROR_STREAM("test" << 123));
  ASSERT_EQ(output, error + test123);
}

TEST(lpp_custom_basic, roslog_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_FATAL("test123"));
  ASSERT_EQ(output, fatal + test123);
}

TEST(lpp_custom_basic, roslog_syntax_severity_fatal_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_FATAL_STREAM("test" << 123));
  ASSERT_EQ(output, fatal + test123);
}

