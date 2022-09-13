//
// Created by 4c3y (acey) on 12.09.22.
//

#include "test_utils.h"
#include <gtest/gtest.h>
#include <log++.h>

TEST(default_basic, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(DLOG(INFO) << "xyz");

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_default_basic.cc"));

  ASSERT_EQ(output[0], 'I');
}

TEST(default_basic, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(INFO) << "xyz");

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_default_basic.cc"));

  ASSERT_EQ(output[0], 'I');
}

TEST(default_basic, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(WARNING) << "xyz");

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_default_basic.cc"));
  ASSERT_EQ(output[0], 'W');
}

TEST(default_basic, glog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(ERROR) << "xyz");

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_default_basic.cc"));
  ASSERT_EQ(output[0], 'E');
}

TEST(default_basic, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(D, "Test" << 123));
  ASSERT_EQ(output, "DEBUG Test123\n");
}

TEST(default_basic, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(I, "Test" << 123));
  ASSERT_EQ(output, "INFO  Test123\n");
}

TEST(default_basic, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(W, "Test" << 123));
  ASSERT_EQ(output, "WARN  Test123\n");
}

TEST(default_basic, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(E, "Test" << 123));
  ASSERT_EQ(output, "ERROR Test123\n");
}

TEST(default_basic, lpp_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(F, "Test" << 123));
  ASSERT_EQ(output, "FATAL Test123\n");
}

TEST(default_basic, roslog_syntax_severity_debug) {
  ROS_DEBUG("Test");
  ROS_DEBUG_STREAM("" << "Test");
}

TEST(default_basic, roslog_syntax_severity_info) {
  ROS_INFO("Test");
  ROS_INFO_STREAM("" << "Test");
}

TEST(default_basic, roslog_syntax_severity_warn) {
  ROS_WARN("Test");
  ROS_WARN_STREAM("" << "Test");
}

TEST(default_basic, roslog_syntax_severity_error) {
  ROS_ERROR("Test");
  ROS_ERROR_STREAM("" << "Test");
}

TEST(default_basic, roslog_syntax_severity_fatal) {
  ROS_FATAL("Test");
  ROS_FATAL_STREAM("" << "Test");
}