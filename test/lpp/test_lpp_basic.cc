//
// Created by 4c3y (acey) on 06.09.22.
//

#include <test_utils.h>
#include <gtest/gtest.h>
#include <log++.h>

//! LPP syntax

TEST(lpp_basic, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(D, "Test" << 123));
  ASSERT_EQ(output, "DEBUG Test123\n");
}

TEST(lpp_basic, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(I, "Test" << 123));
  ASSERT_EQ(output, "INFO  Test123\n");
}

TEST(lpp_basic, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(W, "Test" << 123));
  ASSERT_EQ(output, "WARN  Test123\n");
}

TEST(lpp_basic, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(E, "Test" << 123));
  ASSERT_EQ(output, "ERROR Test123\n");
}

TEST(lpp_basic, lpp_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(F, "Test" << 123));
  ASSERT_EQ(output, "FATAL Test123\n");
}


//! Glog syntax
TEST(lpp_basic, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(DLOG(INFO) << "Test" << 123);
  ASSERT_EQ(output, "DEBUG Test123\n");
}

TEST(lpp_basic, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(INFO) << "Test" << 123);
  ASSERT_EQ(output, "INFO  Test123\n");
}

TEST(lpp_basic, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(WARNING) << "Test" << 123);
  ASSERT_EQ(output, "WARN  Test123\n");
}

TEST(lpp_basic, glog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(ERROR) << "Test" << 123);
  ASSERT_EQ(output, "ERROR Test123\n");
}

TEST(lpp_basic, glog_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(FATAL) << "Test" << 123);
  ASSERT_EQ(output, "FATAL Test123\n");
}


//! Roslog syntax
TEST(lpp_basic, roslog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG("Test123"));
  ASSERT_EQ(output, "DEBUG Test123\n");
}

TEST(lpp_basic, roslog_syntax_severity_debug_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_STREAM("Test" << 123));
  ASSERT_EQ(output, "DEBUG Test123\n");
}

TEST(lpp_basic, roslog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO("Test123"));
  ASSERT_EQ(output, "INFO  Test123\n");
}

TEST(lpp_basic, roslog_syntax_severity_info_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_STREAM("Test" << 123));
  ASSERT_EQ(output, "INFO  Test123\n");
}

TEST(lpp_basic, roslog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_WARN("Test123"));
  ASSERT_EQ(output, "WARN  Test123\n");
}

TEST(lpp_basic, roslog_syntax_severity_warning_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_WARN_STREAM("Test" << 123));
  ASSERT_EQ(output, "WARN  Test123\n");
}

TEST(lpp_basic, roslog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_ERROR("Test123"));
  ASSERT_EQ(output, "ERROR Test123\n");
}

TEST(lpp_basic, roslog_syntax_severity_error_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_ERROR_STREAM("Test" << 123));
  ASSERT_EQ(output, "ERROR Test123\n");
}

TEST(lpp_basic, roslog_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_FATAL("Test123"));
  ASSERT_EQ(output, "FATAL Test123\n");
}

TEST(lpp_basic, roslog_syntax_severity_fatal_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_FATAL_STREAM("Test" << 123));
  ASSERT_EQ(output, "FATAL Test123\n");
}