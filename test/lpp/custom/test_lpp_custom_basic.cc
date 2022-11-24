//
// Created by 4c3y (acey) on 24.11.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

using namespace lpp::custom;

void logCallback(BaseSeverity severity, const std::string& str) {

  std::string severity_str;
  switch (severity) {

    case BaseSeverity::DEBUG:severity_str="debug"; break;
    case BaseSeverity::INFO:severity_str="info"; break;
    case BaseSeverity::WARN:severity_str="warning"; break;
    case BaseSeverity::ERROR:severity_str="error"; break;
    case BaseSeverity::FATAL:severity_str="fatal"; break;
  }


  std::cout << "Log++ [" << severity_str << "] " << str << std::endl;
}

//! LPP syntax

TEST(lpp_custom_basic, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(D, "test"));

  ASSERT_EQ(output, debug);
}

TEST(lpp_custom_basic, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(I, "test"));

  ASSERT_EQ(output, info);
}

TEST(lpp_custom_basic, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(W, "test"));

  ASSERT_EQ(output, warning);
}

TEST(lpp_custom_basic, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(E, "test"));

  ASSERT_EQ(output, error);
}

TEST(lpp_custom_basic, lpp_syntax_severity_fatal) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(F, "test"));

  ASSERT_EQ(output, fatal);
}


//! Glog syntax
TEST(lpp_custom_basic, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(DLOG(INFO) << "test";);

  ASSERT_EQ(output, debug);
}

TEST(lpp_custom_basic, glog_syntax_severity_info) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(INFO) << "test";);

  ASSERT_EQ(output, info);
}

TEST(lpp_custom_basic, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(WARNING) << "test";);

  ASSERT_EQ(output, warning);
}

TEST(lpp_custom_basic, glog_syntax_severity_error) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(ERROR) << "test";);

  ASSERT_EQ(output, error);
}

TEST(lpp_custom_basic, glog_syntax_severity_fatal) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG(FATAL) << "test";);

  ASSERT_EQ(output, fatal);
}


//! Roslog syntax

TEST(lpp_custom_basic, roslog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG("test"));
  ASSERT_EQ(output, debug);
}

TEST(lpp_custom_basic, roslog_syntax_severity_debug_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_STREAM("test"));
  ASSERT_EQ(output, debug);
}

TEST(lpp_custom_basic, roslog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO("test"));
  ASSERT_EQ(output, info);
}

TEST(lpp_custom_basic, roslog_syntax_severity_info_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_STREAM("test"));
  ASSERT_EQ(output, info);
}

TEST(lpp_custom_basic, roslog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_WARN("test"));
  ASSERT_EQ(output, warning);
}

TEST(lpp_custom_basic, roslog_syntax_severity_warning_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_WARN_STREAM("test"));
  ASSERT_EQ(output, warning);
}

TEST(lpp_custom_basic, roslog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_ERROR("test"));
  ASSERT_EQ(output, error);
}

TEST(lpp_custom_basic, roslog_syntax_severity_error_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_ERROR_STREAM("test"));
  ASSERT_EQ(output, error);
}

TEST(lpp_custom_basic, roslog_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_FATAL("test"));
  ASSERT_EQ(output, fatal);
}

TEST(lpp_custom_basic, roslog_syntax_severity_fatal_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_FATAL_STREAM("test"));
  ASSERT_EQ(output, fatal);
}

