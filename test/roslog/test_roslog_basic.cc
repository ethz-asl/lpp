//
// Created by acey on 30.08.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

using namespace lpp::rostest;

TEST(roslog_basic, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(DLOG(INFO) << "Test");
  EXPECT_TRUE(debug == removeNumbersFromString(output) || v2::debug == removeNumbersFromString(output));
}

TEST(roslog_basic, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(INFO) << "Test123");
  EXPECT_TRUE(info == removeNumbersFromString(output) || v2::info == removeNumbersFromString(output));
}

TEST(roslog_basic, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(WARNING) << "Test");
  EXPECT_TRUE(warning == removeNumbersFromString(output) || v2::warning == removeNumbersFromString(output));
}

TEST(roslog_basic, glog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(ERROR) << "Test");
  EXPECT_TRUE(error == removeNumbersFromString(output) || v2::error == removeNumbersFromString(output));
}

TEST(roslog_basic, glog_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(FATAL) << "Test");
  EXPECT_TRUE(fatal == removeNumbersFromString(output) || v2::fatal == removeNumbersFromString(output));
}

//! ################ lpp ################
TEST(roslog_basic, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(D, "" << "Test"));
  EXPECT_TRUE(debug == removeNumbersFromString(output) || v2::debug == removeNumbersFromString(output));
}

TEST(roslog_basic, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(I, "" << "Test"));
  EXPECT_TRUE(info == removeNumbersFromString(output) || v2::info == removeNumbersFromString(output));
}

TEST(roslog_basic, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(W, "" << "Test"));
  EXPECT_TRUE(warning == removeNumbersFromString(output) || v2::warning == removeNumbersFromString(output));
}

TEST(roslog_basic, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(E, "" << "Test"));
  EXPECT_TRUE(error == removeNumbersFromString(output) || v2::error == removeNumbersFromString(output));
}

TEST(roslog_basic, lpp_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(F, "" << "Test"));
  EXPECT_TRUE(fatal == removeNumbersFromString(output) || v2::fatal == removeNumbersFromString(output));
}

//! ################ Roslog ################
TEST(roslog_basic, roslog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG("Test"));
  EXPECT_TRUE(debug == removeNumbersFromString(output) || v2::debug == removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_debug_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_STREAM("Test"));
  EXPECT_TRUE(debug == removeNumbersFromString(output) || v2::debug == removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO("Test"));
  EXPECT_TRUE(info == removeNumbersFromString(output) || v2::info == removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_info_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_STREAM("" << "Test"));
  EXPECT_TRUE(info == removeNumbersFromString(output) || v2::info == removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_warn) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN("Test"));
  EXPECT_TRUE(warning == removeNumbersFromString(output) || v2::warning == removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_warn_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN_STREAM("" << "Test"));
  EXPECT_TRUE(warning == removeNumbersFromString(output) || v2::warning == removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR("Test"));
  EXPECT_TRUE(error == removeNumbersFromString(output) || v2::error == removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_error_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_STREAM("" << "Test"));
  EXPECT_TRUE(error == removeNumbersFromString(output) || v2::error == removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL("Test"));
  EXPECT_TRUE(fatal == removeNumbersFromString(output) || v2::fatal == removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_fatal_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_STREAM("" << "Test"));
  EXPECT_TRUE(fatal == removeNumbersFromString(output) || v2::fatal == removeNumbersFromString(output));
}
