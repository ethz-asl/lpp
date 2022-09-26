//
// Created by acey on 30.08.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

using namespace lpp;

TEST(roslog_basic, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(DLOG(INFO) << "Test");
  ASSERT_EQ(rostest::debug, removeNumbersFromString(output));
}

TEST(roslog_basic, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(INFO) << "Test123");
  ASSERT_EQ(rostest::info, removeNumbersFromString(output));
}

TEST(roslog_basic, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(WARNING) << "Test");
  ASSERT_EQ(rostest::warning, removeNumbersFromString(output));
}

TEST(roslog_basic, glog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(ERROR) << "Test");
  ASSERT_EQ(rostest::error, removeNumbersFromString(output));
}

TEST(roslog_basic, glog_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(FATAL) << "Test");
  ASSERT_EQ(rostest::fatal, removeNumbersFromString(output));
}

//! ################ lpp ################
TEST(roslog_basic, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(D, "" << "Test"));
  ASSERT_EQ(rostest::debug, removeNumbersFromString(output));
}

TEST(roslog_basic, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(I, "" << "Test"));
  ASSERT_EQ(rostest::info, removeNumbersFromString(output));
}

TEST(roslog_basic, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(W, "" << "Test"));
  ASSERT_EQ(rostest::warning, removeNumbersFromString(output));
}

TEST(roslog_basic, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(E, "" << "Test"));
  ASSERT_EQ(rostest::error, removeNumbersFromString(output));
}

TEST(roslog_basic, lpp_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(F, "" << "Test"));
  ASSERT_EQ(rostest::fatal, removeNumbersFromString(output));
}

//! ################ Roslog ################
TEST(roslog_basic, roslog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG("Test"));
  ASSERT_EQ(rostest::debug, removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_debug_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_STREAM("Test"));
  ASSERT_EQ(rostest::debug, removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO("Test"));
  ASSERT_EQ(rostest::info, removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_info_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_STREAM("" << "Test"));
  ASSERT_EQ(rostest::info, removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_warn) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN("Test"));
  ASSERT_EQ(rostest::warning, removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_warn_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN_STREAM("" << "Test"));
  ASSERT_EQ(rostest::warning, removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR("Test"));
  ASSERT_EQ(rostest::error, removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_error_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_STREAM("" << "Test"));
  ASSERT_EQ(rostest::error, removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL("Test"));
  ASSERT_EQ(rostest::fatal, removeNumbersFromString(output));
}

TEST(roslog_basic, roslog_syntax_severity_fatal_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_STREAM("" << "Test"));
  ASSERT_EQ(rostest::fatal, removeNumbersFromString(output));
}
