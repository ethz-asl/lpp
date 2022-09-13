//
// Created by acey on 23.08.22.
//

#include "test_utils.h"
#include <gtest/gtest.h>
#include <log++.h>

//! ################ glog ################

TEST(glog_basic, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(DLOG(INFO) << "xyz");

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));

  ASSERT_EQ(output[0], 'I');
}

TEST(glog_basic, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(INFO) << "xyz");

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));

  ASSERT_EQ(output[0], 'I');
}

TEST(glog_basic, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(WARNING) << "xyz");

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));

  ASSERT_EQ(output[0], 'W');
}

TEST(glog_basic, glog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(ERROR) << "xyz");

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));

  ASSERT_EQ(output[0], 'E');
}


//! ################ lpp ################

TEST(glog_basic, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(D, "Test" << "123"));

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));

  ASSERT_TRUE(output[0] == 'I');
}

TEST(glog_basic, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(I, "Test" << 123));

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));

  ASSERT_TRUE(output[0] == 'I');
}

TEST(glog_basic, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(W, "Test" << 123));

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));
  ASSERT_EQ(output[0], 'W');
}

TEST(glog_basic, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(E, "Test" << 123));

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));
  ASSERT_EQ(output[0], 'E');
}

//TODO https://stackoverflow.com/questions/55760359/is-it-possible-to-test-that-an-abort-routine-doesnt-return
/*
TEST(glog_lpp_syntax, lpp_fatal) {
  FLAGS_logtostderr = true;
  LOG_INIT(*test_argv)

  testing::internal::CaptureStderr();
  LOG(F, "Test" << "123")
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(output[0] == 'F');
}
*/

//! ################ Roslog ################
TEST(glog_basic, roslog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR( ROS_DEBUG("Test123"));

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));
  ASSERT_EQ(output[0], 'I');
}

TEST(glog_basic, roslog_syntax_severity_debug_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_DEBUG_STREAM("Test123"));

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));
  ASSERT_EQ(output[0], 'I');
}

TEST(glog_basic, roslog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_INFO("Test123"));

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));
  ASSERT_EQ(output[0], 'I');
}

TEST(glog_basic, roslog_syntax_severity_info_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_INFO_STREAM("Test" << 123));

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));
  ASSERT_EQ(output[0], 'I');
}

TEST(glog_basic, roslog_syntax_severity_warn) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN("Test123"));

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));
  ASSERT_EQ(output[0], 'W');
}

TEST(glog_basic, roslog_syntax_severity_warn_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN_STREAM("Test" << 123));

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));
  ASSERT_EQ(output[0], 'W');
}

TEST(glog_basic, roslog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR("Test123"));

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));
  ASSERT_EQ(output[0], 'E');
}

TEST(glog_basic, roslog_syntax_severity_error_stream) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_STREAM("Test" << 123));

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_glog_basic.cc"));
  ASSERT_EQ(output[0], 'E');
}

#undef MODE_GLOG