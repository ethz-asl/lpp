//
// Created by acey on 23.08.22.
//
#define MODE_GLOG
#include "test_utils.h"
#include <gtest/gtest.h>
#include "log++.h"

//! ################ glog ################
TEST(glog_glog_syntax, severity_info) {
  LOG_INIT(*test_argv);
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  LOG(INFO) << "xyz";
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));

  ASSERT_EQ(output[0], 'I');
}

TEST(glog_glog_syntax, severity_warning) {
  LOG_INIT(*test_argv);
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  LOG(WARNING) << "xyz";
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));

  ASSERT_EQ(output[0], 'W');
}

TEST(glog_glog_syntax, severity_error) {
  LOG_INIT(*test_argv);
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  LOG(ERROR) << "xyz";
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));

  ASSERT_EQ(output[0], 'E');
}


//! ################ lpp ################
TEST(glog_lpp_syntax, severity_info) {
  LOG_INIT(*test_argv);
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  LOG(I, "Test" << "123")
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));

  ASSERT_TRUE(output[0] == 'I');
}

TEST(glog_lpp_syntax, severity_warning) {
  LOG_INIT(*test_argv);
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  LOG(W, "Test" << "123")
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));
  ASSERT_EQ(output[0], 'W');
}

TEST(glog_lpp_syntax, severity_error) {
  LOG_INIT(*test_argv);
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  LOG(E, "Test" << "123")
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));
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
TEST(glog_roslog_syntax, severity_info) {
  LOG_INIT(*test_argv);
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  ROS_INFO("Test123");
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));
  ASSERT_EQ(output[0], 'I');
}

TEST(glog_roslog_syntax, severity_info_stream) {
  LOG_INIT(*test_argv);
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  ROS_INFO_STREAM("Test" << 123);
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));
  ASSERT_EQ(output[0], 'I');
}

TEST(glog_roslog_syntax, severity_warn) {
  LOG_INIT(*test_argv);
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  ROS_WARN("Test123");
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));
  ASSERT_EQ(output[0], 'W');
}

TEST(glog_roslog_syntax, severity_warn_stream) {
  LOG_INIT(*test_argv);
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  ROS_WARN_STREAM("Test" << 123);
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));
  ASSERT_EQ(output[0], 'W');
}

TEST(glog_roslog_syntax, severity_error) {
  LOG_INIT(*test_argv);
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  ROS_ERROR("Test123");
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));
  ASSERT_EQ(output[0], 'E');
}

TEST(glog_roslog_syntax, severity_error_stream) {
  LOG_INIT(*test_argv);
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  ROS_ERROR_STREAM("Test" << 123);
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));
  ASSERT_EQ(output[0], 'E');
}

#undef MODE_GLOG