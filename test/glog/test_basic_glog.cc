//
// Created by acey on 23.08.22.
//
#define MODE_GLOG
#include "test_utils.h"
#include <gtest/gtest.h>
#include "log++.h"

//! ################ glog ################
TEST(glog_glog_syntax, severity_info) {
  LOG_INIT(*test_argv);       //Only in first glog test
  FLAGS_logtostderr = true;   //Only in first glog test

  testing::internal::CaptureStderr();

  LOG(INFO) << "xyz";
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));

  ASSERT_EQ(output[0], 'I');
}

TEST(glog_glog_syntax, severity_warning) {
  testing::internal::CaptureStderr();
  FLAGS_logtostderr = true;
  LOG(WARNING) << "xyz";
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));

  ASSERT_EQ(output[0], 'W');
}

TEST(glog_glog_syntax, severity_error) {
  testing::internal::CaptureStderr();
  FLAGS_logtostderr = true;
  LOG(ERROR) << "xyz";
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "xyz"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));

  ASSERT_EQ(output[0], 'E');
}


//! ################ lpp ################
TEST(glog_lpp_syntax, severity_info) {
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  LOG(I, "Test" << "123")
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_basic_glog.cc"));

  ASSERT_TRUE(output[0] == 'I');
}

TEST(glog_lpp_syntax, severity_warning) {
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  LOG(W, "Test" << "123")
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_EQ(output[0], 'W');
}

TEST(glog_lpp_syntax, severity_error) {
  FLAGS_logtostderr = true;

  testing::internal::CaptureStderr();
  LOG(E, "Test" << "123")
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_TRUE(isSubstring(output, "Test123"));
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



#undef MODE_GLOG