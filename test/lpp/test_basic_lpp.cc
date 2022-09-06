//
// Created by 4c3y (acey) on 06.09.22.
//

#include <test_utils.h>
#include <gtest/gtest.h>
#include <log++.h>

//! LPP syntax
TEST(LPP_lpp_syntax, severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(I, "Test" << 123));
  ASSERT_EQ(output, "INFO  Test123\n");
}

TEST(LPP_lpp_syntax, severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(W, "Test" << 123));
  ASSERT_EQ(output, "WARN  Test123\n");
}

TEST(LPP_lpp_syntax, severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(E, "Test" << 123));
  ASSERT_EQ(output, "ERROR Test123\n");
}


//! Glog syntax
TEST(LPP_glog_syntax, severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(INFO) << "Test" << 123);
  ASSERT_EQ(output, "INFO  Test123\n");
}

TEST(LPP_glog_syntax, severity_warning) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(WARNING) << "Test" << 123);
  ASSERT_EQ(output, "WARN  Test123\n");
}

TEST(LPP_glog_syntax, severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(ERROR) << "Test" << 123);
  ASSERT_EQ(output, "ERROR Test123\n");
}