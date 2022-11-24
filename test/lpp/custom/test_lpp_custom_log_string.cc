//
// Created by 4c3y (acey) on 24.11.22.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>
#include "callback.h"

using namespace lpp::custom;

TEST(lpp_log_string, severity_info) {
  LOG_INIT(*test_argv, logCallback);

  std::vector<std::string> errors;

//Can't capture variables in lambda with LPP_CAPTURE_STDERR()
  testing::internal::CaptureStdout();
  LOG_STRING(INFO, &errors) << "test" << 123;
  std::string output = testing::internal::GetCapturedStdout();

  ASSERT_EQ(output, "");
  ASSERT_EQ("test123", errors.at(0));
}

TEST(lpp_log_string, severity_info_null) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG_STRING(INFO, nullptr) << "test" << 123);
  ASSERT_EQ(output, info + test123);
}

TEST(lpp_log_string, severity_warning) {
  LOG_INIT(*test_argv, logCallback);

  std::vector<std::string> errors;

//Can't capture variables in lambda with LPP_CAPTURE_STDERR()
  testing::internal::CaptureStdout();
  LOG_STRING(WARNING, &errors) << "test" << 123;
  std::string output = testing::internal::GetCapturedStdout();

  ASSERT_EQ(output, "");
  ASSERT_EQ("test123", errors.at(0));
}

TEST(lpp_log_string, severity_warning_null) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG_STRING(WARNING, nullptr) << "test" << 123);
  ASSERT_EQ(output, warning + test123);
}

TEST(lpp_log_string, severity_error) {
  LOG_INIT(*test_argv, logCallback);

  std::vector<std::string> errors;

//Can't capture variables in lambda with LPP_CAPTURE_STDERR()
  testing::internal::CaptureStdout();
  LOG_STRING(ERROR, &errors) << "test" << 123;
  std::string output = testing::internal::GetCapturedStdout();

  ASSERT_EQ(output, "");
  ASSERT_EQ("test123", errors.at(0));
}

TEST(lpp_log_string, severity_error_null) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG_STRING(ERROR, nullptr) << "test" << 123);
  ASSERT_EQ(output, error + test123);
}

TEST(lpp_log_string, severity_fatal) {
  LOG_INIT(*test_argv, logCallback);

  std::vector<std::string> errors;

//Can't capture variables in lambda with LPP_CAPTURE_STDERR()
  testing::internal::CaptureStdout();
  LOG_STRING(FATAL, &errors) << "test" << 123;
  std::string output = testing::internal::GetCapturedStdout();
  ASSERT_EQ(output, "");

  ASSERT_EQ("test123", errors.at(0));
}

TEST(lpp_log_string, severity_fatal_null) {
  LOG_INIT(*test_argv, logCallback);

  std::string output = LPP_CAPTURE_STDOUT(LOG_STRING(FATAL, nullptr) << "test" << 123);
  ASSERT_EQ(output, fatal + test123);
}