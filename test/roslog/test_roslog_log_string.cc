//
// Created by 4c3y (acey) on 28.09.22.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

using namespace lpp;

TEST(roslog_log_string, severity_info) {
  LOG_INIT(*test_argv);

  std::vector<std::string> errors;

  //Can't capture variables in lambda with LPP_CAPTURE_STDERR()
  testing::internal::CaptureStdout();
  LOG_STRING(INFO, &errors) << "LOG_STRING: " << "collected info";
  std::string output = testing::internal::GetCapturedStdout();

  ASSERT_EQ(output, "");
  ASSERT_EQ("LOG_STRING: collected info", errors.at(0));
}

TEST(roslog_log_string, severity_info_null) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG_STRING(INFO, nullptr) << "LOG_STRING: " << "collected info");
  ASSERT_EQ(removeNumbersFromString(output), logstr::info);
}

TEST(roslog_log_string, severity_warning) {
  LOG_INIT(*test_argv);

  std::vector<std::string> errors;

  //Can't capture variables in lambda with LPP_CAPTURE_STDERR()
  testing::internal::CaptureStderr();
  LOG_STRING(WARNING, &errors) << "LOG_STRING: " << "collected warn";
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_EQ(output, "");
  ASSERT_EQ("LOG_STRING: collected warn", errors.at(0));
}

TEST(roslog_log_string, severity_warning_null) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG_STRING(WARNING, nullptr) << "LOG_STRING: " << "collected warn");
  ASSERT_EQ(removeNumbersFromString(output), logstr::warning);
}

TEST(roslog_log_string, severity_error) {
  LOG_INIT(*test_argv);

  std::vector<std::string> errors;

  //Can't capture variables in lambda with LPP_CAPTURE_STDERR()
  testing::internal::CaptureStderr();
  LOG_STRING(ERROR, &errors) << "LOG_STRING: " << "collected error";
  std::string output = testing::internal::GetCapturedStderr();

  ASSERT_EQ(output, "");
  ASSERT_EQ("LOG_STRING: collected error", errors.at(0));
}

TEST(roslog_log_string, severity_error_null) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG_STRING(ERROR, nullptr) << "LOG_STRING: " << "collected error");
  ASSERT_EQ(removeNumbersFromString(output), logstr::error);
}

TEST(roslog_log_string, severity_fatal) {
  LOG_INIT(*test_argv);

  std::vector<std::string> errors;

  //Can't capture variables in lambda with LPP_CAPTURE_STDERR()
  testing::internal::CaptureStderr();
  LOG_STRING(FATAL, &errors) << "LOG_STRING: " << "collected fatal";
  std::string output = testing::internal::GetCapturedStderr();
  ASSERT_EQ(output, "");

  ASSERT_EQ("LOG_STRING: collected fatal", errors.at(0));
}

TEST(roslog_log_string, severity_fatal_null) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG_STRING(FATAL, nullptr) << "LOG_STRING: " << "collected fatal");
  ASSERT_EQ(removeNumbersFromString(output), logstr::fatal);
}