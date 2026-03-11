#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

#include "sysd_test_utils.h"

using lpp::sysdtest::entries;

TEST(sysd_log_string, severity_info) {
  lpp::sysdtest::init(*test_argv);
  std::vector<std::string> errors;
  LOG_STRING(INFO, &errors) << "LOG_STRING: " << "collected info";
  ASSERT_TRUE(entries().empty());
  ASSERT_EQ(errors.at(0), "LOG_STRING: collected info");
}

TEST(sysd_log_string, severity_info_null) {
  lpp::sysdtest::init(*test_argv);
  LOG_STRING(INFO, nullptr) << "LOG_STRING: " << "collected info";
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
  ASSERT_EQ(entries().at(0).message, "LOG_STRING: collected info");
}

TEST(sysd_log_string, severity_warning) {
  lpp::sysdtest::init(*test_argv);
  std::vector<std::string> errors;
  LOG_STRING(WARNING, &errors) << "LOG_STRING: " << "collected warn";
  ASSERT_TRUE(entries().empty());
  ASSERT_EQ(errors.at(0), "LOG_STRING: collected warn");
}

TEST(sysd_log_string, severity_warning_null) {
  lpp::sysdtest::init(*test_argv);
  LOG_STRING(WARNING, nullptr) << "LOG_STRING: " << "collected warn";
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
  ASSERT_EQ(entries().at(0).message, "LOG_STRING: collected warn");
}

TEST(sysd_log_string, severity_error) {
  lpp::sysdtest::init(*test_argv);
  std::vector<std::string> errors;
  LOG_STRING(ERROR, &errors) << "LOG_STRING: " << "collected error";
  ASSERT_TRUE(entries().empty());
  ASSERT_EQ(errors.at(0), "LOG_STRING: collected error");
}

TEST(sysd_log_string, severity_error_null) {
  lpp::sysdtest::init(*test_argv);
  LOG_STRING(ERROR, nullptr) << "LOG_STRING: " << "collected error";
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
  ASSERT_EQ(entries().at(0).message, "LOG_STRING: collected error");
}

TEST(sysd_log_string, severity_fatal) {
  lpp::sysdtest::init(*test_argv);
  std::vector<std::string> errors;
  LOG_STRING(FATAL, &errors) << "LOG_STRING: " << "collected fatal";
  ASSERT_TRUE(entries().empty());
  ASSERT_EQ(errors.at(0), "LOG_STRING: collected fatal");
}

TEST(sysd_log_string, severity_fatal_null) {
  lpp::sysdtest::init(*test_argv);
  LOG_STRING(FATAL, nullptr) << "LOG_STRING: " << "collected fatal";
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
  ASSERT_EQ(entries().at(0).message, "LOG_STRING: collected fatal");
}
