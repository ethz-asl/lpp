//
// Created by acey on 07.09.23.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

TEST(nolog_log_string, severity_info) {
  LOG_INIT(*test_argv);

  std::vector<std::string> errors;

  LOG_STRING(INFO, &errors) << "LOG_STRING: "
                            << "collected info";
  ASSERT_TRUE(errors.empty());
}

TEST(nolog_log_string, severity_info_null) {
  LOG_INIT(*test_argv);

  std::string output =
      LPP_CAPTURE_STDOUT(LOG_STRING(INFO, nullptr) << "LOG_STRING: "
                                                   << "collected info");
  ASSERT_EQ(output, "");
}

TEST(nolog_log_string, severity_warning) {
  LOG_INIT(*test_argv);

  std::vector<std::string> errors;

  LOG_STRING(WARNING, &errors) << "LOG_STRING: "
                               << "collected warn";
  ASSERT_TRUE(errors.empty());
}

TEST(nolog_log_string, severity_warning_null) {
  LOG_INIT(*test_argv);

  std::string output =
      LPP_CAPTURE_STDOUT(LOG_STRING(WARNING, nullptr) << "LOG_STRING: "
                                                      << "collected warn");
  ASSERT_EQ(output, "");
}

TEST(nolog_log_string, severity_error) {
  LOG_INIT(*test_argv);

  std::vector<std::string> errors;

  LOG_STRING(ERROR, &errors) << "LOG_STRING: "
                             << "collected error";
  ASSERT_TRUE(errors.empty());
}

TEST(nolog_log_string, severity_error_null) {
  LOG_INIT(*test_argv);

  std::string output =
      LPP_CAPTURE_STDOUT(LOG_STRING(ERROR, nullptr) << "LOG_STRING: "
                                                    << "collected error");
  ASSERT_EQ(output, "");
}

TEST(nolog_log_string, severity_fatal) {
  LOG_INIT(*test_argv);

  std::vector<std::string> errors;

  LOG_STRING(ERROR, &errors) << "LOG_STRING: "
                             << "collected error";
  ASSERT_TRUE(errors.empty());
}

TEST(nolog_log_string, severity_fatal_null) {
  LOG_INIT(*test_argv);

  std::string output =
      LPP_CAPTURE_STDOUT(LOG_STRING(FATAL, nullptr) << "LOG_STRING: "
                                                    << "collected fatal");
  ASSERT_EQ(output, "");
}
