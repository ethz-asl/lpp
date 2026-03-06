#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

#include "sysd_test_utils.h"

using lpp::sysdtest::entries;

TEST(sysd_LogEveryN, lpp_syntax_severity_debug) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY(D, 3, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_LogEveryN, lpp_syntax_severity_info) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY(I, 3, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
}

TEST(sysd_LogEveryN, lpp_syntax_severity_warning) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY(W, 3, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
}

TEST(sysd_LogEveryN, lpp_syntax_severity_error) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY(E, 3, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
}

TEST(sysd_LogEveryN, lpp_syntax_severity_fatal) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY(F, 3, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
}

TEST(sysd_LogEveryN, glog_syntax_severity_debug) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    DLOG_EVERY_N(INFO, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_LogEveryN, glog_syntax_severity_info) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY_N(INFO, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
}

TEST(sysd_LogEveryN, glog_syntax_severity_warning) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY_N(WARNING, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
}

TEST(sysd_LogEveryN, glog_syntax_severity_error) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY_N(ERROR, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
}

TEST(sysd_LogEveryN, glog_syntax_severity_fatal) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY_N(FATAL, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
}
