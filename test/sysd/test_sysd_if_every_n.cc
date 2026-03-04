#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

#include "sysd_test_utils.h"

using lpp::sysdtest::entries;

TEST(sysd_if_every_n, glog_syntax_if_every_n_severity_debug) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    DLOG_IF_EVERY_N(INFO, i <= 3, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_if_every_n, glog_syntax_if_every_n_severity_info) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_IF_EVERY_N(INFO, i <= 3, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
}

TEST(sysd_if_every_n, glog_syntax_if_every_n_severity_warning) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_IF_EVERY_N(WARNING, i <= 3, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
}

TEST(sysd_if_every_n, glog_syntax_if_every_n_severity_error) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_IF_EVERY_N(ERROR, i <= 3, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
}

TEST(sysd_if_every_n, glog_syntax_if_every_n_severity_fatal) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_IF_EVERY_N(FATAL, i <= 3, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
}
