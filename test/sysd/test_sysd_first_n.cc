#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

#include "sysd_test_utils.h"

using lpp::sysdtest::entries;

TEST(sysd_LogFirstN, lpp_syntax_severity_debug) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_FIRST(D, 3, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 3);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_LogFirstN, lpp_syntax_severity_info) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_FIRST(I, 3, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 3);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
}

TEST(sysd_LogFirstN, lpp_syntax_severity_warning) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_FIRST(W, 3, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 3);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
}

TEST(sysd_LogFirstN, lpp_syntax_severity_error) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_FIRST(E, 3, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 3);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
}

TEST(sysd_LogFirstN, lpp_syntax_severity_fatal) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_FIRST(F, 3, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 3);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
}

TEST(sysd_LogFirstN, glog_syntax_severity_debug) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    DLOG_FIRST_N(INFO, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 3);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_LogFirstN, glog_syntax_severity_info) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_FIRST_N(INFO, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 3);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
}

TEST(sysd_LogFirstN, glog_syntax_severity_warning) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_FIRST_N(WARNING, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 3);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
}

TEST(sysd_LogFirstN, glog_syntax_severity_error) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_FIRST_N(ERROR, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 3);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
}

TEST(sysd_LogFirstN, glog_syntax_severity_fatal) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_FIRST_N(FATAL, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 3);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
}

TEST(sysd_LogFirstN, ros_debug_once) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_DEBUG_ONCE("Test123");
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_LogFirstN, ros_info_once) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_INFO_ONCE("Test123");
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
}

TEST(sysd_LogFirstN, ros_warn_once) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_WARN_ONCE("Test123");
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
}

TEST(sysd_LogFirstN, ros_error_once) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_ERROR_ONCE("Test123");
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
}

TEST(sysd_LogFirstN, ros_fatal_once) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_FATAL_ONCE("Test123");
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
}
