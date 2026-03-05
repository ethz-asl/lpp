#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

#include "sysd_test_utils.h"

using lpp::sysdtest::entries;

TEST(sysd_basic, lpp_syntax_severity_debug) {
  lpp::sysdtest::init(*test_argv);
  LOG(D, "Test" << 123);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
  ASSERT_EQ(entries().at(0).message, "Test123");
  ASSERT_EQ(entries().at(0).identifier, lpp::internal::filenameFromPath(*test_argv));
}

TEST(sysd_basic, lpp_syntax_severity_info) {
  lpp::sysdtest::init(*test_argv);
  LOG(I, "Test" << 123);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
  ASSERT_EQ(entries().at(0).message, "Test123");
}

TEST(sysd_basic, lpp_syntax_severity_warning) {
  lpp::sysdtest::init(*test_argv);
  LOG(W, "Test" << 123);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
  ASSERT_EQ(entries().at(0).message, "Test123");
}

TEST(sysd_basic, lpp_syntax_severity_error) {
  lpp::sysdtest::init(*test_argv);
  LOG(E, "Test" << 123);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
  ASSERT_EQ(entries().at(0).message, "Test123");
}

TEST(sysd_basic, lpp_syntax_severity_fatal) {
  lpp::sysdtest::init(*test_argv);
  LOG(F, "Test" << 123);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
  ASSERT_EQ(entries().at(0).message, "Test123");
}

TEST(sysd_basic, glog_syntax_severity_debug) {
  lpp::sysdtest::init(*test_argv);
  DLOG(INFO) << "Test";
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, glog_syntax_severity_info) {
  lpp::sysdtest::init(*test_argv);
  LOG(INFO) << "Test123";
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
  ASSERT_EQ(entries().at(0).message, "Test123");
}

TEST(sysd_basic, glog_syntax_severity_warning) {
  lpp::sysdtest::init(*test_argv);
  LOG(WARNING) << "Test";
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, glog_syntax_severity_error) {
  lpp::sysdtest::init(*test_argv);
  LOG(ERROR) << "Test";
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, glog_syntax_severity_fatal) {
  lpp::sysdtest::init(*test_argv);
  LOG(FATAL) << "Test";
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, roslog_syntax_severity_debug) {
  lpp::sysdtest::init(*test_argv);
  ROS_DEBUG("Test");
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, roslog_syntax_severity_debug_stream) {
  lpp::sysdtest::init(*test_argv);
  ROS_DEBUG_STREAM("Test");
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, roslog_syntax_severity_info) {
  lpp::sysdtest::init(*test_argv);
  ROS_INFO("Test");
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, roslog_syntax_severity_info_stream) {
  lpp::sysdtest::init(*test_argv);
  ROS_INFO_STREAM("Test");
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, roslog_syntax_severity_warn) {
  lpp::sysdtest::init(*test_argv);
  ROS_WARN("Test");
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, roslog_syntax_severity_warn_stream) {
  lpp::sysdtest::init(*test_argv);
  ROS_WARN_STREAM("Test");
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, roslog_syntax_severity_error) {
  lpp::sysdtest::init(*test_argv);
  ROS_ERROR("Test");
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, roslog_syntax_severity_error_stream) {
  lpp::sysdtest::init(*test_argv);
  ROS_ERROR_STREAM("Test");
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, roslog_syntax_severity_fatal) {
  lpp::sysdtest::init(*test_argv);
  ROS_FATAL("Test");
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
  ASSERT_EQ(entries().at(0).message, "Test");
}

TEST(sysd_basic, roslog_syntax_severity_fatal_stream) {
  lpp::sysdtest::init(*test_argv);
  ROS_FATAL_STREAM("Test");
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
  ASSERT_EQ(entries().at(0).message, "Test");
}
