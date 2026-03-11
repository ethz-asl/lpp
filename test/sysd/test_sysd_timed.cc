#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

#include <thread>

#include "sysd_test_utils.h"

using lpp::sysdtest::entries;

TEST(sysd_timed, lpp_syntax_floating_point_time) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_TIMED(I, 0.1, "Test" << 123);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  ASSERT_GE(entries().size(), 2);
  ASSERT_LE(entries().size(), 3);
}

TEST(sysd_timed, lpp_syntax_severity_debug) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_TIMED(D, 1, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_timed, lpp_syntax_severity_info) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_TIMED(I, 1, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
}

TEST(sysd_timed, lpp_syntax_severity_warning) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_TIMED(W, 1, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
}

TEST(sysd_timed, lpp_syntax_severity_error) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_TIMED(E, 1, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
}

TEST(sysd_timed, lpp_syntax_severity_fatal) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_TIMED(F, 1, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
}

TEST(sysd_timed, glog_syntax_severity_debug) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    DLOG_EVERY_T(INFO, 1) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_timed, glog_syntax_severity_info) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY_T(INFO, 1) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
}

TEST(sysd_timed, glog_syntax_severity_warning) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY_T(WARNING, 1) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
}

TEST(sysd_timed, glog_syntax_severity_error) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY_T(ERROR, 1) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
}

TEST(sysd_timed, glog_syntax_severity_fatal) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    LOG_EVERY_T(FATAL, 1) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
}

TEST(sysd_timed, ros_syntax_severity_debug) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_DEBUG_THROTTLE(1, "Test123");
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_timed, ros_syntax_severity_debug_stream) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_DEBUG_STREAM_THROTTLE(1, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_timed, ros_syntax_severity_info) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_INFO_THROTTLE(1, "Test123");
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
}

TEST(sysd_timed, ros_syntax_severity_info_stream) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_INFO_STREAM_THROTTLE(1, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
}

TEST(sysd_timed, ros_syntax_severity_warning) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_WARN_THROTTLE(1, "Test123");
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
}

TEST(sysd_timed, ros_syntax_severity_warning_stream) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_WARN_STREAM_THROTTLE(1, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
}

TEST(sysd_timed, ros_syntax_severity_error) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_ERROR_THROTTLE(1, "Test123");
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
}

TEST(sysd_timed, ros_syntax_severity_error_stream) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_ERROR_STREAM_THROTTLE(1, "Test" << 123);
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
}

TEST(sysd_timed, ros_syntax_severity_fatal) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_FATAL_THROTTLE(1, "Test123");
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
}

TEST(sysd_timed, ros_syntax_severity_fatal_stream) {
  lpp::sysdtest::init(*test_argv);
  for (int i = 0; i < 5; i++) {
    ROS_FATAL_STREAM_THROTTLE(1, "Test123");
  }
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
}
