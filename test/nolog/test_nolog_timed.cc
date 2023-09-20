//
// Created by acey on 07.09.23.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <mutex>
#include <async_tests.h>

std::vector<AsyncTest> generateTests() {
  return {
      {"nolog_timed_lpp_syntax_severity_debug_Test","",[]() { LOG_TIMED(D, 1, "Test" << 123); }, EQUAL, STDOUT},
      {"nolog_timed_lpp_syntax_severity_info_Test","",[]() { LOG_TIMED(I, 1, "Test" << 123); }, EQUAL, STDOUT},
      {"nolog_timed_lpp_syntax_severity_warning_Test","",[]() { LOG_TIMED(W, 1, "Test" << 123); }, EQUAL, STDOUT},
      {"nolog_timed_lpp_syntax_severity_error_Test","",[]() { LOG_TIMED(E, 1, "Test" << 123); }, EQUAL, STDOUT},
      {"nolog_timed_lpp_syntax_severity_fatal_Test","",[]() { LOG_TIMED(F, 1, "Test" << 123); }, EQUAL, STDOUT},

      {"nolog_timed_glog_syntax_severity_debug_Test","",[]() { DLOG_EVERY_T(INFO, 1) << "Test" << 123; }, IS_SUBSTRING, STDERR},
      {"nolog_timed_glog_syntax_severity_info_Test","",[]() { LOG_EVERY_T(INFO, 1) << "Test" << 123; }, IS_SUBSTRING, STDOUT},
      {"nolog_timed_glog_syntax_severity_warning_Test","",[]() { LOG_EVERY_T(WARNING, 1) << "Test" << 123; }, IS_SUBSTRING, STDOUT},
      {"nolog_timed_glog_syntax_severity_error_Test","",[]() { LOG_EVERY_T(ERROR, 1) << "Test" << 123; }, IS_SUBSTRING, STDOUT},
      {"nolog_timed_glog_syntax_severity_fatal_Test","",[]() { LOG_EVERY_T(FATAL, 1) << "Test" << 123; }, IS_SUBSTRING, STDOUT},

      {"nolog_timed_ros_syntax_severity_debug_Test", "", []() {ROS_DEBUG_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDOUT},
      {"nolog_timed_ros_syntax_severity_debug_stream_Test", "", []() {ROS_DEBUG_STREAM_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDOUT},
      {"nolog_timed_ros_syntax_severity_info_Test", "", []() {ROS_INFO_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDOUT},
      {"nolog_timed_ros_syntax_severity_info_stream_Test", "", []() {ROS_INFO_STREAM_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDOUT},
      {"nolog_timed_ros_syntax_severity_warning_Test", "", []() {ROS_WARN_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDERR},
      {"nolog_timed_ros_syntax_severity_warning_stream_Test", "", []() {ROS_WARN_STREAM_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDERR},
      {"nolog_timed_ros_syntax_severity_error_Test", "", []() {ROS_ERROR_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDERR},
      {"nolog_timed_ros_syntax_severity_error_stream_Test", "", []() {ROS_ERROR_STREAM_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDERR},
      {"nolog_timed_ros_syntax_severity_fatal_Test", "", []() {ROS_FATAL_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDERR},
      {"nolog_timed_ros_syntax_severity_fatal_stream_Test", "", []() {ROS_FATAL_STREAM_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDERR},
  };
}

TEST(nolog_timed, lpp_syntax_severity_debug) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, lpp_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, lpp_syntax_severity_warning) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, lpp_syntax_severity_error) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, lpp_syntax_severity_fatal) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, glog_syntax_severity_debug) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, glog_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, glog_syntax_severity_warning) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, glog_syntax_severity_error) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, glog_syntax_severity_fatal) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, ros_syntax_severity_debug) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, ros_syntax_severity_debug_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, ros_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, ros_syntax_severity_info_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, ros_syntax_severity_warning) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, ros_syntax_severity_warning_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, ros_syntax_severity_error) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, ros_syntax_severity_error_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, ros_syntax_severity_fatal) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(nolog_timed, ros_syntax_severity_fatal_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}