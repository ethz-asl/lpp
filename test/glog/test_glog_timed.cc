//
// Created by 4c3y (acey) on 16.09.22.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <mutex>
#include <async_tests.h>

std::vector<AsyncTest> generateTests() {
  return {
  {"glog_timed_lpp_syntax_severity_debug_Test",{"Test123"},[]() { LOG_TIMED(D, 1, "Test" << 123); }, IS_SUBSTRING, STDERR},
  {"glog_timed_lpp_syntax_severity_info_Test",{"Test123"},[]() { LOG_TIMED(I, 1, "Test" << 123); }, IS_SUBSTRING, STDERR},
  {"glog_timed_lpp_syntax_severity_warning_Test",{"Test123"},[]() { LOG_TIMED(W, 1, "Test" << 123); }, IS_SUBSTRING, STDERR},
  {"glog_timed_lpp_syntax_severity_error_Test",{"Test123"},[]() { LOG_TIMED(E, 1, "Test" << 123); }, IS_SUBSTRING, STDERR},
  //{"glog_timed_lpp_syntax_severity_fatal_Test",{"Test123",[]() { LOG_TIMED(F, 1, "Test" << 123); }, IS_SUBSTRING, STDERR},

  {"glog_timed_glog_syntax_severity_debug_Test",{"Test123"},[]() { DLOG_EVERY_T(INFO, 1) << "Test" << 123; }, IS_SUBSTRING, STDERR},
  {"glog_timed_glog_syntax_severity_info_Test",{"Test123"},[]() { LOG_EVERY_T(INFO, 1) << "Test" << 123; }, IS_SUBSTRING, STDERR},
  {"glog_timed_glog_syntax_severity_warning_Test",{"Test123"},[]() { LOG_EVERY_T(WARNING, 1) << "Test" << 123; }, IS_SUBSTRING, STDERR},
  {"glog_timed_glog_syntax_severity_error_Test",{"Test123"},[]() { LOG_EVERY_T(ERROR, 1) << "Test" << 123; }, IS_SUBSTRING, STDERR},
  //{"glog_timed_glog_syntax_severity_fatal_Test",{"Test123"},[]() { LOG_EVERY_T(FATAL, 1) << "Test" << 123; }, IS_SUBSTRING, STDERR},

  {"glog_timed_ros_syntax_severity_debug_Test", {"Test123"}, []() {ROS_DEBUG_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDERR},
  {"glog_timed_ros_syntax_severity_debug_stream_Test", {"Test123"}, []() {ROS_DEBUG_STREAM_THROTTLE(1, "Test" << 123); }, IS_SUBSTRING, STDERR},
  {"glog_timed_ros_syntax_severity_info_Test", {"Test123"}, []() {ROS_INFO_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDERR},
  {"glog_timed_ros_syntax_severity_info_stream_Test", {"Test123"}, []() {ROS_INFO_STREAM_THROTTLE(1, "Test" << 123); }, IS_SUBSTRING, STDERR},
  {"glog_timed_ros_syntax_severity_warning_Test", {"Test123"}, []() {ROS_WARN_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDERR},
  {"glog_timed_ros_syntax_severity_warning_stream_Test", {"Test123"}, []() {ROS_WARN_STREAM_THROTTLE(1, "Test" << 123); }, IS_SUBSTRING, STDERR},
  {"glog_timed_ros_syntax_severity_error_Test", {"Test123"}, []() {ROS_ERROR_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDERR},
  {"glog_timed_ros_syntax_severity_error_stream_Test", {"Test123"}, []() {ROS_ERROR_STREAM_THROTTLE(1, "Test" << 123); }, IS_SUBSTRING, STDERR},
  //{"glog_timed_ros_syntax_severity_fatal_Test", {"Test123"}, []() {ROS_FATAL_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDERR},
  //{"glog_timed_ros_syntax_severity_fatal_stream_Test", {"Test123"}, []() {ROS_FATAL_STREAM_THROTTLE(1, "Test123"); }, IS_SUBSTRING, STDERR},
  };
}

TEST(glog_timed, lpp_syntax_floating_point_time) {
  LOG_INIT(*test_argv);
  for (int i = 0; i < 5; i++) {

    std::string output = LPP_CAPTURE_STDERR(LOG_TIMED(I, 0.1, "Test" << 123));
    if (i % 2 == 0) {
      ASSERT_TRUE(isSubstring(output, "I"));
      ASSERT_TRUE(isSubstring(output, "Test123"));
    }
    //sleep 0.1s
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

TEST(glog_timed, lpp_syntax_severity_debug) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, lpp_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, lpp_syntax_severity_warning) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, lpp_syntax_severity_error) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, lpp_syntax_severity_fatal) {
  //ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, glog_syntax_severity_debug) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, glog_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, glog_syntax_severity_warning) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, glog_syntax_severity_error) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, glog_syntax_severity_fatal) {
  //ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, ros_syntax_severity_debug) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, ros_syntax_severity_debug_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, ros_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, ros_syntax_severity_info_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, ros_syntax_severity_warning) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, ros_syntax_severity_warning_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, ros_syntax_severity_error) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, ros_syntax_severity_error_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, ros_syntax_severity_fatal) {
  //ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(glog_timed, ros_syntax_severity_fatal_stream) {
  //ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}