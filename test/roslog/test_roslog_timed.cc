//
// Created by 4c3y (acey) on 16.09.22.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <mutex>
#include <async_tests.h>

using namespace lpp::rostest;

std::vector<AsyncTest> generateTests() {
  return {
  {"roslog_timed_lpp_syntax_severity_debug_Test", {debug},[]() { LOG_TIMED(D, 1, "Test" << 123); }, REMOVE_NUMBERS_FROM_STRING, STDOUT},
  {"roslog_timed_lpp_syntax_severity_info_Test", {info, v2::info},[]() { LOG_TIMED(I, 1, "Test" << 123); }, REMOVE_NUMBERS_FROM_STRING, STDOUT},
  {"roslog_timed_lpp_syntax_severity_warning_Test", {warning, v2::warning},[]() { LOG_TIMED(W, 1, "Test" << 123); }, REMOVE_NUMBERS_FROM_STRING, STDERR},
  {"roslog_timed_lpp_syntax_severity_error_Test",{error},[]() { LOG_TIMED(E, 1, "Test" << 123); }, REMOVE_NUMBERS_FROM_STRING, STDERR},
  {"roslog_timed_lpp_syntax_severity_fatal_Test",{fatal},[]() { LOG_TIMED(F, 1, "Test" << 123); }, REMOVE_NUMBERS_FROM_STRING, STDERR},

  {"roslog_timed_glog_syntax_severity_debug_Test",{debug},[]() { DLOG_EVERY_T(INFO, 1) << "Test" << 123; }, REMOVE_NUMBERS_FROM_STRING, STDOUT},
  {"roslog_timed_glog_syntax_severity_info_Test", {info, v2::info},[]() { LOG_EVERY_T(INFO, 1) << "Test" << 123; }, REMOVE_NUMBERS_FROM_STRING, STDOUT},
  {"roslog_timed_glog_syntax_severity_warning_Test", {warning, v2::warning},[]() { LOG_EVERY_T(WARNING, 1) << "Test" << 123; }, REMOVE_NUMBERS_FROM_STRING, STDERR},
  {"roslog_timed_glog_syntax_severity_error_Test",{error},[]() { LOG_EVERY_T(ERROR, 1) << "Test" << 123; }, REMOVE_NUMBERS_FROM_STRING, STDERR},
  {"roslog_timed_glog_syntax_severity_fatal_Test",{fatal},[]() { LOG_EVERY_T(FATAL, 1) << "Test" << 123; }, REMOVE_NUMBERS_FROM_STRING, STDERR},

  {"roslog_timed_ros_syntax_severity_debug_Test", {debug}, []() {ROS_DEBUG_THROTTLE(1, "Test123"); }, REMOVE_NUMBERS_FROM_STRING, STDOUT},
  {"roslog_timed_ros_syntax_severity_debug_stream_Test", {debug}, []() {ROS_DEBUG_STREAM_THROTTLE(1, "Test" << 123); }, REMOVE_NUMBERS_FROM_STRING, STDOUT},
  {"roslog_timed_ros_syntax_severity_info_Test",  {info, v2::info}, []() {ROS_INFO_THROTTLE(1, "Test123"); }, REMOVE_NUMBERS_FROM_STRING, STDOUT},
  {"roslog_timed_ros_syntax_severity_info_stream_Test",  {info, v2::info}, []() {ROS_INFO_STREAM_THROTTLE(1, "Test" << 123); }, REMOVE_NUMBERS_FROM_STRING, STDOUT},
  {"roslog_timed_ros_syntax_severity_warning_Test", {warning, v2::warning}, []() {ROS_WARN_THROTTLE(1, "Test123"); }, REMOVE_NUMBERS_FROM_STRING, STDERR},
  {"roslog_timed_ros_syntax_severity_warning_stream_Test", {warning, v2::warning}, []() {ROS_WARN_STREAM_THROTTLE(1, "Test" << 123); }, REMOVE_NUMBERS_FROM_STRING, STDERR},
  {"roslog_timed_ros_syntax_severity_error_Test", {error}, []() {ROS_ERROR_THROTTLE(1, "Test123"); }, REMOVE_NUMBERS_FROM_STRING, STDERR},
  {"roslog_timed_ros_syntax_severity_error_stream_Test", {error}, []() {ROS_ERROR_STREAM_THROTTLE(1, "Test" << 123); }, REMOVE_NUMBERS_FROM_STRING, STDERR},
  {"roslog_timed_ros_syntax_severity_fatal_Test", {fatal}, []() {ROS_FATAL_THROTTLE(1, "Test123"); }, REMOVE_NUMBERS_FROM_STRING, STDERR},
  {"roslog_timed_ros_syntax_severity_fatal_stream_Test", {fatal}, []() {ROS_FATAL_STREAM_THROTTLE(1, "Test123"); }, REMOVE_NUMBERS_FROM_STRING, STDERR},
  };
}

TEST(roslog_timed, lpp_syntax_floating_point_time) {
  LOG_INIT(*test_argv);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_TIMED(I, 0.1, "Test" << 123));
    if (i % 2 == 0) {
      ASSERT_EQ(removeNumbersFromString(output),  info);
    }
    //sleep 0.1s
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

TEST(roslog_timed, lpp_syntax_severity_debug) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, lpp_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, lpp_syntax_severity_warning) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, lpp_syntax_severity_error) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, lpp_syntax_severity_fatal) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, glog_syntax_severity_debug) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, glog_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, glog_syntax_severity_warning) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, glog_syntax_severity_error) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, glog_syntax_severity_fatal) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, ros_syntax_severity_debug) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, ros_syntax_severity_debug_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, ros_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, ros_syntax_severity_info_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, ros_syntax_severity_warning) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, ros_syntax_severity_warning_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, ros_syntax_severity_error) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, ros_syntax_severity_error_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, ros_syntax_severity_fatal) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(roslog_timed, ros_syntax_severity_fatal_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}