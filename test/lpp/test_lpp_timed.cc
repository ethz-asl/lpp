//
// Created by 4c3y (acey) on 16.09.22.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <mutex>
#include <async_tests.h>

std::vector<AsyncTest> generateTests() {
  return {
  {"lpp_timed_lpp_syntax_severity_debug_Test",{"DEBUG Test123\n"},[]() { LOG_TIMED(D, 1, "Test" << 123); }, EQUAL, STDOUT},
  {"lpp_timed_lpp_syntax_severity_info_Test",{"INFO  Test123\n"},[]() { LOG_TIMED(I, 1, "Test" << 123); }, EQUAL, STDOUT},
  {"lpp_timed_lpp_syntax_severity_warning_Test",{"WARN  Test123\n"},[]() { LOG_TIMED(W, 1, "Test" << 123); }, EQUAL, STDOUT},
  {"lpp_timed_lpp_syntax_severity_error_Test",{"ERROR Test123\n"},[]() { LOG_TIMED(E, 1, "Test" << 123); }, EQUAL, STDOUT},
  {"lpp_timed_lpp_syntax_severity_fatal_Test",{"FATAL Test123\n"},[]() { LOG_TIMED(F, 1, "Test" << 123); }, EQUAL, STDOUT},

  {"lpp_timed_glog_syntax_severity_debug_Test",{"DEBUG Test123\n"},[]() { DLOG_EVERY_T(INFO, 1) << "Test" << 123; }, EQUAL, STDOUT},
  {"lpp_timed_glog_syntax_severity_info_Test",{"INFO  Test123\n"},[]() { LOG_EVERY_T(INFO, 1) << "Test" << 123; }, EQUAL, STDOUT},
  {"lpp_timed_glog_syntax_severity_warning_Test",{"WARN  Test123\n"},[]() { LOG_EVERY_T(WARNING, 1) << "Test" << 123; }, EQUAL, STDOUT},
  {"lpp_timed_glog_syntax_severity_error_Test",{"ERROR Test123\n"},[]() { LOG_EVERY_T(ERROR, 1) << "Test" << 123; }, EQUAL, STDOUT},
  {"lpp_timed_glog_syntax_severity_fatal_Test",{"FATAL Test123\n"},[]() { LOG_EVERY_T(FATAL, 1) << "Test" << 123; }, EQUAL, STDOUT},

  {"lpp_timed_ros_syntax_severity_debug_Test", {"DEBUG Test123\n"}, []() {ROS_DEBUG_THROTTLE(1, "Test123"); }, EQUAL, STDOUT},
  {"lpp_timed_ros_syntax_severity_debug_stream_Test", {"DEBUG Test123\n"}, []() {ROS_DEBUG_STREAM_THROTTLE(1, "Test" << 123); }, EQUAL, STDOUT},
  {"lpp_timed_ros_syntax_severity_info_Test", {"INFO  Test123\n"}, []() {ROS_INFO_THROTTLE(1, "Test123"); }, EQUAL, STDOUT},
  {"lpp_timed_ros_syntax_severity_info_stream_Test", {"INFO  Test123\n"}, []() {ROS_INFO_STREAM_THROTTLE(1, "Test" << 123); }, EQUAL, STDOUT},
  {"lpp_timed_ros_syntax_severity_warning_Test", {"WARN  Test123\n"}, []() {ROS_WARN_THROTTLE(1, "Test123"); }, EQUAL, STDOUT},
  {"lpp_timed_ros_syntax_severity_warning_stream_Test", {"WARN  Test123\n"}, []() {ROS_WARN_STREAM_THROTTLE(1, "Test" << 123); }, EQUAL, STDOUT},
  {"lpp_timed_ros_syntax_severity_error_Test", {"ERROR Test123\n"}, []() {ROS_ERROR_THROTTLE(1, "Test123"); }, EQUAL, STDOUT},
  {"lpp_timed_ros_syntax_severity_error_stream_Test", {"ERROR Test123\n"}, []() {ROS_ERROR_STREAM_THROTTLE(1, "Test" << 123); }, EQUAL, STDOUT},
  {"lpp_timed_ros_syntax_severity_fatal_Test", {"FATAL Test123\n"}, []() {ROS_FATAL_THROTTLE(1, "Test123"); }, EQUAL, STDOUT},
  {"lpp_timed_ros_syntax_severity_fatal_stream_Test", {"FATAL Test123\n"}, []() {ROS_FATAL_STREAM_THROTTLE(1, "Test123"); }, EQUAL, STDOUT},
  };
}

TEST(lpp_timed, lpp_syntax_floating_point_time) {
  LOG_INIT(*test_argv);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_TIMED(I, 0.1, "Test" << 123));
    if (i % 2 == 0) {
      ASSERT_EQ(output, "INFO  Test123\n");
    }
    //sleep 0.1s
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

TEST(lpp_timed, lpp_syntax_severity_debug) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, lpp_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, lpp_syntax_severity_warning) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, lpp_syntax_severity_error) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, lpp_syntax_severity_fatal) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, glog_syntax_severity_debug) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, glog_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, glog_syntax_severity_warning) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, glog_syntax_severity_error) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, glog_syntax_severity_fatal) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, ros_syntax_severity_debug) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, ros_syntax_severity_debug_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, ros_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, ros_syntax_severity_info_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, ros_syntax_severity_warning) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, ros_syntax_severity_warning_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, ros_syntax_severity_error) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, ros_syntax_severity_error_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, ros_syntax_severity_fatal) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(lpp_timed, ros_syntax_severity_fatal_stream) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}