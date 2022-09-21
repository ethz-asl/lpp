//
// Created by 4c3y (acey) on 16.09.22.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <mutex>
#include <async_tests.h>

std::vector<AsyncTest> generateTests() {
  return {
  {"default_timed_lpp_syntax_severity_info_Test","INFO  Test123\n",[]() { LOG_TIMED(I, 1, "Test123"); }, CompareType::EQUAL},
  {"default_timed_glog_syntax_severity_info_Test","INFO  Test123\n",[]() { LOG_EVERY_T(INFO, 1) << "Test" << 123; }, CompareType::EQUAL},
  {"default_timed_ros_syntax_severity_info_Test", "Test123", []() {ROS_INFO_THROTTLE(1, "Test123"); }, CompareType::IS_SUBSTRING}
  };
}

TEST(default_timed, lpp_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(default_timed, glog_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}

TEST(default_timed, ros_syntax_severity_info) {
  ASSERT_TRUE(TestResult::getInstance().get(GET_CLASS_NAME(*this, nullptr)));
}