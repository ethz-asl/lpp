//
// Created by 4c3y (acey) on 12.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

using namespace lpp::rosprintf;
using namespace lpp::testing;

TEST(default_rosprintf, ros_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG(ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = debug == removeNumbersFromString(output) || v2::debug == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_debug_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = debug == removeNumbersFromString(output) || v2::debug == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_debug_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = debug == removeNumbersFromString(output) || v2::debug == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO(ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = info == removeNumbersFromString(output) || v2::info == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_info_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = info == removeNumbersFromString(output) || v2::info == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_info_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = info == removeNumbersFromString(output) || v2::info == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_warn) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN(ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = warning == removeNumbersFromString(output) || v2::warning == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_warn_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = warning == removeNumbersFromString(output) || v2::warning == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_warn_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = warning == removeNumbersFromString(output) || v2::warning == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR(ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = error == removeNumbersFromString(output) || v2::error == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_error_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = error == removeNumbersFromString(output) || v2::error == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_error_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = error == removeNumbersFromString(output) || v2::error == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL(ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = fatal == removeNumbersFromString(output) || v2::fatal == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_fatal_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = fatal == removeNumbersFromString(output) || v2::fatal == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}

TEST(default_rosprintf, ros_fatal_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  bool isEqual = fatal == removeNumbersFromString(output) || v2::fatal == removeNumbersFromString(output);
  EXPECT_TRUE(isEqual);
}