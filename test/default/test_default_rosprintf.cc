//
// Created by 4c3y (acey) on 12.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>
inline static constexpr const char *ERROR_MESSAGE = "Base angle (%f) is less than the minimum angle (%f)";

namespace eq {
inline static const std::string debug = "\x1B[m[DEBUG] [.]: Base angle (.) is less than the minimum angle (.)\x1B[m\n";
inline static const std::string info = "\x1B[m[ INFO] [.]: Base angle (.) is less than the minimum angle (.)\x1B[m\n";
inline static const std::string
    warning = "\x1B[m[ WARN] [.]: Base angle (.) is less than the minimum angle (.)\x1B[m\n";
inline static const std::string error = "\x1B[m[ERROR] [.]: Base angle (.) is less than the minimum angle (.)\x1B[m\n";
inline static const std::string fatal = "\x1B[m[FATAL] [.]: Base angle (.) is less than the minimum angle (.)\x1B[m\n";
}

TEST(default_rosprintf, ros_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::debug, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_debug_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::debug, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_debug_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::debug, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::info, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_info_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::info, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_info_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::info, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_warn) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::warning, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_warn_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::warning, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_warn_throttle) {
  LOG_INIT(*test_argv);
  std::string output = LPP_CAPTURE_STDERR(ROS_WARN_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::warning, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::error, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_error_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::error, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_error_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::error, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::fatal, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_fatal_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::fatal, removeNumbersFromString(output));
}

TEST(default_rosprintf, ros_fatal_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(eq::fatal, removeNumbersFromString(output));
}