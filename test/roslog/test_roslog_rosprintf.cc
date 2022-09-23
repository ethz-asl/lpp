#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

using namespace lpp;

TEST(roslog_rosprintf, ros_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(rosprintf::debug, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_debug_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(rosprintf::debug, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(rosprintf::info, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_info_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(rosprintf::info, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_warn) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(rosprintf::warning, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_warn_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(rosprintf::warning, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(rosprintf::error, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_error_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(rosprintf::error, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(rosprintf::fatal, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_fatal_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(rosprintf::fatal, removeNumbersFromString(output));
}