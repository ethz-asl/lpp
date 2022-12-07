#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

using namespace lpp::rosprintf;
using namespace lpp::testing;

TEST(roslog_rosprintf, ros_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(debug, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_debug_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(debug, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_debug_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(debug, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(info, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_info_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(info, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_info_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(info, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_warn) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(warning, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_warn_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(warning, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_warn_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(warning, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(error, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_error_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(error, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_error_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(error, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(fatal, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_fatal_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(fatal, removeNumbersFromString(output));
}

TEST(roslog_rosprintf, ros_fatal_throttle) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_THROTTLE(1, ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(fatal, removeNumbersFromString(output));
}