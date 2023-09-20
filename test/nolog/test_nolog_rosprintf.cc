//
// Created by acey on 07.09.23.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

using namespace lpp::testing;

TEST(nolog_rosprintf, severity_debug) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ("", output);
}

TEST(nolog_rosprintf, severity_debug_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ("", output);
}

TEST(nolog_rosprintf, severity_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ("", output);
}

TEST(nolog_rosprintf, severity_info_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ("", output);
}

TEST(nolog_rosprintf, severity_warn) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ("", output);
}

TEST(nolog_rosprintf, severity_warn_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ("", output);
}

TEST(nolog_rosprintf, severity_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ("", output);
}

TEST(nolog_rosprintf, severity_error_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ("", output);
}

TEST(nolog_rosprintf, severity_fatal) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ("", output);
}

TEST(nolog_rosprintf, severity_fatal_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ("", output);
}
