//
// Created by 4c3y (acey) on 08.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

using namespace lpp::testing;

TEST(lpp_rosprintf, ros_debug) {
    LOG_INIT(*test_argv);

    std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG(ERROR_MESSAGE, 3.3, 5.5));
    ASSERT_EQ(output, "DEBUG " + EXPECTED_ERROR_MESSAGE);
}

TEST(lpp_rosprintf, ros_debug_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(output, "DEBUG " + EXPECTED_ERROR_MESSAGE);
}

TEST(lpp_rosprintf, ros_info) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(output, "INFO  " + EXPECTED_ERROR_MESSAGE);
}

TEST(lpp_rosprintf, ros_info_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(output, "INFO  " + EXPECTED_ERROR_MESSAGE);
}

TEST(lpp_rosprintf, ros_warn) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_WARN(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(output, "WARN  " + EXPECTED_ERROR_MESSAGE);
}

TEST(lpp_rosprintf, ros_warn_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_WARN_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(output, "WARN  " + EXPECTED_ERROR_MESSAGE);
}

TEST(lpp_rosprintf, ros_error) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_ERROR(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(output, "ERROR " + EXPECTED_ERROR_MESSAGE);
}

TEST(lpp_rosprintf, ros_error_once) {
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_ERROR_ONCE(ERROR_MESSAGE, 3.3, 5.5));
  ASSERT_EQ(output, "ERROR " + EXPECTED_ERROR_MESSAGE);
}

TEST(lpp_rosprintf, ros_fatal) {
    LOG_INIT(*test_argv);

    std::string output = LPP_CAPTURE_STDOUT(ROS_FATAL(ERROR_MESSAGE, 3.3, 5.5));
    ASSERT_EQ(output, "FATAL " + EXPECTED_ERROR_MESSAGE);
}

TEST(lpp_rosprintf, ros_fatal_once) {
    LOG_INIT(*test_argv);

    std::string output = LPP_CAPTURE_STDOUT(ROS_FATAL_ONCE(ERROR_MESSAGE, 3.3, 5.5));
    ASSERT_EQ(output, "FATAL " + EXPECTED_ERROR_MESSAGE);
}