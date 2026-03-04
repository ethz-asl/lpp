#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

#include "sysd_test_utils.h"

using namespace lpp::testing;
using lpp::sysdtest::entries;

TEST(sysd_rosprintf, ros_debug) {
  lpp::sysdtest::init(*test_argv);
  ROS_DEBUG(ERROR_MESSAGE, 3.3, 5.5);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
  ASSERT_EQ(entries().at(0).message, "Base angle (3.300000) is less than the minimum angle (5.500000)");
}

TEST(sysd_rosprintf, ros_debug_once) {
  lpp::sysdtest::init(*test_argv);
  ROS_DEBUG_ONCE(ERROR_MESSAGE, 3.3, 5.5);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_rosprintf, ros_info) {
  lpp::sysdtest::init(*test_argv);
  ROS_INFO(ERROR_MESSAGE, 3.3, 5.5);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
}

TEST(sysd_rosprintf, ros_info_once) {
  lpp::sysdtest::init(*test_argv);
  ROS_INFO_ONCE(ERROR_MESSAGE, 3.3, 5.5);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
}

TEST(sysd_rosprintf, ros_warn) {
  lpp::sysdtest::init(*test_argv);
  ROS_WARN(ERROR_MESSAGE, 3.3, 5.5);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
}

TEST(sysd_rosprintf, ros_warn_once) {
  lpp::sysdtest::init(*test_argv);
  ROS_WARN_ONCE(ERROR_MESSAGE, 3.3, 5.5);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::WARN);
}

TEST(sysd_rosprintf, ros_error) {
  lpp::sysdtest::init(*test_argv);
  ROS_ERROR(ERROR_MESSAGE, 3.3, 5.5);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
}

TEST(sysd_rosprintf, ros_error_once) {
  lpp::sysdtest::init(*test_argv);
  ROS_ERROR_ONCE(ERROR_MESSAGE, 3.3, 5.5);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::ERROR);
}

TEST(sysd_rosprintf, ros_fatal) {
  lpp::sysdtest::init(*test_argv);
  ROS_FATAL(ERROR_MESSAGE, 3.3, 5.5);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
}

TEST(sysd_rosprintf, ros_fatal_once) {
  lpp::sysdtest::init(*test_argv);
  ROS_FATAL_ONCE(ERROR_MESSAGE, 3.3, 5.5);
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::FATAL);
}
