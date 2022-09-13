//
// Created by 4c3y (acey) on 08.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(glog_rosprintf, ros_info) {
  LOG_INIT(*test_argv);

  std::string c = LPP_CAPTURE_STDERR(ROS_INFO("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5));

  ASSERT_TRUE(isSubstring(c, "Base angle (3.300000) is less than the minimum angle (5.500000)\n"));
  ASSERT_TRUE(c[0] == 'I');
}

TEST(glog_rosprintf, ros_info_once) {
  LOG_INIT(*test_argv);

  std::string c = LPP_CAPTURE_STDERR(ROS_INFO_ONCE("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5));

  ASSERT_TRUE(isSubstring(c, "Base angle (3.300000) is less than the minimum angle (5.500000)\n"));
  ASSERT_TRUE(c[0] == 'I');
}

TEST(glog_rosprintf, ros_warn) {
  LOG_INIT(*test_argv);

  std::string c = LPP_CAPTURE_STDERR(ROS_WARN("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5));

  ASSERT_TRUE(isSubstring(c, "Base angle (3.300000) is less than the minimum angle (5.500000)\n"));
  ASSERT_TRUE(c[0] == 'W');
}

TEST(glog_rosprintf, ros_warn_once) {
  LOG_INIT(*test_argv);

  std::string c = LPP_CAPTURE_STDERR(ROS_WARN_ONCE("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5));

  ASSERT_TRUE(isSubstring(c, "Base angle (3.300000) is less than the minimum angle (5.500000)\n"));
  ASSERT_TRUE(c[0] == 'W');
}

TEST(glog_rosprintf, ros_error) {
  LOG_INIT(*test_argv);

  std::string c = LPP_CAPTURE_STDERR(ROS_ERROR("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5));

  ASSERT_TRUE(isSubstring(c, "Base angle (3.300000) is less than the minimum angle (5.500000)\n"));
  ASSERT_TRUE(c[0] == 'E');
}

TEST(glog_rosprintf, ros_error_once) {
  LOG_INIT(*test_argv);

  std::string c = LPP_CAPTURE_STDERR(ROS_ERROR_ONCE("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5));

  ASSERT_TRUE(isSubstring(c, "Base angle (3.300000) is less than the minimum angle (5.500000)\n"));
  ASSERT_TRUE(c[0] == 'E');
}