//
// Created by 4c3y (acey) on 08.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(lpp_rosprintf, ros_info) {
  LOG_INIT(*test_argv);

  testing::internal::CaptureStdout();
  ROS_INFO("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5);
  std::string c = testing::internal::GetCapturedStdout();

  ASSERT_EQ(c, "INFO  Base angle (3.300000) is less than the minimum angle (5.500000)\n");
}

TEST(lpp_rosprintf, ros_info_once) {
  LOG_INIT(*test_argv);

  testing::internal::CaptureStdout();
  ROS_INFO_ONCE("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5);
  std::string c = testing::internal::GetCapturedStdout();

  ASSERT_EQ(c, "INFO  Base angle (3.300000) is less than the minimum angle (5.500000)\n");

}

TEST(lpp_rosprintf, ros_warn) {
  LOG_INIT(*test_argv);

  testing::internal::CaptureStdout();
  ROS_WARN("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5);
  std::string c = testing::internal::GetCapturedStdout();

  ASSERT_EQ(c, "WARN  Base angle (3.300000) is less than the minimum angle (5.500000)\n");
}

TEST(lpp_rosprintf, ros_warn_once) {
  LOG_INIT(*test_argv);

  testing::internal::CaptureStdout();
  ROS_WARN_ONCE("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5);
  std::string c = testing::internal::GetCapturedStdout();

  ASSERT_EQ(c, "WARN  Base angle (3.300000) is less than the minimum angle (5.500000)\n");
}

TEST(lpp_rosprintf, ros_error) {
  LOG_INIT(*test_argv);

  testing::internal::CaptureStdout();
  ROS_ERROR("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5);
  std::string c = testing::internal::GetCapturedStdout();

  ASSERT_EQ(c, "ERROR Base angle (3.300000) is less than the minimum angle (5.500000)\n");
}

TEST(lpp_rosprintf, ros_error_once) {
  LOG_INIT(*test_argv);

  testing::internal::CaptureStdout();
  ROS_ERROR_ONCE("Base angle (%f) is less than the minimum angle (%f)", 3.3, 5.5);
  std::string c = testing::internal::GetCapturedStdout();

  ASSERT_EQ(c, "ERROR Base angle (3.300000) is less than the minimum angle (5.500000)\n");
}