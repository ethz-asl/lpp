//
// Created by 4c3y (acey) on 06.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(LPP_LogPolicyFirstN, lpp_syntax) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(I, 3, "Test"));

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test"));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(LPP_LogPolicyFirstN, glog_syntax) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST_N(INFO, 3) << "Test");

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test"));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(LPP_LogPolicyFirstN, ros_info_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {

    testing::internal::CaptureStdout();
    ROS_INFO_ONCE("Test123");
    std::string output = testing::internal::GetCapturedStdout();
    if (i == 0) {
      ASSERT_EQ(output, "INFO  Test123\n");
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(LPP_LogPolicyFirstN, ros_warn_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {

    testing::internal::CaptureStdout();
    ROS_WARN_ONCE("Test123");
    std::string output = testing::internal::GetCapturedStdout();
    if (i == 0) {
      ASSERT_EQ(output, "WARN  Test123\n");
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(LPP_LogPolicyFirstN, ros_error_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {

    testing::internal::CaptureStdout();
    ROS_ERROR_ONCE("Test123");
    std::string output = testing::internal::GetCapturedStdout();
    if (i == 0) {
      ASSERT_EQ(output, "ERROR Test123\n");
    } else {
      ASSERT_EQ("", output);
    }
  }
}

