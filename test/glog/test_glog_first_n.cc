//
// Created by 4c3y (acey) on 08.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(glog_LogPolicyFirstN, lpp_syntax) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_FIRST(I, 3, "Test" << 123);
    std::string output = testing::internal::GetCapturedStderr();

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogPolicyFirstN, glog_syntax) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_FIRST_N(INFO, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStderr();

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogPolicyFirstN, ros_info_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    ROS_INFO_ONCE("Test123");
    std::string output = testing::internal::GetCapturedStderr();
    if (i == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(glog_LogPolicyFirstN, ros_warn_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    ROS_WARN_ONCE("Test123");
    std::string output = testing::internal::GetCapturedStderr();
    if (i == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_EQ(output[0], 'W');
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(glog_LogPolicyFirstN, ros_error_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    ROS_ERROR_ONCE("Test123");
    std::string output = testing::internal::GetCapturedStderr();
    if (i == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_EQ(output[0], 'E');
    } else {
      ASSERT_EQ("", output);
    }
  }
}