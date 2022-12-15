//
// Created by 4c3y (acey) on 15.12.22.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

TEST(default_if_every_n, glog_syntax_if_every_n_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    DLOG_IF_EVERY_N(INFO, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStderr();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, LPP_FILENAME));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_if_every_n, glog_syntax_if_every_n_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_IF_EVERY_N(INFO, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStderr();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, LPP_FILENAME));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_if_every_n, glog_syntax_if_every_n_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_IF_EVERY_N(WARNING, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStderr();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, LPP_FILENAME));
      ASSERT_EQ(output[0], 'W');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_if_every_n, glog_syntax_if_every_n_severity_error) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStderr();
    LOG_IF_EVERY_N(ERROR, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStderr();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, LPP_FILENAME));
      ASSERT_EQ(output[0], 'E');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_if_every_n, glog_syntax_if_every_n_severity_fatal) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::function<void()> fn = []() {
    for (int i = 0; i < 5; i++) {
      testing::internal::CaptureStderr();
      LOG_IF_EVERY_N(FATAL, i <= 3, 3) << "Test" << 123;
      std::string output = testing::internal::GetCapturedStderr();
    }
  };

  ASSERT_TRUE(checkAbort(fn));
}