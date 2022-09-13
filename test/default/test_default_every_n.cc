//
// Created by 4c3y (acey) on 12.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(default_LogEveryN, lpp_syntax_severity_debug) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(D, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, "DEBUG Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogEveryN, lpp_syntax_severity_info) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(I, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, "INFO  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogEveryN, lpp_syntax_severity_warning) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(W, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, "WARN  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogEveryN, lpp_syntax_severity_error) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(E, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, "ERROR Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogEveryN, lpp_syntax_severity_fatal) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(F, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, "FATAL Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogPolicyEveryN, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(DLOG_EVERY_N(INFO, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogPolicyEveryN, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY_N(INFO, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogPolicyEveryN, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY_N(WARNING, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogPolicyEveryN, glog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY_N(ERROR, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
    } else {
      ASSERT_EQ(output, "");
    }
  }
}