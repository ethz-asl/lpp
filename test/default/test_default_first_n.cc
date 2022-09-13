//
// Created by 4c3y (acey) on 12.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(default_LogPolicyFirstN, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST_N(INFO, 3) << "Test" << 123);

    if (i < 3) {

      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_default_first_n.cc"));

      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogPolicyFirstN, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST_N(WARNING, 3) << "Test" << 123);

    if (i < 3) {

      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_default_first_n.cc"));

      ASSERT_EQ(output[0], 'W');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogPolicyFirstN, glog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST_N(ERROR, 3) << "Test" << 123);

    if (i < 3) {

      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_default_first_n.cc"));

      ASSERT_EQ(output[0], 'E');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogPolicyFirstN, lpp_syntax_severity_info) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(I, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, "INFO  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogPolicyFirstN, lpp_syntax_severity_warning) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(W, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, "WARN  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogPolicyFirstN, lpp_syntax_severity_error) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(I, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, "ERROR Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(default_LogPolicyFirstN, lpp_syntax_severity_fatal) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(I, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, "FATAL Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}