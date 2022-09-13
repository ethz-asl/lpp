//
// Created by 4c3y (acey) on 08.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(glog_LogEveryN, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY(D, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_every_n.cc"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogEveryN, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY(I, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_every_n.cc"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogEveryN, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY(W, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_every_n.cc"));
      ASSERT_EQ(output[0], 'W');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogEveryN, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY(E, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_every_n.cc"));
      ASSERT_EQ(output[0], 'E');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogEveryN, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(DLOG_EVERY_N(INFO, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_every_n.cc"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogEveryN, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY_N(INFO, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_every_n.cc"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogEveryN, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY_N(WARNING, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_every_n.cc"));
      ASSERT_EQ(output[0], 'W');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogEveryN, glog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY_N(ERROR, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_every_n.cc"));
      ASSERT_EQ(output[0], 'E');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}