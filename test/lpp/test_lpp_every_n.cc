//
// Created by 4c3y (acey) on 06.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(lpp_LogEveryN, lpp_syntax_severity_debug) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(D, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, "DEBUG Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogEveryN, lpp_syntax_severity_info) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(I, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, "INFO  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogEveryN, lpp_syntax_severity_warning) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(W, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, "WARN  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogEveryN, lpp_syntax_severity_error) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(E, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, "ERROR Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogEveryN, lpp_syntax_severity_fatal) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(F, 3, "Test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, "FATAL Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogEveryN, glog_syntax_severity_debug) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(DLOG_EVERY_N(INFO, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, "DEBUG Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogEveryN, glog_syntax_severity_info) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY_N(INFO, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, "INFO  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogEveryN, glog_syntax_severity_warning) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY_N(WARNING, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, "WARN  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogEveryN, glog_syntax_severity_error) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY_N(ERROR, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, "ERROR Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogEveryN, glog_syntax_severity_fatal) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY_N(FATAL, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, "FATAL Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}