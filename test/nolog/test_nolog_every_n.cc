//
// Created by acey on 06.09.23.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

TEST(nolog_LogEveryN, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(D, 3, "Test" << 123));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogEveryN, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(I, 3, "Test" << 123));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogEveryN, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY(W, 3, "Test" << 123));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogEveryN, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY(E, 3, "Test" << 123));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogEveryN, lpp_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_EVERY(F, 3, "Test" << 123));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogEveryN, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output =
        LPP_CAPTURE_STDOUT(DLOG_EVERY_N(INFO, 3) << "Test" << 123);
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogEveryN, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output =
        LPP_CAPTURE_STDOUT(LOG_EVERY_N(INFO, 3) << "Test" << 123);
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogEveryN, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output =
        LPP_CAPTURE_STDERR(LOG_EVERY_N(WARNING, 3) << "Test" << 123);
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogEveryN, glog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output =
        LPP_CAPTURE_STDERR(LOG_EVERY_N(ERROR, 3) << "Test" << 123);
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogEveryN, glog_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output =
        LPP_CAPTURE_STDERR(LOG_EVERY_N(FATAL, 3) << "Test" << 123);
    ASSERT_EQ(output, "");
  }
}
