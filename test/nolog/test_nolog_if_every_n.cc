//
// Created by acey on 07.09.23.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

TEST(nolog_if_every_n, glog_syntax_if_every_n_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(DLOG_IF_EVERY_N(INFO, i <= 3, 3) << "Test" << 12);
    ASSERT_EQ("", output);
  }
}

TEST(nolog_if_every_n, glog_syntax_if_every_n_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_IF_EVERY_N(INFO, i <= 3, 3) << "Test" << 12);
    ASSERT_EQ("", output);
  }
}

TEST(nolog_if_every_n, glog_syntax_if_every_n_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_IF_EVERY_N(WARNING, i <= 3, 3) << "Test" << 12);
    ASSERT_EQ("", output);
  }
}

TEST(nolog_if_every_n, glog_syntax_if_every_n_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_IF_EVERY_N(ERROR, i <= 3, 3) << "Test" << 12);
    ASSERT_EQ("", output);
  }
}

TEST(nolog_if_every_n, glog_syntax_if_every_n_severity_fatal) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_IF_EVERY_N(FATAL, i <= 3, 3) << "Test" << 12);
    ASSERT_EQ("", output);
  }
}

