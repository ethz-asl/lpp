//
// Created by 4c3y (acey) on 08.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

//Should print 2 times [Severity] [Rostime]: Test

TEST(roslog_LogEveryN, lpp_syntax_severity_debug) {
  for (int i = 0; i < 5; i++) {
    LOG_EVERY(D, 3, "Test");
  }
}

TEST(roslog_LogEveryN, lpp_syntax_severity_info) {
  for (int i = 0; i < 5; i++) {
    LOG_EVERY(I, 3, "Test");
  }
}

TEST(roslog_LogEveryN, lpp_syntax_severity_warning) {
  for (int i = 0; i < 5; i++) {
    LOG_EVERY(W, 3, "Test");
  }
}

TEST(roslog_LogEveryN, lpp_syntax_severity_error) {
  for (int i = 0; i < 5; i++) {
    LOG_EVERY(E, 3, "Test");
  }
}

TEST(roslog_LogEveryN, lpp_syntax_severity_fatal) {
  for (int i = 0; i < 5; i++) {
    LOG_EVERY(F, 3, "Test");
  }
}

TEST(roslog_LogEveryN, glog_syntax_severity_debug) {
  //TODO implement
  /*
  for (int i = 0; i < 5; i++) {
    DLOG_EVERY_N(INFO, 3) << "Test";
  }*/
}

TEST(roslog_LogEveryN, glog_syntax_severity_info) {

  for (int i = 0; i < 5; i++) {
    LOG_EVERY_N(INFO, 3) << "Test";
  }
}

TEST(roslog_LogEveryN, glog_syntax_severity_warning) {

  for (int i = 0; i < 5; i++) {
    LOG_EVERY_N(WARNING, 3) << "Test";
  }
}

TEST(roslog_LogEveryN, glog_syntax_severity_error) {

  for (int i = 0; i < 5; i++) {
    LOG_EVERY_N(ERROR, 3) << "Test";
  }
}

TEST(roslog_LogEveryN, glog_syntax_severity_fatal) {

  for (int i = 0; i < 5; i++) {
    LOG_EVERY_N(FATAL, 3) << "Test";
  }
}