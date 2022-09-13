//
// Created by 4c3y (acey) on 08.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(roslog_LogFirstN, lpp_syntax_severity_debug) {
  for (int i = 0; i < 5; i++) {
    LOG_FIRST(D, 3, "Test");
  }
}

TEST(roslog_LogFirstN, lpp_syntax_severity_info) {
  for (int i = 0; i < 5; i++) {
    LOG_FIRST(I, 3, "Test");
  }
}

TEST(roslog_LogFirstN, lpp_syntax_severity_warning) {
  for (int i = 0; i < 5; i++) {
    LOG_FIRST(W, 3, "Test");
  }
}

TEST(roslog_LogFirstN, lpp_syntax_severity_error) {
  for (int i = 0; i < 5; i++) {
    LOG_FIRST(E, 3, "Test");
  }
}

TEST(roslog_LogFirstN, lpp_syntax_severity_fatal) {
  for (int i = 0; i < 5; i++) {
    LOG_FIRST(F, 3, "Test");
  }
}

TEST(roslog_LogFirstN, glog_syntax_severity_debug) {
  //TODO
  /*
  for (int i = 0; i < 5; i++) {
    DLOG_FIRST_N(INFO, 3) << "Test";
  }*/
}

TEST(roslog_LogFirstN, glog_syntax_severity_info) {
  for (int i = 0; i < 5; i++) {
    LOG_FIRST_N(INFO, 3) << "Test";
  }
}

TEST(roslog_LogFirstN, glog_syntax_severity_warning) {
  for (int i = 0; i < 5; i++) {
    LOG_FIRST_N(WARNING, 3) << "Test";
  }
}

TEST(roslog_LogFirstN, glog_syntax_severity_error) {
  for (int i = 0; i < 5; i++) {
    LOG_FIRST_N(ERROR, 3) << "Test";
  }
}

TEST(roslog_LogFirstN, glog_syntax_severity_fatal) {
  for (int i = 0; i < 5; i++) {
    LOG_FIRST_N(FATAL, 3) << "Test";
  }
}

TEST(roslog_LogFirstN, ros_debug_once) {
  for (int i = 0; i < 5; i++) {
    ROS_DEBUG_ONCE("Test123");
  }
}

TEST(roslog_LogFirstN, ros_info_once) {
  for (int i = 0; i < 5; i++) {
    ROS_INFO_ONCE("Test123");
  }
}

TEST(roslog_LogFirstN, ros_warn_once) {
  for (int i = 0; i < 5; i++) {
    ROS_WARN_ONCE("Test123");
  }
}

TEST(roslog_LogFirstN, ros_error_once) {
  for (int i = 0; i < 5; i++) {
    ROS_ERROR_ONCE("Test123");
  }
}

TEST(roslog_LogFirstN, ros_fatal_once) {
  for (int i = 0; i < 5; i++) {
    ROS_FATAL_ONCE("Test123");
  }
}
