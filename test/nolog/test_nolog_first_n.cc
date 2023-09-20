//
// Created by acey on 07.09.23.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

TEST(nolog_LogFirstN, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(D, 3, "Test" << 123));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(I, 3, "Test" << 123));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST(W, 3, "Test" << 123));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST(E, 3, "Test" << 123));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, lpp_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST(F, 3, "Test" << 123));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output =
        LPP_CAPTURE_STDOUT(DLOG_FIRST_N(INFO, 3) << "Test" << 123);
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output =
        LPP_CAPTURE_STDOUT(LOG_FIRST_N(INFO, 3) << "Test" << 123);
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output =
        LPP_CAPTURE_STDERR(LOG_FIRST_N(WARNING, 3) << "Test" << 123);
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, glog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output =
        LPP_CAPTURE_STDERR(LOG_FIRST_N(ERROR, 3) << "Test" << 123);
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, glog_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output =
        LPP_CAPTURE_STDERR(LOG_FIRST_N(FATAL, 3) << "Test" << 123);
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, ros_debug_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(ROS_DEBUG_ONCE("Test123"));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, ros_info_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_ONCE("Test123"));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, ros_warn_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(ROS_WARN_ONCE("Test123"));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, ros_error_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_ONCE("Test123"));
    ASSERT_EQ(output, "");
  }
}

TEST(nolog_LogFirstN, ros_fatal_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_ONCE("Test123"));
    ASSERT_EQ(output, "");
  }
}