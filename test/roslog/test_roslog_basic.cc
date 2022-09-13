//
// Created by acey on 30.08.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

/**
 * Ros logging stdout/stderr capture doesn't work. Probably because of threading things.
 * At the moment, unittests only test if ros function calls are called  through Log++.
 */

//! ################ GLOG ################
TEST(roslog_basic, glog_syntax_severity_debug) {
  //TODO implement
  //DLOG(INFO) << "Test";
}

TEST(roslog_basic, glog_syntax_severity_info) {
  LOG(INFO) << "Test";
}

TEST(roslog_basic, glog_syntax_severity_warning) {
  LOG(WARNING) << "Test";
}

TEST(roslog_basic, glog_syntax_severity_error) {
  LOG(ERROR) << "Test";
}

TEST(roslog_basic, glog_syntax_severity_fatal) {
  LOG(FATAL) << "Test";
}

//! ################ lpp ################
TEST(roslog_basic, lpp_syntax_severity_debug) {
  LOG(I, "" << "Test");
}

TEST(roslog_basic, lpp_syntax_severity_info) {
  LOG(I, "" << "Test");
}

TEST(roslog_basic, lpp_syntax_severity_warning) {
  LOG(W, "" << "Test");
}

TEST(roslog_basic, lpp_syntax_severity_error) {
  LOG(E, "" << "Test");
}

TEST(roslog_basic, lpp_syntax_severity_fatal) {
  LOG(F, "" << "Test");
}

//! ################ Roslog ################
TEST(roslog_basic, roslog_syntax_severity_debug) {
  ROS_DEBUG("Test");
}

TEST(roslog_basic, roslog_syntax_severity_debug_stream) {
  ROS_DEBUG_STREAM("Test");
}

TEST(roslog_basic, roslog_syntax_severity_info) {
  ROS_INFO("Test");
}

TEST(roslog_basic, roslog_syntax_severity_info_stream) {
  ROS_INFO_STREAM("" << "Test");
}

TEST(roslog_basic, roslog_syntax_severity_warn) {
  ROS_WARN("Test");
}

TEST(roslog_basic, roslog_syntax_severity_warn_stream) {
 ROS_WARN_STREAM("" << "Test");
}

TEST(roslog_basic, roslog_syntax_severity_error) {
  ROS_ERROR("Test");
}

TEST(roslog_basic, roslog_syntax_severity_error_stream) {
  ROS_ERROR_STREAM("" << "Test");
}

TEST(roslog_basic, roslog_syntax_severity_fatal) {
  ROS_FATAL("Test");
}

TEST(roslog_basic, roslog_syntax_severity_fatal_stream) {
  ROS_FATAL_STREAM("" << "Test");
}
