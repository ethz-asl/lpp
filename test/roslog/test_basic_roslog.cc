//
// Created by acey on 30.08.22.
//

#include <gtest/gtest.h>
#include "test_utils.h"

#include "log++.h"

/**
 * Ros logging stdout/stderr capture doesn't work. Probably because of threading things.
 * Unittests only cover ros function calls through Log++ at the moment.
 */

//! ################ GLOG ################
TEST(roslog_glog_syntax, severity_info) {
  LOG(INFO) << "Test";
}

TEST(roslog_glog_syntax, severity_warning) {
  LOG(WARNING) << "Test";
}

TEST(roslog_glog_syntax, severity_error) {
  LOG(ERROR) << "Test";
}

TEST(roslog_glog_syntax, severity_fatal) {
  LOG(FATAL) << "Test";
}

//! ################ lpp ################
TEST(roslog_lpp_syntax, severity_info) {
  LOG(I, "" << "Test");
}

TEST(roslog_lpp_syntax, severity_warning) {
  LOG(W, "" << "Test");
}

TEST(roslog_lpp_syntax, severity_error) {
  LOG(E, "" << "Test");
}

TEST(roslog_lpp_syntax, severity_fatal) {
  LOG(F, "" << "Test");
}

//! ################ Roslog ################
TEST(roslog_roslog_syntax, severity_info) {
  ROS_INFO("Test");
  ROS_INFO_STREAM("" << "Test");
}

TEST(roslog_roslog_syntax, severity_warn) {
  ROS_WARN("Test");
  ROS_WARN_STREAM("" << "Test");
}

TEST(roslog_roslog_syntax, severity_error) {
  ROS_ERROR("Test");
  ROS_ERROR_STREAM("" << "Test");
}

TEST(roslog_roslog_syntax, severity_fatal) {
  ROS_FATAL("Test");
  ROS_FATAL_STREAM("" << "Test");
}
#undef MODE_ROSLOG