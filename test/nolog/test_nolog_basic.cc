//
// Created by acey on 06.09.23.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(nolog_basic, glog_syntax_severity_debug){
  LOG_INIT(*test_argv);
  DLOG(INFO) << "TEst";

  std::string output = LPP_CAPTURE_STDOUT(DLOG(INFO) << "Test");
  ASSERT_EQ("", output);
}

TEST(nolog_basic, glog_syntax_severity_info){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(INFO) << "Test123");
  ASSERT_EQ("", output);
}

TEST(nolog_basic, glog_syntax_severity_warning){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(WARNING) << "Test");
  ASSERT_EQ("", output);
}

TEST(nolog_basic, glog_syntax_severity_error){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(ERROR) << "Test");
  ASSERT_EQ("", output);
}

TEST(nolog_basic, glog_syntax_severity_fatal){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(FATAL) << "Test");
  ASSERT_EQ("", output);
}

//! ################ lpp ################
TEST(nolog_basic, lpp_syntax_severity_debug){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(D, "" << "Test"));
  ASSERT_EQ("", output);
}

TEST(nolog_basic, lpp_syntax_severity_info){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(LOG(I, "" << "Test"));
  ASSERT_EQ("", output);
}

TEST(nolog_basic, lpp_syntax_severity_warning){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(W, "" << "Test"));
  ASSERT_EQ("", output);
}

TEST(nolog_basic, lpp_syntax_severity_error){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(LOG(E, "" << "Test"));
  ASSERT_EQ("", output);
}

//! ################ Roslog ################
TEST(nolog_basic, roslog_syntax_severity_debug){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_STREAM("Test"));
  ASSERT_EQ("", output);
}

TEST(nolog_basic, roslog_syntax_severity_info){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_STREAM("Test"));
  ASSERT_EQ("", output);
}

TEST(nolog_basic, roslog_syntax_severity_warning){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_WARN_STREAM("Test"));
  ASSERT_EQ("", output);
}

TEST(nolog_basic, roslog_syntax_severity_error){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_STREAM("Test"));
  ASSERT_EQ("", output);
}

TEST(nolog_basic, roslog_syntax_severity_fatal){
  LOG_INIT(*test_argv);

  std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_STREAM("Test"));
  ASSERT_EQ("", output);
}