//
// Created by 4c3y (acey) on 27.09.22.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

TEST(default_vlog, glog_syntax_severity_v1) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDERR(VLOG(1) << "Test" << 123);
  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_default_vlog.cc"));
  ASSERT_EQ(output[0], 'I');
}

TEST(default_vlog, glog_syntax_severity_v3) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDERR(VLOG(3) << "Test123");
  ASSERT_TRUE(isSubstring(output, "Test123"));
  ASSERT_TRUE(isSubstring(output, "test_default_vlog.cc"));
  ASSERT_EQ(output[0], 'I');
}

TEST(default_vlog, glog_syntax_severity_v5) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDERR(VLOG(5) << "Test123");
  ASSERT_EQ(output, "");
}