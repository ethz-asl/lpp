//
// Created by acey on 30.08.22.
//

#include <gtest/gtest.h>
#include "test_utils.h"

#undef MODE_GLOG
#define MODE_ROSLOG
#include "log++.h"

//TODO
TEST(roslog_glog_syntax, severity_info) {
  LOG_INIT(*test_argv);

  testing::internal::CaptureStderr();
  LOG(INFO) << "xyz";
  std::string output = testing::internal::GetCapturedStderr();
  LOG(INFO) << output;
}
#undef MODE_ROSLOG