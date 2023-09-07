//
// Created by 4c3y (acey) on 27.09.22.
//

#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

TEST(lpp_vlog, glog_syntax_severity_v1) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG(1) << "Test" << 123);
  ASSERT_EQ(output, "DEBUG Test123\n");
}

TEST(lpp_vlog, glog_syntax_severity_v3) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG(3) << "Test123");
  ASSERT_EQ(output, "DEBUG Test123\n");
}

TEST(lpp_vlog, glog_syntax_severity_v5) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG(5) << "Test123");
  ASSERT_EQ(output, "");
}

TEST(lpp_vlog, glog_syntax_severity_if_v1) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(1, true) << "Test123");
  ASSERT_EQ(output, "DEBUG Test123\n");
}

TEST(lpp_vlog, glog_syntax_severity_if_v3) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(1, true) << "Test123");
  ASSERT_EQ(output, "DEBUG Test123\n");
}

TEST(lpp_vlog, glog_syntax_severity_if_v5) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(5, true) << "Test123");
  ASSERT_EQ(output, "");
}

TEST(lpp_vlog, glog_syntax_severity_ifnot_v1) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(1, false) << "Test123");
  ASSERT_EQ(output, "");
}

TEST(lpp_vlog, glog_syntax_severity_ifnot_v3) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(3, false) << "Test123");
  ASSERT_EQ(output, "");
}

TEST(lpp_vlog, glog_syntax_severity_ifnot_v5) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  std::string output = LPP_CAPTURE_STDOUT(VLOG_IF(5, false) << "Test123");
  ASSERT_EQ(output, "");
}


TEST(lpp_vlog, glog_syntax_every_n_severity_v1) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(VLOG_EVERY_N(1, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, "DEBUG Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_vlog, glog_syntax_every_n_severity_v3) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(VLOG_EVERY_N(3, 3) << "Test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, "DEBUG Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_vlog, glog_syntax_every_n_severity_v5) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(VLOG_EVERY_N(5, 3) << "Test" << 123);
    ASSERT_EQ(output, "");
  }
}

TEST(lpp_vlog, glog_syntax_if_every_n_severity_v1) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    VLOG_IF_EVERY_N(1, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, "DEBUG Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_vlog, glog_syntax_if_every_n_severity_v3) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    VLOG_IF_EVERY_N(3, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    if (i <= 3 && i % 3 == 0) {
      ASSERT_EQ(output, "DEBUG Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_vlog, glog_syntax_if_every_n_severity_v5) {
  LOG_INIT(*test_argv);
  FLAGS_v = 3;

  for (int i = 0; i < 5; i++) {
    testing::internal::CaptureStdout();
    VLOG_IF_EVERY_N(5, i <= 3, 3) << "Test" << 123;
    std::string output = testing::internal::GetCapturedStdout();

    ASSERT_EQ(output, "");
  }
}
