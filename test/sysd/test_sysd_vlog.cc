#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

#include "sysd_test_utils.h"

using lpp::sysdtest::entries;

TEST(sysd_vlog, glog_syntax_severity_v1) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  VLOG(1) << "Test" << 123;
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_vlog, glog_syntax_severity_v3) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  VLOG(3) << "Test123";
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_vlog, glog_syntax_severity_v5) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  VLOG(5) << "Test123";
  ASSERT_TRUE(entries().empty());
}

TEST(sysd_vlog, glog_syntax_severity_if_v1) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  VLOG_IF(1, true) << "Test123";
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::DEBUG);
}

TEST(sysd_vlog, glog_syntax_severity_if_v3) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  VLOG_IF(3, true) << "Test123";
  ASSERT_EQ(entries().size(), 1);
}

TEST(sysd_vlog, glog_syntax_severity_if_v5) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  VLOG_IF(5, true) << "Test123";
  ASSERT_TRUE(entries().empty());
}

TEST(sysd_vlog, glog_syntax_severity_ifnot_v1) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  VLOG_IF(1, false) << "Test123";
  ASSERT_TRUE(entries().empty());
}

TEST(sysd_vlog, glog_syntax_severity_ifnot_v3) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  VLOG_IF(3, false) << "Test123";
  ASSERT_TRUE(entries().empty());
}

TEST(sysd_vlog, glog_syntax_severity_ifnot_v5) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  VLOG_IF(5, false) << "Test123";
  ASSERT_TRUE(entries().empty());
}

TEST(sysd_vlog, glog_syntax_every_n_severity_v1) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  for (int i = 0; i < 5; i++) {
    VLOG_EVERY_N(1, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
}

TEST(sysd_vlog, glog_syntax_every_n_severity_v3) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  for (int i = 0; i < 5; i++) {
    VLOG_EVERY_N(3, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
}

TEST(sysd_vlog, glog_syntax_every_n_severity_v5) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  for (int i = 0; i < 5; i++) {
    VLOG_EVERY_N(5, 3) << "Test" << 123;
  }
  ASSERT_TRUE(entries().empty());
}

TEST(sysd_vlog, glog_syntax_if_every_n_severity_v1) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  for (int i = 0; i < 5; i++) {
    VLOG_IF_EVERY_N(1, i <= 3, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
}

TEST(sysd_vlog, glog_syntax_if_every_n_severity_v3) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  for (int i = 0; i < 5; i++) {
    VLOG_IF_EVERY_N(3, i <= 3, 3) << "Test" << 123;
  }
  ASSERT_EQ(entries().size(), 2);
}

TEST(sysd_vlog, glog_syntax_if_every_n_severity_v5) {
  lpp::sysdtest::init(*test_argv);
  FLAGS_v = 3;
  for (int i = 0; i < 5; i++) {
    VLOG_IF_EVERY_N(5, i <= 3, 3) << "Test" << 123;
  }
  ASSERT_TRUE(entries().empty());
}
