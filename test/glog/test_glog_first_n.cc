//
// Created by 4c3y (acey) on 08.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(glog_LogFirstN, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST(D, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_first_n.cc"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogFirstN, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST(I, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_first_n.cc"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogFirstN, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST(W, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_first_n.cc"));
      ASSERT_EQ(output[0], 'W');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogFirstN, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST(E, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_first_n.cc"));
      ASSERT_EQ(output[0], 'E');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogFirstN, glog_syntax_severity_debug) {
  //TODO DLOG_FIRST_N does not exist in glog
  /*
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(DLOG_FIRST_N(INFO, 3) << "Test" << 123);

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_first_n.cc"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }*/
}

TEST(glog_LogFirstN, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST_N(INFO, 3) << "Test" << 123);

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_first_n.cc"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogFirstN, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST_N(WARNING, 3) << "Test" << 123);

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_first_n.cc"));
      ASSERT_EQ(output[0], 'W');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogFirstN, glog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST_N(ERROR, 3) << "Test" << 123);

    if (i < 3) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_first_n.cc"));
      ASSERT_EQ(output[0], 'E');
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(glog_LogFirstN, ros_debug_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(ROS_DEBUG_ONCE("Test123"));

    if (i == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_first_n.cc"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(glog_LogFirstN, ros_info_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(ROS_INFO_ONCE("Test123"));

    if (i == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_first_n.cc"));
      ASSERT_EQ(output[0], 'I');
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(glog_LogFirstN, ros_warn_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(ROS_WARN_ONCE("Test123"));

    if (i == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_first_n.cc"));
      ASSERT_EQ(output[0], 'W');
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(glog_LogFirstN, ros_error_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_ONCE("Test123"));

    if (i == 0) {
      ASSERT_TRUE(isSubstring(output, "Test123"));
      ASSERT_TRUE(isSubstring(output, "test_glog_first_n.cc"));
      ASSERT_EQ(output[0], 'E');
    } else {
      ASSERT_EQ("", output);
    }
  }
}