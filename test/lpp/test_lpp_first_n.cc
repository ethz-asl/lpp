//
// Created by 4c3y (acey) on 06.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

TEST(lpp_LogFirstN, lpp_syntax_severity_debug) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(D, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, "DEBUG Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogFirstN, lpp_syntax_severity_info) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(I, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, "INFO  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogFirstN, lpp_syntax_severity_warning) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(W, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, "WARN  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogFirstN, lpp_syntax_severity_error) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(E, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, "ERROR Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogFirstN, lpp_syntax_severity_fatal) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(F, 3, "Test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, "FATAL Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogFirstN, glog_syntax_severity_debug) {
  //TODO DLOG_FIRST_N does not exist in glog
  /*
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(DLOG_FIRST_N(INFO, 3) << "Test" << 123);

    if (i < 3) {
      ASSERT_EQ(output, "INFO  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }*/
}

TEST(lpp_LogFirstN, glog_syntax_severity_info) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST_N(INFO, 3) << "Test" << 123);

    if (i < 3) {
      ASSERT_EQ(output, "INFO  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogFirstN, glog_syntax_severity_warning) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST_N(WARNING, 3) << "Test" << 123);

    if (i < 3) {
      ASSERT_EQ(output, "WARN  Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogFirstN, glog_syntax_severity_error) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST_N(ERROR, 3) << "Test" << 123);

    if (i < 3) {
      ASSERT_EQ(output, "ERROR Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogFirstN, glog_syntax_severity_fatal) {
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST_N(FATAL, 3) << "Test" << 123);

    if (i < 3) {
      ASSERT_EQ(output, "FATAL Test123\n");
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_LogFirstN, ros_info_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_ONCE("Test123"));

    if (i == 0) {
      ASSERT_EQ(output, "INFO  Test123\n");
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(lpp_LogFirstN, ros_warn_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(ROS_WARN_ONCE("Test123"));

    if (i == 0) {
      ASSERT_EQ(output, "WARN  Test123\n");
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(lpp_LogFirstN, ros_error_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(ROS_ERROR_ONCE("Test123"));

    if (i == 0) {
      ASSERT_EQ(output, "ERROR Test123\n");
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(lpp_LogFirstN, ros_fatal_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(ROS_FATAL_ONCE("Test123"));

    if (i == 0) {
      ASSERT_EQ(output, "FATAL Test123\n");
    } else {
      ASSERT_EQ("", output);
    }
  }
}

