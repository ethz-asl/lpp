//
// Created by 4c3y (acey) on 08.09.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>

using namespace lpp;

TEST(roslog_LogFirstN, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(D, 3, "Test"));

    if (i < 3) {
      ASSERT_EQ(rostest::debug, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(I, 3, "Test"));

    if (i < 3) {
      ASSERT_EQ(rostest::info, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST(W, 3, "Test"));

    if (i < 3) {
      ASSERT_EQ(rostest::warning, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST(E, 3, "Test"));

    if (i < 3) {
      ASSERT_EQ(rostest::error, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, lpp_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST(F, 3, "Test"));

    if (i < 3) {
      ASSERT_EQ(rostest::fatal, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(DLOG_FIRST_N(INFO, 3) << "Test");

    if (i < 3) {
      ASSERT_EQ(rostest::debug, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, glog_syntax_severity_info) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST_N(INFO, 3) << "Test");

    if (i < 3) {
      ASSERT_EQ(rostest::info, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST_N(WARNING, 3) << "Test");

    if (i < 3) {
      ASSERT_EQ(rostest::warning, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, glog_syntax_severity_error) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST_N(ERROR, 3) << "Test");

    if (i < 3) {
      ASSERT_EQ(rostest::error, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, glog_syntax_severity_fatal) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(LOG_FIRST_N(FATAL, 3) << "Test");

    if (i < 3) {
      ASSERT_EQ(rostest::fatal, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, ros_debug_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_ONCE("Test123"));

    if (i == 0) {
      ASSERT_EQ(rostest::debug, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, ros_info_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_ONCE("Test123"));

    if (i == 0) {
      ASSERT_EQ(rostest::info, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, ros_warn_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(ROS_WARN_ONCE("Test123"));

    if (i == 0) {
      ASSERT_EQ(rostest::warning, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, ros_error_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(ROS_ERROR_ONCE("Test123"));

    if (i == 0) {
      ASSERT_EQ(rostest::error, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}

TEST(roslog_LogFirstN, ros_fatal_once) {
  LOG_INIT(*test_argv);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDERR(ROS_FATAL_ONCE("Test123"));

    if (i == 0) {
      ASSERT_EQ(rostest::fatal, removeNumbersFromString(output));
    } else {
      ASSERT_EQ("", removeNumbersFromString(output));
    }
  }
}
