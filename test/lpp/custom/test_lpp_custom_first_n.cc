//
// Created by 4c3y (acey) on 24.11.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>
#include "callback.h"

using namespace lpp::custom;

TEST(lpp_custom_LogFirstN, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(D, 3, "test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, debug);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogFirstN, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(I, 3, "test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, info);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogFirstN, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(W, 3, "test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, warning);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogFirstN, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(E, 3, "test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, error);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogFirstN, lpp_syntax_severity_fatal) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST(F, 3, "test" << 123));

    if (i < 3) {
      ASSERT_EQ(output, fatal);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogFirstN, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(DLOG_FIRST_N(INFO, 3) << "test" << 123);

    if (i < 3) {
      ASSERT_EQ(output, debug);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogFirstN, glog_syntax_severity_info) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST_N(INFO, 3) << "test" << 123);

    if (i < 3) {
      ASSERT_EQ(output, info);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogFirstN, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST_N(WARNING, 3) << "test" << 123);

    if (i < 3) {
      ASSERT_EQ(output, warning);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogFirstN, glog_syntax_severity_error) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST_N(ERROR, 3) << "test" << 123);

    if (i < 3) {
      ASSERT_EQ(output, error);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogFirstN, glog_syntax_severity_fatal) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_FIRST_N(FATAL, 3) << "test" << 123);

    if (i < 3) {
      ASSERT_EQ(output, fatal);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogFirstN, ros_debug_once) {
  LOG_INIT(*test_argv, logCallback);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(ROS_DEBUG_ONCE("test123"));

    if (i == 0) {
      ASSERT_EQ(output, debug);
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(lpp_custom_LogFirstN, ros_info_once) {
  LOG_INIT(*test_argv, logCallback);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(ROS_INFO_ONCE("test123"));

    if (i == 0) {
      ASSERT_EQ(output, info);
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(lpp_custom_LogFirstN, ros_warn_once) {
  LOG_INIT(*test_argv, logCallback);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(ROS_WARN_ONCE("test123"));

    if (i == 0) {
      ASSERT_EQ(output, warning);
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(lpp_custom_LogFirstN, ros_error_once) {
  LOG_INIT(*test_argv, logCallback);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(ROS_ERROR_ONCE("test123"));

    if (i == 0) {
      ASSERT_EQ(output, error);
    } else {
      ASSERT_EQ("", output);
    }
  }
}

TEST(lpp_custom_LogFirstN, ros_fatal_once) {
  LOG_INIT(*test_argv, logCallback);

  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(ROS_FATAL_ONCE("test123"));

    if (i == 0) {
      ASSERT_EQ(output, fatal);
    } else {
      ASSERT_EQ("", output);
    }
  }
}