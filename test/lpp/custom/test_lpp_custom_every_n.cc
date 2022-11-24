//
// Created by 4c3y (acey) on 24.11.22.
//

#include <gtest/gtest.h>
#include <test_utils.h>
#include <log++.h>
#include "callback.h"

using namespace lpp::custom;

TEST(lpp_custom_LogEveryN, lpp_syntax_severity_debug) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(D, 3, "test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, debug);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogEveryN, lpp_syntax_severity_info) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(I, 3, "test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, info);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogEveryN, lpp_syntax_severity_warning) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(W, 3, "test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, warning);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogEveryN, lpp_syntax_severity_error) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(E, 3, "test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, error);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogEveryN, lpp_syntax_severity_fatal) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY(F, 3, "test" << 123));

    if (i % 3 == 0) {
      ASSERT_EQ(output, fatal);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogEveryN, glog_syntax_severity_debug) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(DLOG_EVERY_N(INFO, 3) << "test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, debug);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogEveryN, glog_syntax_severity_info) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY_N(INFO, 3) << "test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, info);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogEveryN, glog_syntax_severity_warning) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY_N(WARNING, 3) << "test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, warning);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogEveryN, glog_syntax_severity_error) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY_N(ERROR, 3) << "test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, error);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}

TEST(lpp_custom_LogEveryN, glog_syntax_severity_fatal) {
  LOG_INIT(*test_argv, logCallback);
  for (int i = 0; i < 5; i++) {
    std::string output = LPP_CAPTURE_STDOUT(LOG_EVERY_N(FATAL, 3) << "test" << 123);

    if (i % 3 == 0) {
      ASSERT_EQ(output, fatal);
    } else {
      ASSERT_EQ(output, "");
    }
  }
}