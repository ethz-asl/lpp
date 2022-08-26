//
// Created by 4c3y on 18.08.22.
//

#ifndef LOG__LOG_H_
#define LOG__LOG_H_

/**
 * Static assertions to check if selected log mode is supported by the libraries available
 */

#ifdef MODE_GLOG
#ifndef GLOG_SUPPORTED
static_assert(false, "Logging Mode is set to glog but glog is not found");
#endif
#endif

#ifdef MODE_ROSLOG
#ifndef ROSLOG_SUPPORTED
static_assert(false, "Logging Mode is set to roslog but roslog is not found");
#endif
#endif


//! Helper macro

// If a macro is detected, add an arg, so the second one will be 1.
#define DETECT_EXIST_TRUE ~,1

// DETECT_EXIST merely concatenate a converted macro to the end of DETECT_EXIST_TRUE.
// If empty, DETECT_EXIST_TRUE converts fine.  If not 0 remains second argument.
#define DETECT_EXIST(X) DETECT_EXIST_IMPL(CONCAT2(DETECT_EXIST_TRUE,X), 0, ~)
#define DETECT_EXIST_IMPL(...) SECOND_ARG(__VA_ARGS__)

#ifdef GLOG_SUPPORTED

#include <glog/logging.h>
#undef LOG

#define LOG_INIT(argv0) google::InitGoogleLogging(argv0);

#else
#define LOG_INIT(argv0, mode) static_assert((mode <= 3 && mode >= 0), "Invalid mode");
#endif // GLOG_SUPPORTED

#ifdef ROSLOG_SUPPORTED
#include <ros/console.h>
#endif


//! Severity
#define I 1
#define W 2
#define E 3
#define F 4


//! Hack to enable macro overloading. Used to overload glog's LOG() macro.
#define CAT(A, B) A ## B
#define SELECT(NAME, NUM) CAT( NAME ## _, NUM )

#define GET_COUNT(_1, _2, _3, _4, _5, _6, COUNT, ...) COUNT
#define VA_SIZE(...) GET_COUNT( __VA_ARGS__, 6, 5, 4, 3, 2, 1 )
#define VA_SELECT(NAME, ...) SELECT( NAME, VA_SIZE(__VA_ARGS__) )(__VA_ARGS__)

//! Overloads
#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedMacroInspection"
#define LOG(...) VA_SELECT( LOG, __VA_ARGS__ )
#pragma clang diagnostic pop

/**
 * LOG_1 = Google logging syntax
 * LOG_2 = LPP logging syntax
 */
#if defined MODE_GLOG || defined MODE_DEFAULT
#define LOG_1(severity) COMPACT_GOOGLE_LOG_ ## severity.stream()
#endif

//Map LOG_2 to google logging syntax
#ifdef MODE_GLOG
#pragma clang diagnostic push
#pragma ide diagnostic ignored "bugprone-macro-parentheses"

#define LOG_2(severity, x)                                      \
switch(severity) {                                              \
     case I: LOG_1(INFO) << x; break;                             \
     case W: LOG_1(WARNING) << x; break;                           \
     case E: LOG_1(ERROR) << x;break;                             \
     case F: LOG_1(FATAL) << x; break;};

#pragma clang diagnostic pop
#endif


#ifdef MODE_ROSLOG
#define LOG_1(severity) ROS_ ## severity ## _ STREAM ## severity.stream()
#endif

#ifdef MODE_LPP
#define LPP_INFO  I
#define LPP_WARN  W
#define LPP_ERROR E
#define LPP_FATAL F
#define LOG_1(severity) LOG_2(LPP_ ## severity, severity.stream())
#endif

std::string severityToString(int severity) {
  switch (severity) {
    case 1:return "Info ";
    case 2:return "Warning ";
    case 3:return "Error ";
    case 4:return "Fatal ";
    default:std::cerr << "Warning: skipped invalid severity level" << std::endl;
      return "";
  }
}

//#define LOG_2(severity, x) std::cout << severityToString((severity)) << x << std::endl; // NOLINT(bugprone-macro-parentheses)

#endif //LOG__LOG_H_
