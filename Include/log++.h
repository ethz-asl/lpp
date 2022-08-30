//
// Created by 4c3y on 18.08.22.
//

#ifndef LOG__LOG_H_
#define LOG__LOG_H_

//! assert if mode is supported by the libraries available
#if defined MODE_GLOG && !defined GLOG_SUPPORTED
#error Logging Mode is set to glog but glog was not found
#endif

#if defined MODE_ROSLOG && !defined ROSLOG_SUPPORTED
#error Logging Mode is set to roslog but roslog was not found
#endif


//! Includes
#ifdef GLOG_SUPPORTED
#include <glog/logging.h>
#endif // GLOG_SUPPORTED

#ifdef ROSLOG_SUPPORTED
#include <ros/console.h>
#endif


//! un-define macros to avoid conflicts
#ifdef GLOG_SUPPORTED
#undef LOG
#endif

#if defined ROSLOG_SUPPORTED && !defined MODE_ROSLOG
#undef ROS_INFO
#undef ROS_INFO_STREAM
#undef ROS_WARN
#undef ROS_WARN_STREAM
#undef ROS_ERROR
#undef ROS_ERROR_STREAM
#undef ROS_FATAL
#undef ROS_FATAL_STREAM
#endif

struct LPPInternal {
  bool is_lpp_initialized = false;
  bool is_glog_initialized = false;
} lppInternal;


//! Log init
#ifdef MODE_GLOG

//! If LOG_INIT is called more than once, do nothing.
#define LOG_INIT(argv0) if (!lppInternal.is_glog_initialized) { \
google::InitGoogleLogging(argv0); lppInternal.is_glog_initialized = true;} \
lppInternal.is_lpp_initialized = true
#else
#define LOG_INIT(argv0) lppInternal.is_lpp_initialized = true
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

//! MODE_GLOG
#if defined MODE_GLOG || defined MODE_DEFAULT
#define LOG_1(severity) COMPACT_GOOGLE_LOG_ ## severity.stream()
#endif


#ifdef MODE_GLOG
#pragma clang diagnostic push
#pragma ide diagnostic ignored "bugprone-macro-parentheses"

#define LOG_2(severity, x)                                        \
switch(severity) {                                                \
     case I: LOG_1(INFO) << x; break;                             \
     case W: LOG_1(WARNING) << x; break;                          \
     case E: LOG_1(ERROR) << x;break;                             \
     case F: LOG_1(FATAL) << x; break;};

#define ROS_INFO(x) LOG(INFO) << x
#define ROS_INFO_STREAM(x) LOG(INFO) << x
#define ROS_WARN(x) LOG(WARNING) << x
#define ROS_WARN_STREAM(x) LOG(WARNING) << x
#define ROS_ERROR(x) LOG(ERROR) << x
#define ROS_ERROR_STREAM(x) LOG(ERROR) << x
#define ROS_FATAL(x) LOG(FATAL) << x
#define ROS_FATAL_STREAM(x) LOG(FATAL) << x;
#pragma clang diagnostic pop
#endif


//! MODE_ROSLOG
#ifdef MODE_ROSLOG

#define LOG_1(severity) InternalRoslog()
#define LOG_2(severity, x) ROS_INFO_STREAM(x) // NOLINT(bugprone-macro-parentheses)


struct InternalRoslog {
  std::stringstream ss;
  ~InternalRoslog() {
    ROS_INFO_STREAM(ss.str());
  }
};

template<typename T>
InternalRoslog &&operator<<(InternalRoslog &&wrap, T const &whatever) {
  wrap.ss << whatever;
  return std::move(wrap);
}

#endif


struct Log {
  ~Log() {
    std::cout << std::endl;
  }
};

template<typename T>
Log &&
operator<<(Log &&wrap, T const &whatever) {
  std::cout << whatever;
  return std::move(wrap);
}


//! MODE_LPP
#ifdef MODE_LPP
#define INTERNAL_LPP_LOG(x) Log() << glogSeverityToLpp(#x) << " "

#define ROS_INFO(x) LOG_2(I, x)
#define ROS_INFO_STREAM(x) LOG_2(I, x)
#define ROS_WARN(x) LOG_2(W, x)
#define ROS_WARN_STREAM(x) LOG_2(W, x)
#define ROS_ERROR(x) LOG_2(E, x)
#define ROS_ERROR_STREAM(x) LOG_2(E, x)
#define ROS_FATAL(x) LOG_2(F, x)
#define ROS_FATAL_STREAM(x) LOG_2(F, x)

#define LOG_1(severity) INTERNAL_LPP_LOG(severity)
#define LOG_2(severity, x) std::cout << severityToString((severity)) << x << std::endl // NOLINT(bugprone-macro-parentheses)
#endif


//! Helper functions
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

std::string glogSeverityToLpp(std::string glog_severity) {
  std::transform(glog_severity.begin() + 1, glog_severity.end(), glog_severity.begin() + 1, ::tolower);
  return glog_severity;
}

#endif //LOG__LOG_H_
