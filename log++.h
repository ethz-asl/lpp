//
// Created by 4c3y on 18.08.22.
//

#ifndef LOG__LOG_H_
#define LOG__LOG_H_

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

#define LOG_INIT(argv0, mode) google::InitGoogleLogging(argv0); static_assert((mode <= 3 && mode >= 0), "Invalid mode");

#else
#define LOG_INIT(argv0, mode) static_assert((mode <= 3 && mode >= 0), "Invalid mode");
#endif // GLOG_SUPPORTED

#ifdef ROSLOG_SUPPORTED
#include <ros/console.h>
#endif


//! Severity

#define I 0
#define W 1
#define E 2
#define F 3


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

#define LOG_1(severity) COMPACT_GOOGLE_LOG_ ## severity.stream() //Glog syntax
#define LOG_2(severity, x) std::cout << severityToString((severity)) << x << std::endl; // NOLINT(bugprone-macro-parentheses)
#pragma clang diagnostic pop

std::string severityToString(int severity) {

  switch (severity) {
    case 0:return "Info ";
    case 1:return "Warning ";
    case 2:return "Error ";
    case 3:return "Fatal ";
    default:std::cerr << "Warning: skipped invalid severity level" << std::endl;
      return "";
  }
}

namespace lpp {

/**
 * Logging messages formats
 */

enum Mode {
  GLOG = 0,
  ROSLOG = 1,
  LOGPP = 2,
  DEFAULT = 3 //Keep formats of each logging framework
};

class LogController {
 public:
  static void setLoggingMode(Mode mode) {


    switch (mode) {
      case GLOG:
        static_assert(is_glog_supported, "glog not supported");
        break;

      case ROSLOG:
        static_assert(is_roslog_supported, "roslog not supported");
        break;

      default:break;
    }
    logging_mode_ = mode;

  }
 private:
  static int logging_mode_;




  static constexpr bool is_roslog_supported =
#ifdef ROSLOG_SUPPORTED
      true;
#else
  false;
#endif

  static const bool is_glog_supported =
#ifdef GLOG_SUPPORTED
   true;
#else
  false;
#endif


};
} // end namespace lpp

#endif //LOG__LOG_H_