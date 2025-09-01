/* BSD 3-Clause License
 *
 * Copyright ©2022, Autonomous Systems Lab, ETH Zurich, 4c3y
 * (https://github.com/4c3y) All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LOG__LOG_H_
#define LOG__LOG_H_

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <unordered_map>
#include <utility>

#if !defined MODE_LPP && !defined MODE_GLOG && !defined MODE_ROSLOG && \
    !defined MODE_DEFAULT && !defined MODE_NOLOG
#define MODE_DEFAULT
#warning "No mode defined. Selected MODE_DEFAULT";
#endif

/**
 * @brief Terminates compilation if more than one mode is defined
 *
 * To suppress potentially multiple thousands lines of errors,
 * all modes except MODE_DEFAULT gets undefined in this case.
 *
 * Defining MODE_DEFAULT will prevent errors from being generated for each
 * logging function that is called.
 */
#if defined(MODE_LPP) + defined(MODE_GLOG) + defined(MODE_ROSLOG) + \
        defined(MODE_DEFAULT) + defined(MODE_NOLOG) >               \
    1
#undef MODE_LPP
#undef MODE_GLOG
#undef MODE_ROSLOG
#undef MODE_NOLOG
#define MODE_DEFAULT
#error "More than one mode is defined"
#endif

//! Check if libraries are available at compile time and include required
//! headers
#if __has_include(<glog/logging.h>)
#include <glog/logging.h>
#define LPP_GLOG_SUPPORTED
#endif

#if __has_include(<ros/console.h>)
#include <ros/console.h>
#define LPP_ROSLOG_SUPPORTED
#endif

//! Debug flag, If LPP_DEBUG is enabled, debug output should be printed.
#ifndef NDEBUG
#define LPP_DEBUG
#endif

//! Can't use namespaces in macros, used to increase readability
#define LPP_INTL lpp::internal

// Diagnostic push/pop
#if __clang__
#define DIAG_PUSH _Pragma("clang diagnostic push")
#define DIAG_POP _Pragma("clang diagnostic pop")
#elif __GNUC__
#define DIAG_PUSH _Pragma("GCC diagnostic push")
#define DIAG_POP _Pragma("GCC diagnostic pop")
#else
#define DIAG_PUSH
#define DIAG_POP
#endif
DIAG_PUSH

// jetbrains ide hints
#ifdef __JETBRAINS_IDE__
#define IDE_IGNORE(inspect) _Pragma("ide diagnostic ignored \"" inspect "\"")
#else
#define IDE_IGNORE(inspect)
#endif
IDE_IGNORE(
    "modernize-concat-nested-namespaces")  // Keep compatibility with C++11

namespace lpp {
namespace internal {
enum class BaseSeverity { DEBUG, INFO, WARN, ERROR, FATAL };

//! Initialization logic
class Init {
 public:
  bool lpp_initialized = false;
  bool glog_initialized = false;
};

class Logging {
 public:
  inline void call(BaseSeverity severity, const std::string &str) {
    fn_(severity, str);
  }

  inline void setLoggingFunction(
      const std::function<void(BaseSeverity, const std::string &)> &fn) {
    fn_ = fn;
  }

 private:
  std::function<void(BaseSeverity, const std::string &)> fn_ =
      [](BaseSeverity severity, const std::string &str) {
        switch (severity) {
          case BaseSeverity::DEBUG:
            std::cout << "DEBUG " << str << std::endl;
            return;
          case BaseSeverity::INFO:
            std::cout << "INFO  " << str << std::endl;
            return;
          case BaseSeverity::WARN:
            std::cout << "WARN  " << str << std::endl;
            return;
          case BaseSeverity::ERROR:
            std::cout << "ERROR " << str << std::endl;
            return;
          case BaseSeverity::FATAL:
            std::cout << "FATAL " << str << std::endl;
            return;
        }
      };
};

inline static Logging logging;
inline static Init lppInit;
}  // namespace internal
}  // namespace lpp

//! assert if mode is supported by the libraries available
#if defined MODE_GLOG && !defined LPP_GLOG_SUPPORTED
#error Logging Mode is set to glog but glog was not found
#endif

#if defined MODE_ROSLOG && !defined LPP_ROSLOG_SUPPORTED
#error Logging Mode is set to roslog but roslog was not found
#endif

//! Un-define glog`s LOG macro to avoid conflicts
#ifdef LPP_GLOG_SUPPORTED
#undef LOG
#endif

//! Un-define log methods for redefinition
#if defined LPP_GLOG_SUPPORTED && !defined MODE_GLOG && !defined MODE_DEFAULT
#undef LOG_IF
#undef LOG_EVERY_N
#undef LOG_FIRST_N
#undef LOG_IF_EVERY_N
#undef VLOG
#undef VLOG_EVERY_N
#undef VLOG_IF
#undef VLOG_IF_EVERY_N
#undef DLOG
#undef DLOG_EVERY_N
#undef DLOG_IF_EVERY_N
#undef LOG_STRING
#if defined(LOG_EVERY_T)
#undef LOG_EVERY_T
#undef DLOG_EVERY_T
#define LPP_IS_GLOG_V0_6
#define LPP_GLOG_V0_6_WARNING(str)
#endif
#endif

#ifndef LPP_GLOG_V0_6_WARNING
#define LPP_GLOG_V0_6_WARNING(str) LPP_WARN(str)
#endif

#if defined LPP_ROSLOG_SUPPORTED && !defined MODE_ROSLOG && \
    !defined MODE_DEFAULT
#undef ROS_DEBUG
#undef ROS_DEBUG_STREAM
#undef ROS_INFO
#undef ROS_INFO_STREAM
#undef ROS_WARN
#undef ROS_WARN_STREAM
#undef ROS_ERROR
#undef ROS_ERROR_STREAM
#undef ROS_FATAL
#undef ROS_FATAL_STREAM

#undef ROS_DEBUG_COND
#undef ROS_DEBUG_STREAM_COND
#undef ROS_INFO_COND
#undef ROS_INFO_STREAM_COND
#undef ROS_WARN_COND
#undef ROS_WARN_STREAM_COND
#undef ROS_ERROR_COND
#undef ROS_ERROR_STREAM_COND
#undef ROS_FATAL_COND
#undef ROS_FATAL_STREAM_COND

#undef ROS_DEBUG_ONCE
#undef ROS_INFO_ONCE
#undef ROS_WARN_ONCE
#undef ROS_ERROR_ONCE
#undef ROS_FATAL_ONCE

#undef ROS_DEBUG_THROTTLE
#undef ROS_DEBUG_STREAM_THROTTLE
#undef ROS_INFO_THROTTLE
#undef ROS_INFO_STREAM_THROTTLE
#undef ROS_WARN_THROTTLE
#undef ROS_WARN_STREAM_THROTTLE
#undef ROS_ERROR_THROTTLE
#undef ROS_ERROR_STREAM_THROTTLE
#undef ROS_FATAL_THROTTLE
#undef ROS_FATAL_STREAM_THROTTLE
#endif

using namespace lpp::internal;

/**
 * Used to initialize Log++
 *
 * If called more than once, all further calls will be ignored.
 * @param argv is used if MODE_GLOG is defined, otherwise unused.
 * @param callback is used if MODE_LPP is defined, otherwise unused.
 */
inline void LOG_INIT(
    [[maybe_unused]] char *argv,
    [[maybe_unused]] const std::function<
        void(BaseSeverity, const std::string &)> &callback = nullptr) {
  // If LOG_INIT is called more than once, do nothing.
  if (!lppInit.glog_initialized || !lppInit.lpp_initialized) {
#if defined MODE_LPP
    if (callback != nullptr) {
      logging.setLoggingFunction(callback);
    }
#endif

#if defined LPP_DEBUG && (defined MODE_ROSLOG || defined MODE_DEFAULT)
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                       ros::console::levels::Debug)) {
      ros::console::notifyLoggerLevelsChanged();
    }
#endif

#if defined MODE_GLOG || defined MODE_DEFAULT
#ifdef LPP_GLOG_SUPPORTED
    google::InitGoogleLogging(argv);
    FLAGS_logtostderr = true;
    lppInit.glog_initialized = true;
#endif
#endif
    lppInit.lpp_initialized = true;
  }
}

#define LPP_ASSERT_LPP(x)                                                     \
  static_assert(                                                              \
      (x) == LPP_INTL::LppSeverity::D || (x) == LPP_INTL::LppSeverity::I ||   \
          (x) == LPP_INTL::LppSeverity::W ||                                  \
          (x) == LPP_INTL::LppSeverity::E || (x) == LPP_INTL::LppSeverity::F, \
      "Unknown severity level")
#define LPP_ASSERT_GLOG(x)                                    \
  static_assert((x) == LPP_INTL::GlogSeverity::INFO ||        \
                    (x) == LPP_INTL::GlogSeverity::WARNING || \
                    (x) == LPP_INTL::GlogSeverity::ERROR ||   \
                    (x) == LPP_INTL::GlogSeverity::FATAL,     \
                "Unknown severity level")

//! Hack to enable macro overloading. Used to overload glog's LOG() macro.
#define LPP_CAT(A, B) A##B
#define LPP_SELECT(NAME, NUM) LPP_CAT(NAME##_, NUM)

#define LPP_GET_COUNT(_1, _2, _3, _4, _5, _6, COUNT, ...) COUNT
#define LPP_VA_SIZE(...) LPP_GET_COUNT(__VA_ARGS__, 6, 5, 4, 3, 2, 1)
#define LPP_VA_SELECT(NAME, ...) \
  LPP_SELECT(NAME, LPP_VA_SIZE(__VA_ARGS__))(__VA_ARGS__)

//! Helper macros to generate warnings
#define LPP_PRAGMA(x) _Pragma(#x)
#define LPP_WARN(x) LPP_PRAGMA(message(#x))

//! Overloads
DIAG_PUSH
IDE_IGNORE("OCUnusedMacroInspection")
#define LOG(...) LPP_VA_SELECT(LOG, __VA_ARGS__)
DIAG_POP

/**
 * LOG_1 = Google logging syntax
 * LOG_2 = LPP logging syntax
 * LOG_3 = LPP conditional syntax
 */

//! MODE_GLOG
#if defined MODE_GLOG || defined MODE_DEFAULT
#define LOG_1(severity) COMPACT_GOOGLE_LOG_##severity.stream()

#define LPP_INTL_LOG_EVERY_T(severity, t)                                      \
  LPP_GLOG_V0_6_WARNING(                                                       \
      "LOG_EVERY_T is only defined in GLOG v0.6 or newer. File name and line " \
      "numbers will be invalid in the log output.")                            \
  LPP_INTL::InternalPolicyLog<float>(LPP_GET_KEY(), t, toBase(severity),       \
                                     LPP_INTL::PolicyType::TIMED)
#ifndef LOG_EVERY_T
#define LOG_EVERY_T(severity, t)                                               \
  LPP_GLOG_V0_6_WARNING("LOG_EVERY_T is only defined in GLOG v0.6 or newer.")  \
  LPP_INTL::InternalPolicyLog<float>(LPP_GET_KEY(), t,                         \
                                     toBase(LPP_INTL::GlogSeverity::severity), \
                                     LPP_INTL::PolicyType::TIMED)
#endif

#define LPP_INTL_DLOG_EVERY_T(severity, t)                                  \
  LPP_WARN("File name and line numbers will be invalid in the log output.") \
  LOG_EVERY_T(severity, t)

#ifndef DLOG_EVERY_T
#define DLOG_EVERY_T(severity, t)                         \
  LPP_WARN("DLOG_EVERY_T is a Log++ extension")           \
  LPP_INTL::LppGlogExtensionLog(                          \
      LPP_GET_KEY(), t, LPP_INTL::GlogSeverity::severity, \
      LPP_INTL::PolicyType::TIMED,                        \
      [](const std::string &str) { LOG_1(severity) << str; })
#endif

#ifndef DLOG_FIRST_N
#define DLOG_FIRST_N(severity, n)                         \
  LPP_WARN("DLOG_FIRST_N is a Log++ extension")           \
  LPP_INTL::LppGlogExtensionLog(                          \
      LPP_GET_KEY(), n, LPP_INTL::GlogSeverity::severity, \
      LPP_INTL::PolicyType::FIRST_N,                      \
      [](const std::string &str) { LOG_1(severity) << str; })
#endif
#endif

#ifdef MODE_GLOG
DIAG_PUSH
IDE_IGNORE("bugprone-macro-parentheses")

#define LOG_2(severity, x)                                  \
  LPP_ASSERT_LPP(LPP_INTL::LppSeverity::severity);          \
  do {                                                      \
    if constexpr (LPP_INTL::LppSeverity::severity ==        \
                      LPP_INTL::LppSeverity::I ||           \
                  LPP_INTL::LppSeverity::severity ==        \
                      LPP_INTL::LppSeverity::D) {           \
      LOG_1(INFO) << x;                                     \
    } else if constexpr (LPP_INTL::LppSeverity::severity == \
                         LPP_INTL::LppSeverity::W) {        \
      LOG_1(WARNING) << x;                                  \
    } else if constexpr (LPP_INTL::LppSeverity::severity == \
                         LPP_INTL::LppSeverity::E) {        \
      LOG_1(ERROR) << x;                                    \
    } else if constexpr (LPP_INTL::LppSeverity::severity == \
                         LPP_INTL::LppSeverity::F) {        \
      LOG_1(FATAL) << x;                                    \
    }                                                       \
  } while (0)

// Add true at the end to make semicolons mandatory. Compiles to nothing.
#define LOG_3(severity, cond, x) \
  if (cond) {                    \
    LOG_2(severity, x);          \
  }                              \
  true
#define LOG_EVERY(severity, n, x)                           \
  LPP_ASSERT_LPP(LPP_INTL::LppSeverity::severity);          \
  do {                                                      \
    if constexpr (LPP_INTL::LppSeverity::severity ==        \
                  LPP_INTL::LppSeverity::I) {               \
      LOG_EVERY_N(INFO, n) << x;                            \
    } else if constexpr (LPP_INTL::LppSeverity::severity == \
                         LPP_INTL::LppSeverity::W) {        \
      LOG_EVERY_N(WARNING, n) << x;                         \
    } else if constexpr (LPP_INTL::LppSeverity::severity == \
                         LPP_INTL::LppSeverity::E) {        \
      LOG_EVERY_N(ERROR, n) << x;                           \
    } else if constexpr (LPP_INTL::LppSeverity::severity == \
                         LPP_INTL::LppSeverity::F) {        \
      LOG_EVERY_N(FATAL, n) << x;                           \
    } else if constexpr (LPP_INTL::LppSeverity::severity == \
                         LPP_INTL::LppSeverity::D) {        \
      DLOG_EVERY_N(INFO, n) << x;                           \
    }                                                       \
  } while (0)

#define LOG_FIRST(severity, n, x)                           \
  LPP_ASSERT_LPP(LPP_INTL::LppSeverity::severity);          \
  do {                                                      \
    if constexpr (LPP_INTL::LppSeverity::severity ==        \
                      LPP_INTL::LppSeverity::I ||           \
                  LPP_INTL::LppSeverity::severity ==        \
                      LPP_INTL::LppSeverity::D) {           \
      LOG_FIRST_N(INFO, n) << x;                            \
    } else if constexpr (LPP_INTL::LppSeverity::severity == \
                         LPP_INTL::LppSeverity::W) {        \
      LOG_FIRST_N(WARNING, n) << x;                         \
    } else if constexpr (LPP_INTL::LppSeverity::severity == \
                         LPP_INTL::LppSeverity::E) {        \
      LOG_FIRST_N(ERROR, n) << x;                           \
    } else if constexpr (LPP_INTL::LppSeverity::severity == \
                         LPP_INTL::LppSeverity::F) {        \
      LOG_FIRST_N(FATAL, n) << x;                           \
    }                                                       \
  } while (0)

#define LOG_TIMED(severity, t, x)                          \
  do {                                                     \
    if constexpr (LPP_INTL::LppSeverity::severity ==       \
                      LPP_INTL::LppSeverity::I ||          \
                  LPP_INTL::LppSeverity::severity ==       \
                      LPP_INTL::LppSeverity::W ||          \
                  LPP_INTL::LppSeverity::severity ==       \
                      LPP_INTL::LppSeverity::E ||          \
                  LPP_INTL::LppSeverity::severity ==       \
                      LPP_INTL::LppSeverity::F ||          \
                  LPP_INTL::LppSeverity::severity ==       \
                      LPP_INTL::LppSeverity::D) {          \
      LPP_INTL_LOG_EVERY_T(LppSeverity::severity, t) << x; \
    }                                                      \
  } while (0)

#ifndef MODE_DEFAULT
#define ROS_DEBUG(...) DLOG(INFO) << LPP_INTL::formatToString(__VA_ARGS__)
#define ROS_DEBUG_STREAM(x) DLOG(INFO) << x
#define ROS_INFO(...) LOG(INFO) << LPP_INTL::formatToString(__VA_ARGS__)
#define ROS_INFO_STREAM(x) LOG(INFO) << x
#define ROS_WARN(...) LOG(WARNING) << LPP_INTL::formatToString(__VA_ARGS__)
#define ROS_WARN_STREAM(x) LOG(WARNING) << x
#define ROS_ERROR(...) LOG(ERROR) << LPP_INTL::formatToString(__VA_ARGS__)
#define ROS_ERROR_STREAM(x) LOG(ERROR) << x
#define ROS_FATAL(...) LOG(FATAL) << LPP_INTL::formatToString(__VA_ARGS__)
#define ROS_FATAL_STREAM(x) LOG(FATAL) << x

#define ROS_DEBUG_COND(cond, x) DLOG_IF(INFO, cond) << x
#define ROS_DEBUG_STREAM_COND DLOG_IF(INFO, cond) << x;
#define ROS_INFO_COND(cond, x) LOG_IF(INFO, cond) << x
#define ROS_INFO_STREAM_COND(cond, x) LOG_IF(INFO, cond) << x
#define ROS_WARN_COND(cond, x) LOG_IF(WARNING, cond) << x
#define ROS_WARN_STREAM_COND(cond, x) LOG_IF(WARNING, cond) << x
#define ROS_ERROR_COND(cond, x) LOG_IF(ERROR, cond) << x
#define ROS_ERROR_STREAM_COND(cond, x) LOG_IF(ERROR, cond) << x
#define ROS_FATAL_COND(cond, x) LOG_IF(ERROR, cond) << x
#define ROS_FATAL_STREAM_COND(cond, x) LOG_IF(ERROR, cond) << x

#define ROS_DEBUG_ONCE(...) \
  LOG_FIRST_N(INFO, 1) << LPP_INTL::formatToString(__VA_ARGS__)
#define ROS_INFO_ONCE(...) \
  LOG_FIRST_N(INFO, 1) << LPP_INTL::formatToString(__VA_ARGS__)
#define ROS_WARN_ONCE(...) \
  LOG_FIRST_N(WARNING, 1) << LPP_INTL::formatToString(__VA_ARGS__)
#define ROS_ERROR_ONCE(...) \
  LOG_FIRST_N(ERROR, 1) << LPP_INTL::formatToString(__VA_ARGS__)
#define ROS_FATAL_ONCE(...) \
  LOG_FIRST_N(FATAL, 1) << LPP_INTL::formatToString(__VA_ARGS__)

#define ROS_DEBUG_THROTTLE(n, ...) \
  LPP_INTL_DLOG_EVERY_T(INFO, n) << LPP_INTL::formatToString(__VA_ARGS__);
#define ROS_DEBUG_STREAM_THROTTLE(n, x) LPP_INTL_DLOG_EVERY_T(INFO, n) << x
#define ROS_INFO_THROTTLE(n, ...)         \
  LPP_INTL_LOG_EVERY_T(LppSeverity::I, n) \
      << LPP_INTL::formatToString(__VA_ARGS__);
#define ROS_INFO_STREAM_THROTTLE(n, x) \
  LPP_INTL_LOG_EVERY_T(LppSeverity::I, n) << x
#define ROS_WARN_THROTTLE(n, ...)         \
  LPP_INTL_LOG_EVERY_T(LppSeverity::W, n) \
      << LPP_INTL::formatToString(__VA_ARGS__);
#define ROS_WARN_STREAM_THROTTLE(n, x) \
  LPP_INTL_LOG_EVERY_T(LppSeverity::W, n) << x
#define ROS_ERROR_THROTTLE(n, ...)        \
  LPP_INTL_LOG_EVERY_T(LppSeverity::E, n) \
      << LPP_INTL::formatToString(__VA_ARGS__);
#define ROS_ERROR_STREAM_THROTTLE(n, x) \
  LPP_INTL_LOG_EVERY_T(LppSeverity::E, n) << x
#define ROS_FATAL_THROTTLE(n, ...)        \
  LPP_INTL_LOG_EVERY_T(LppSeverity::F, n) \
      << LPP_INTL::formatToString(__VA_ARGS__);
#define ROS_FATAL_STREAM_THROTTLE(n, x) \
  LPP_INTL_LOG_EVERY_T(LppSeverity::F, n) << x
#endif

DIAG_POP
#endif

//! MODE_ROSLOG
#ifdef MODE_ROSLOG
#define LOG_IF(severity, cond) \
  if (cond) InternalLog(GlogSeverity::severity)

#define LOG_1(severity) LPP_INTL::InternalLog(LPP_INTL::GlogSeverity::severity)
#define LOG_2(severity, x) \
  LPP_INTL::InternalLog(LPP_INTL::LppSeverity::severity) << x
#define LOG_3(severity, cond, x) \
  if (cond) LPP_INTL::InternalLog(LppSeverity::severity) << x
#endif

#if defined MODE_ROSLOG || defined MODE_LPP || defined MODE_DEFAULT
#define LOG_EVERY(severity, n, x)                           \
  LPP_INTL::InternalLogCount::getInstance()                 \
      .update<unsigned int, LPP_INTL::PolicyType::EVERY_N>( \
          LPP_GET_KEY(), n, LPP_INTL::InternalLog() << x,   \
          toBase(LPP_INTL::LppSeverity::                    \
                     severity))  // NOLINT(bugprone-macro-parentheses)
#define LOG_FIRST(severity, n, x)                           \
  LPP_INTL::InternalLogCount::getInstance()                 \
      .update<unsigned int, LPP_INTL::PolicyType::FIRST_N>( \
          LPP_GET_KEY(), n, LPP_INTL::InternalLog() << x,   \
          toBase(LPP_INTL::LppSeverity::                    \
                     severity))  // NOLINT(bugprone-macro-parentheses)
#define LOG_TIMED(severity, t, x)                         \
  LPP_INTL::InternalLogCount::getInstance()               \
      .update<float, LPP_INTL::PolicyType::TIMED>(        \
          LPP_GET_KEY(), t, LPP_INTL::InternalLog() << x, \
          toBase(LPP_INTL::LppSeverity::                  \
                     severity))  // NOLINT(bugprone-macro-parentheses)
#endif

#if defined MODE_ROSLOG || defined MODE_LPP || MODE_NOLOG
/**
 * Replace glog's FLAGS_v and VLOG_IS_ON to avoid linker errors
 * if glog is installed but not linked to lpp.
 */
[[maybe_unused]] inline static int32_t LPP_FLAGS_v;

#ifdef LPP_GLOG_SUPPORTED
#define FLAGS_v LPP_FLAGS_v
#endif  // LPP_GLOG_SUPPORTED
#endif  // defined MODE_ROSLOG || defined MODE_LPP || MODE_NOLOG

#if defined MODE_ROSLOG || defined MODE_LPP
#define LOG_EVERY_N(severity, n)                                            \
  LPP_INTL::InternalPolicyLog<unsigned int>(                                \
      LPP_GET_KEY(), n, LPP_INTL::toBase(LPP_INTL::GlogSeverity::severity), \
      LPP_INTL::PolicyType::EVERY_N)
#define LOG_EVERY_T(severity, t)                                              \
  LPP_GLOG_V0_6_WARNING("LOG_EVERY_T is only defined in GLOG v0.6 or newer.") \
  LPP_INTL::InternalPolicyLog<float>(                                         \
      LPP_GET_KEY(), t, LPP_INTL::toBase(LPP_INTL::GlogSeverity::severity),   \
      LPP_INTL::PolicyType::TIMED)
#define LOG_IF_EVERY_N(severity, condition, n) \
  if (condition) LOG_EVERY_N(severity, n)
#define LOG_FIRST_N(severity, n)                                            \
  LPP_INTL::InternalPolicyLog<unsigned int>(                                \
      LPP_GET_KEY(), n, LPP_INTL::toBase(LPP_INTL::GlogSeverity::severity), \
      LPP_INTL::PolicyType::FIRST_N)

#define DLOG(severity)                               \
  LPP_ASSERT_GLOG(LPP_INTL::GlogSeverity::severity); \
  LPP_INTL::InternalLog(LPP_INTL::BaseSeverity::DEBUG)
#define DLOG_EVERY_N(severity, n)                                          \
  LPP_ASSERT_GLOG(LPP_INTL::GlogSeverity::severity);                       \
  LPP_INTL::InternalPolicyLog<unsigned int>(LPP_GET_KEY(), n,              \
                                            LPP_INTL::BaseSeverity::DEBUG, \
                                            LPP_INTL::PolicyType::EVERY_N)
#define DLOG_EVERY_T(severity, t)                                   \
  LPP_WARN("DLOG_FIRST_N is a Log++ extension");                    \
  LPP_ASSERT_GLOG(LPP_INTL::GlogSeverity::severity);                \
  LPP_INTL::InternalPolicyLog<float>(LPP_GET_KEY(), t,              \
                                     LPP_INTL::BaseSeverity::DEBUG, \
                                     LPP_INTL::PolicyType::TIMED)
#define DLOG_FIRST_N(severity, n)                                          \
  LPP_WARN("DLOG_FIRST_N is a Log++ extension");                           \
  LPP_ASSERT_GLOG(LPP_INTL::GlogSeverity::severity);                       \
  LPP_INTL::InternalPolicyLog<unsigned int>(LPP_GET_KEY(), n,              \
                                            LPP_INTL::BaseSeverity::DEBUG, \
                                            LPP_INTL::PolicyType::FIRST_N)
#define DLOG_IF_EVERY_N(severity, condition, n)                            \
  LPP_ASSERT_GLOG(LPP_INTL::GlogSeverity::severity);                       \
  if (condition)                                                           \
  LPP_INTL::InternalPolicyLog<unsigned int>(LPP_GET_KEY(), n,              \
                                            LPP_INTL::BaseSeverity::DEBUG, \
                                            LPP_INTL::PolicyType::EVERY_N)
#define LOG_STRING(severity, ptr)                                              \
  LPP_ASSERT_GLOG(LPP_INTL::GlogSeverity::severity);                           \
  LPP_INTL::InternalGlogLogStringLog(toBase(LPP_INTL::GlogSeverity::severity), \
                                     ptr)

#undef VLOG_IS_ON
#define VLOG_IS_ON(verboselevel) LPP_FLAGS_v >= (verboselevel) ? true : false
#define VLOG(verboselevel)                                 \
  LPP_INTL::InternalCondLog(LPP_INTL::BaseSeverity::DEBUG, \
                            VLOG_IS_ON(verboselevel))
#define VLOG_EVERY_N(verboselevel, n) \
  if (VLOG_IS_ON(verboselevel)) LOG_EVERY_N(DEBUG, n)
#define VLOG_IF(verboselevel, condition) \
  if (condition) VLOG(verboselevel)
#define VLOG_IF_EVERY_N(verboselevel, condition, n) \
  if (VLOG_IS_ON(verboselevel) && condition) LOG_EVERY_N(DEBUG, n)
#endif

//! MODE_LPP
#ifdef MODE_LPP
#define ROS_DEBUG(...) LOG_2(D, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_DEBUG_STREAM(x) LOG_2(D, x)
#define ROS_INFO(...) LOG_2(I, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_INFO_STREAM(x) LOG_2(I, x)
#define ROS_WARN(...) LOG_2(W, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_WARN_STREAM(x) LOG_2(W, x)
#define ROS_ERROR(...) LOG_2(E, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_ERROR_STREAM(x) LOG_2(E, x)
#define ROS_FATAL(...) LOG_2(F, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_FATAL_STREAM(x) LOG_2(F, x)

#define ROS_DEBUG_COND(cond, x) LOG_3(D, cond, x)
#define ROS_DEBUG_STREAM_COND(cond, x) LOG_3(D, cond, x)
#define ROS_INFO_COND(cond, x) LOG_3(I, cond, x)
#define ROS_INFO_STREAM_COND(cond, x) LOG_3(I, cond, x)
#define ROS_WARN_COND(cond, x) LOG_3(W, cond, x)
#define ROS_WARN_STREAM_COND(cond, x) LOG_3(W, cond, x)
#define ROS_ERROR_COND(cond, x) LOG_3(E, cond, x)
#define ROS_ERROR_STREAM_COND(cond, x) LOG_3(E, cond, x)
#define ROS_FATAL_COND(cond, x) LOG_3(F, cond, x)
#define ROS_FATAL_STREAM_COND(cond, x) LOG_3(F, cond, x)

#define ROS_DEBUG_ONCE(...) \
  LOG_FIRST(D, 1, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_INFO_ONCE(...) \
  LOG_FIRST(I, 1, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_WARN_ONCE(...) \
  LOG_FIRST(W, 1, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_ERROR_ONCE(...) \
  LOG_FIRST(E, 1, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_FATAL_ONCE(...) \
  LOG_FIRST(F, 1, LPP_INTL::formatToString(__VA_ARGS__))

#define ROS_DEBUG_THROTTLE(period, ...) \
  LOG_TIMED(D, period, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_DEBUG_STREAM_THROTTLE(period, x) LOG_TIMED(D, period, x)
#define ROS_INFO_THROTTLE(period, ...) \
  LOG_TIMED(I, period, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_INFO_STREAM_THROTTLE(period, x) LOG_TIMED(I, period, x)
#define ROS_WARN_THROTTLE(period, ...) \
  LOG_TIMED(W, period, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_WARN_STREAM_THROTTLE(period, x) LOG_TIMED(W, period, x)
#define ROS_ERROR_THROTTLE(period, ...) \
  LOG_TIMED(E, period, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_ERROR_STREAM_THROTTLE(period, x) LOG_TIMED(E, period, x)
#define ROS_FATAL_THROTTLE(period, ...) \
  LOG_TIMED(F, period, LPP_INTL::formatToString(__VA_ARGS__))
#define ROS_FATAL_STREAM_THROTTLE(period, x) LOG_TIMED(F, period, x)

#define LOG_IF(severity, cond) \
  if (cond) LPP_INTL::InternalLog(GlogSeverity::severity)
#define LOG_1(severity) LPP_INTL::InternalLog(LPP_INTL::GlogSeverity::severity)
#endif

#if defined MODE_LPP || defined MODE_DEFAULT
#define LOG_2(severity, x)                               \
  LPP_INTL::InternalLog(LPP_INTL::LppSeverity::severity) \
      << x  // NOLINT(bugprone-macro-parentheses)
#define LOG_3(severity, cond, x)                         \
  if (cond)                                              \
  LPP_INTL::InternalLog(LPP_INTL::LppSeverity::severity) \
      << x  // NOLINT(bugprone-macro-parentheses)
#endif

//! MODE_NOLOG

#ifdef MODE_NOLOG
// lpp
#define LOG_2(severity, x)               \
  (void)LPP_INTL::LppSeverity::severity; \
  InternalLog() << x
#define LOG_EVERY(severity, n, x)                          \
  (void)LPP_INTL::LppSeverity::severity;                   \
  static_assert(std::is_integral<decltype(n)>::value, ""); \
  InternalLog()
#define LOG_FIRST(severity, n, x)                          \
  (void)LPP_INTL::LppSeverity::severity;                   \
  static_assert(std::is_integral<decltype(n)>::value, ""); \
  InternalLog()
#define LOG_TIMED(severity, t, x)                          \
  (void)LPP_INTL::LppSeverity::severity;                   \
  static_assert(std::is_integral<decltype(t)>::value, ""); \
  InternalLog()

// glog
#define LOG_1(severity)                   \
  (void)LPP_INTL::GlogSeverity::severity; \
  InternalLog()
#define DLOG(severity)                    \
  (void)LPP_INTL::GlogSeverity::severity; \
  InternalLog()
#define DLOG_EVERY_N(severity, n)         \
  (void)LPP_INTL::GlogSeverity::severity; \
  InternalLog()
#define LOG_EVERY_N(severity, n)          \
  (void)LPP_INTL::GlogSeverity::severity; \
  InternalLog()
#define DLOG_FIRST_N(severity, n)                          \
  (void)LPP_INTL::GlogSeverity::severity;                  \
  static_assert(std::is_integral<decltype(n)>::value, ""); \
  InternalLog()
#define LOG_FIRST_N(severity, n)                           \
  (void)LPP_INTL::GlogSeverity::severity;                  \
  static_assert(std::is_integral<decltype(n)>::value, ""); \
  InternalLog()
#define DLOG_IF_EVERY_N(severity, cond, n)                   \
  (void)LPP_INTL::GlogSeverity::severity;                    \
  static_assert(std::is_same<decltype(cond), bool>::value && \
                    std::is_integral<decltype(n)>::value,    \
                "");                                         \
  InternalLog()
#define LOG_IF_EVERY_N(severity, cond, n) DLOG_IF_EVERY_N(severity, cond, n)
#define LOG_STRING(severity, ptr int32_value.data = 12345;)             \
  (void)LPP_INTL::GlogSeverity::severity;                               \
  static_assert(                                                        \
      std::is_same<decltype(ptr), std::vector<std::string> *>::value || \
          std::is_same<decltype(ptr), std::nullptr_t>::value,           \
      "");                                                              \
  InternalLog()
#define VLOG(verboselevel)                                            \
  static_assert(std::is_integral<decltype(verboselevel)>::value, ""); \
  InternalLog()
#define VLOG_IF(verboselevel, condition)                            \
  static_assert(std::is_integral<decltype(verboselevel)>::value &&  \
                    std::is_same<decltype(condition), bool>::value, \
                "");                                                \
  InternalLog()
#define VLOG_EVERY_N(verboselevel, n)                              \
  static_assert(std::is_integral<decltype(verboselevel)>::value && \
                    std::is_integral<decltype(n)>::value,          \
                "");                                               \
  InternalLog()
#define VLOG_IF_EVERY_N(verboselevel, condition, n)                   \
  static_assert(std::is_integral<decltype(verboselevel)>::value &&    \
                    std::is_same<decltype(condition), bool>::value && \
                    std::is_integral<decltype(n)>::value,             \
                "");                                                  \
  InternalLog()
#define DLOG_EVERY_T(severity, t)                          \
  (void)LPP_INTL::GlogSeverity::severity;                  \
  static_assert(std::is_integral<decltype(t)>::value, ""); \
  InternalLog()
#define LOG_EVERY_T(severity, t) DLOG_EVERY_T(severity, t)

// ros
#define ROS_DEBUG(...) LOG_2(D, LPP_INTL::emptyString(__VA_ARGS__))
#define ROS_INFO(...) LOG_2(I, LPP_INTL::emptyString(__VA_ARGS__))
#define ROS_WARN(...) LOG_2(W, LPP_INTL::emptyString(__VA_ARGS__))
#define ROS_ERROR(...) LOG_2(E, LPP_INTL::emptyString(__VA_ARGS__))
#define ROS_FATAL(...) LOG_2(F, LPP_INTL::emptyString(__VA_ARGS__))

#define ROS_DEBUG_STREAM(x) LPP_INTL::emptyString(x)
#define ROS_INFO_STREAM(x) LPP_INTL::emptyString(x)
#define ROS_WARN_STREAM(x) LPP_INTL::emptyString(x)
#define ROS_ERROR_STREAM(x) LPP_INTL::emptyString(x)
#define ROS_FATAL_STREAM(x) LPP_INTL::emptyString(x)

#define ROS_DEBUG_ONCE(...) \
  LOG_2(D, std::string(LPP_INTL::emptyString(__VA_ARGS__)))
#define ROS_INFO_ONCE(...) \
  LOG_2(I, std::string(LPP_INTL::emptyString(__VA_ARGS__)))
#define ROS_WARN_ONCE(...) \
  LOG_2(W, std::string(LPP_INTL::emptyString(__VA_ARGS__)))
#define ROS_ERROR_ONCE(...) \
  LOG_2(E, std::string(LPP_INTL::emptyString(__VA_ARGS__)))
#define ROS_FATAL_ONCE(...) \
  LOG_2(F, std::string(LPP_INTL::emptyString(__VA_ARGS__)))

#define ROS_DEBUG_THROTTLE(t, x)                           \
  static_assert(std::is_integral<decltype(t)>::value, ""); \
  LPP_INTL::emptyString(x)
#define ROS_DEBUG_STREAM_THROTTLE(t, x)                    \
  static_assert(std::is_integral<decltype(t)>::value, ""); \
  InternalLog()
#define ROS_INFO_THROTTLE(t, x) ROS_DEBUG_THROTTLE(t, x)
#define ROS_INFO_STREAM_THROTTLE(t, x) ROS_DEBUG_STREAM_THROTTLE(t, x)
#define ROS_WARN_THROTTLE(t, x) ROS_DEBUG_THROTTLE(t, x)
#define ROS_WARN_STREAM_THROTTLE(t, x) ROS_DEBUG_STREAM_THROTTLE(t, x)
#define ROS_ERROR_THROTTLE(t, x) ROS_DEBUG_THROTTLE(t, x)
#define ROS_ERROR_STREAM_THROTTLE(t, x) ROS_DEBUG_STREAM_THROTTLE(t, x)
#define ROS_FATAL_THROTTLE(t, x) ROS_DEBUG_THROTTLE(t, x)
#define ROS_FATAL_STREAM_THROTTLE(t, x) ROS_DEBUG_STREAM_THROTTLE(t, x)
#endif

namespace lpp {
namespace internal {
#ifdef MODE_NOLOG
//! Used to disable logging for printf(3) like syntax
template <typename... Args>
constexpr inline const char *emptyString([[maybe_unused]] const char *f,
                                         [[maybe_unused]] Args... args) {
  return "";
}

[[maybe_unused]] constexpr const char *emptyString(
    [[maybe_unused]] const char *str) {
  return "";
}

[[maybe_unused]] constexpr const char *emptyString(
    [[maybe_unused]] const std::string &str) {
  return "";
}
#endif
//! Composes a string with the same text that would be printed if format was
//! used on printf(3)
template <typename... Args>
inline std::string formatToString(const char *f, Args... args) {
  size_t sz = snprintf(nullptr, 0, f, args...);
  if (sz == 0) {
    return "";
  }
  char *buf = (char *)malloc(sz + 1);
  snprintf(buf, sz + 1, f, args...);
  return buf;
}

inline std::string formatToString(const char *str) { return str; }

enum class LppSeverity { D, I, W, E, F };

enum class GlogSeverity { DEBUG, INFO, WARNING, ERROR, FATAL };

inline BaseSeverity toBase(LppSeverity lpp_severity) {
  return (BaseSeverity)lpp_severity;
}

inline BaseSeverity toBase(GlogSeverity glog_severity) {
  return (BaseSeverity)glog_severity;
}

//! Internal log class
class InternalLog {
 public:
  InternalLog() { should_print_ = false; };

  explicit InternalLog(BaseSeverity base_severity) : severity_(base_severity) {}
  explicit InternalLog(LppSeverity lpp_severity)
      : severity_(toBase(lpp_severity)) {}
  explicit InternalLog(GlogSeverity glog_severity)
      : severity_(toBase(glog_severity)) {}

  InternalLog(InternalLog const &log) : severity_(log.severity_) {
    ss << log.ss.str();
  }

  BaseSeverity severity_{};
  std::stringstream ss{};

  virtual ~InternalLog() {
    if (!should_print_) {
      return;
    }
#if defined MODE_ROSLOG
    switch (severity_) {
      case BaseSeverity::DEBUG:
        ROS_DEBUG_STREAM(ss.str());
        break;
      case BaseSeverity::INFO:
        ROS_INFO_STREAM(ss.str());
        break;
      case BaseSeverity::WARN:
        ROS_WARN_STREAM(ss.str());
        break;
      case BaseSeverity::ERROR:
        ROS_ERROR_STREAM(ss.str());
        break;
      case BaseSeverity::FATAL:
        ROS_FATAL_STREAM(ss.str());
        break;
    }
#endif
#if defined MODE_LPP || defined MODE_DEFAULT
    lpp::internal::logging.call(severity_, ss.str());
#endif
#if defined MODE_GLOG
    switch (severity_) {
      case BaseSeverity::DEBUG:
        LOG(INFO) << ss.str();
        break;
      case BaseSeverity::INFO:
        LOG(INFO) << ss.str();
        break;
      case BaseSeverity::WARN:
        LOG(WARNING) << ss.str();
        break;
      case BaseSeverity::ERROR:
        LOG(ERROR) << ss.str();
        break;
      case BaseSeverity::FATAL:
        LOG(FATAL) << ss.str();
        break;
    }
#endif
  }

 protected:
  bool should_print_{true};
};

template <typename T>
InternalLog &&operator<<(InternalLog &&wrap, T const &whatever) {
  wrap.ss << whatever;
  return std::move(wrap);
}

//! Used for glog's VLOG macro
class InternalCondLog : public InternalLog {
 public:
  explicit InternalCondLog(BaseSeverity severity, bool cond)
      : InternalLog(severity) {
    should_print_ = cond;
  }
  ~InternalCondLog() override = default;
};

class LogPolicyBase {
 public:
  virtual void update() = 0;
  [[nodiscard]] virtual bool shouldLog() const = 0;
  virtual void onLog() {};
  virtual ~LogPolicyBase() = default;
};

template <typename T>
class LogPolicy : public LogPolicyBase {
 public:
  explicit LogPolicy(T max) : max_(max) {}
  ~LogPolicy() override = default;

 protected:
  T max_{0};
};

class CountableLogPolicy : public LogPolicy<unsigned int> {
 public:
  explicit CountableLogPolicy(unsigned int max) : LogPolicy(max) {}

 protected:
  unsigned int counter_{0};
};

class OccasionPolicy : public CountableLogPolicy {
 public:
  explicit OccasionPolicy(unsigned int max) : CountableLogPolicy(max) {
    counter_ = max_;
  }

  inline void update() override {
    should_log_ = false;

    if (counter_ % max_ == 0) {
      should_log_ = true;
      counter_ = 0;
    }
    counter_++;
  }

  [[nodiscard]] inline bool shouldLog() const override { return should_log_; }

  ~OccasionPolicy() override = default;

 private:
  bool should_log_{false};
};

class FirstNOccurrencesPolicy : public CountableLogPolicy {
 public:
  explicit FirstNOccurrencesPolicy(unsigned int max)
      : CountableLogPolicy(max) {}
  inline void update() override {
    if (!is_n_occurences_reached) {
      counter_++;
    }

    if (counter_ > max_) {
      is_n_occurences_reached = true;
    }
  }

  [[nodiscard]] inline bool shouldLog() const override {
    return !is_n_occurences_reached;
  }

  ~FirstNOccurrencesPolicy() override = default;

 private:
  bool is_n_occurences_reached = false;
};

using namespace std::chrono;

class TimePolicy : public LogPolicy<float> {
 public:
  explicit TimePolicy(float max) : LogPolicy(max) {};

  inline void update() override {
    now_ = duration_cast<microseconds>(system_clock::now().time_since_epoch())
               .count();
  }

  [[nodiscard]] inline bool shouldLog() const override {
    if (now_ >= last_ + static_cast<long>(max_ * 1000000.f)) {
      return true;
    }
    return false;
  }

  void onLog() override { last_ = now_; }

  ~TimePolicy() override = default;

 private:
  long now_{0};
  long last_{0};
};

enum PolicyType { FIRST_N, EVERY_N, TIMED };

typedef std::shared_ptr<LogPolicyBase> LogPolicyPtr;

class LogPolicyFactory {
 public:
  template <typename T>
  static LogPolicyPtr create(PolicyType policy_type, T max) {
    switch (policy_type) {
      case FIRST_N:
        return std::make_shared<FirstNOccurrencesPolicy>(max);
      case EVERY_N:
        return std::make_shared<OccasionPolicy>(max);
      case TIMED:
        return std::make_shared<TimePolicy>(max);
      default:
        abort();
    }
  }
};

struct LogStatementData {
  LogStatementData(LogPolicyPtr log_policy, BaseSeverity severity_type)
      : log_policy_(std::move(log_policy)), severity_type_(severity_type) {}
  LogPolicyPtr log_policy_;
  std::string msg{};
  BaseSeverity severity_type_;
};

class InternalLogCount {
 public:
  static InternalLogCount &getInstance() {
    static InternalLogCount instance;
    return instance;
  }

  template <typename T, PolicyType policyType>
  inline void update(const std::string &key, T max,
                     const InternalLog &internal_log,
                     const BaseSeverity base_severity) {
    // Check if T is a float and only allow it for PolicyType::TIMED
    static_assert(
        std::is_same<T, unsigned int>::value || std::is_same<T, float>::value,
        "Only int or float is allowed for T");

    if constexpr (std::is_same<T, float>::value) {
      static_assert(policyType == PolicyType::TIMED,
                    "Only PolicyType::TIMED can be used with float");
    }
    update(key, max, internal_log.ss.str(), base_severity, policyType);
  }

  template <typename T>
  inline void update(const std::string &key, T max, const std::string &log_msg,
                     const BaseSeverity base_severity, PolicyType policy_type) {
    updatePolicy(key, max, log_msg, base_severity, policy_type);
    mtx_.lock();
    logIfReady(key, log_msg);
    mtx_.unlock();
  }

  template <typename T>
  inline void updatePolicy(const std::string &key, T max,
                           const std::string &log_msg,
                           BaseSeverity base_severity, PolicyType policy_type) {
    mtx_.lock();
    if (!keyExists(key)) {
      LogStatementData data(LogPolicyFactory::create(policy_type, max),
                            base_severity);
      data.msg = log_msg;
      updateLogPolicyData(&data);
      occurences_.insert({key, data});
    } else {
      LogStatementData *data = &occurences_.at(key);
      updateLogPolicyData(data);
    }
    mtx_.unlock();
  }

  inline bool shouldLog(const std::string &key) {
    mtx_.lock();
    bool res = occurences_.at(key).log_policy_->shouldLog();
    mtx_.unlock();
    return res;
  }

  inline void log(const std::string &key) {
    mtx_.lock();
    occurences_.at(key).log_policy_->onLog();
    mtx_.unlock();
  }

 private:
  inline bool keyExists(const std::string &key) {
    return occurences_.find(key) != occurences_.end();
  }

  static inline void updateLogPolicyData(LogStatementData *data) {
    data->log_policy_->update();
  }

  inline void logIfReady(const std::string &key, const std::string &log_msg) {
    LogStatementData *data = &occurences_.at(key);
    if (data->log_policy_->shouldLog()) {
      data->log_policy_->onLog();
      data->msg = log_msg;
      InternalLog(data->severity_type_) << data->msg;
    }
  }

  InternalLogCount() = default;
  std::unordered_map<std::string, LogStatementData> occurences_{};
  std::mutex mtx_{};
};

/**
 * @extends InternalLog
 * Class to process LOG_STRING() macro.
 *
 * LOG_STRING() should log if the vector pointer is NULL,
 * otherwise the message must not be logged and the string
 * is stored in the vector.
 */
class InternalGlogLogStringLog : public InternalLog {
 public:
  InternalGlogLogStringLog(BaseSeverity base_severity,
                           std::vector<std::string> *vecptr)
      : InternalLog(base_severity), vecptr_(vecptr) {
    if (vecptr != nullptr) {
      should_print_ = false;
    }
  };

  ~InternalGlogLogStringLog() override {
    if (vecptr_ != nullptr) {
      vecptr_->push_back(ss.str());
    }
  }

 private:
  std::vector<std::string> *vecptr_;
};

template <typename T>
class InternalPolicyLog : public InternalLog {
 public:
  InternalPolicyLog(std::string key, T n, BaseSeverity base_severity,
                    PolicyType policy_type)
      : InternalLog(base_severity),
        key_(std::move(key)),
        n_(n),
        policy_type_(policy_type) {
    should_print_ = false;
  };

  ~InternalPolicyLog() override {
    if (should_update_) {
      InternalLogCount::getInstance().update(key_, n_, ss.str(), severity_,
                                             policy_type_);
    }
  }

 protected:
  bool should_update_{true};
  std::string key_{};
  T n_{};
  PolicyType policy_type_{};
};

class LppGlogExtensionLog : public InternalPolicyLog<unsigned int> {
 public:
  LppGlogExtensionLog(std::string key, unsigned int n,
                      GlogSeverity glog_severity, PolicyType policy_type,
                      std::function<void(const std::string &str)> fn)
      : InternalPolicyLog(std::move(key), n, toBase(glog_severity),
                          policy_type),
        fn_(std::move(fn)) {
    should_print_ = false;
    should_update_ = false;  // Disable update in InternalPolicyLog destructor
  }

  ~LppGlogExtensionLog() override {
    InternalLogCount::getInstance().updatePolicy(key_, n_, ss.str(), severity_,
                                                 policy_type_);
    if (InternalLogCount::getInstance().shouldLog(key_)) {
      InternalLogCount::getInstance().log(key_);
      fn_(ss.str());
    }
  }

 private:
  std::function<void(const std::string &str)> fn_;
};
}  // namespace internal
}  // namespace lpp

#define LPP_GET_KEY() std::string(__FILE__) + std::to_string(__LINE__)
DIAG_POP

// Undefine macros that are only needed in this file
#undef LPP_GLOG_SUPPORTED
#undef LPP_ROSLOG_SUPPORTED

#endif  // LOG__LOG_H_
