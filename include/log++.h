//
// Created by 4c3y on 18.08.22.
//

#ifndef LOG__LOG_H_
#define LOG__LOG_H_

#include <algorithm>
#include <utility>
#include <set>
#include <sstream>
#include <unordered_map>

//! Check if libraries are available at compile time
#if __has_include(<glog/logging.h>)
#define GLOG_SUPPORTED
#endif

#if __has_include(<ros/console.h>)
#define ROSLOG_SUPPORTED
#endif

//! Initialization logic
namespace lpp {
namespace internal {

class Init {
 public:
  bool is_lpp_initialized = false;
  bool is_glog_initialized = false;
};
inline Init lppInit;
}
}

//! assert if mode is supported by the libraries available
#if defined MODE_GLOG && !defined GLOG_SUPPORTED
#error Logging Mode is set to glog but glog was not found
#endif

#if defined MODE_ROSLOG && !defined ROSLOG_SUPPORTED
#error Logging Mode is set to roslog but roslog was not found
#endif


//! Includes
#if defined GLOG_SUPPORTED && defined MODE_GLOG
#include <glog/logging.h>
#endif // GLOG_SUPPORTED

#if defined ROSLOG_SUPPORTED && defined MODE_ROSLOG
#include <ros/console.h>
#endif


//! un-define macros to avoid conflicts
#ifdef GLOG_SUPPORTED
#undef LOG
#endif


//! Redefine log methods
#if defined GLOG_SUPPORTED && !defined MODE_GLOG
#undef LOG_IF
#undef LOG_EVERY_N
#undef LOG_FIRST_N
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

#undef ROS_INFO_COND
#undef ROS_INFO_STREAM_COND
#undef ROS_WARN_COND
#undef ROS_WARN_STREAM_COND
#undef ROS_ERROR_COND
#undef ROS_ERROR_STREAM_COND
#undef ROS_FATAL_COND
#undef ROS_FATAL_STREAM_COND
#endif

using namespace lpp::internal;
//! Log init
#ifdef MODE_GLOG

//! If LOG_INIT is called more than once, do nothing.
#define LOG_INIT(argv0) if (!lppInit.is_glog_initialized) { \
google::InitGoogleLogging(argv0); lppInit.is_glog_initialized = true;} \
lppInit.is_lpp_initialized = true; FLAGS_logtostderr = true
#else
#define LOG_INIT(argv0) lppInit.is_lpp_initialized = true
#endif


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
 * LOG_3 = LPP conditional syntax
 */

//! MODE_GLOG
#if defined MODE_GLOG || defined MODE_DEFAULT
#define LOG_1(severity) COMPACT_GOOGLE_LOG_ ## severity.stream()
#endif

#ifdef MODE_GLOG
#pragma clang diagnostic push
#pragma ide diagnostic ignored "bugprone-macro-parentheses"

#define LOG_2(severity, x) \
if      (strcmp(#severity, "I") == 0) {LOG_1(INFO) << x;}        \
else if (strcmp(#severity, "W") == 0) {LOG_1(WARNING) << x;}     \
else if (strcmp(#severity, "E") == 0) {LOG_1(ERROR) << x;}       \
else if (strcmp(#severity, "F") == 0) {LOG_1(FATAL) << x;} true

//Add true at the end to make semicolons mandatory. Compiles to nothing.
#define LOG_3(severity, cond, x) if (cond) { LOG_2(severity, x);} true
#define LOG_EVERY(severity, n, x) \
if      (strcmp(#severity, "I") == 0) {LOG_EVERY_N(INFO, n) << x;}        \
else if (strcmp(#severity, "W") == 0) {LOG_EVERY_N(WARNING, n) << x;}     \
else if (strcmp(#severity, "E") == 0) {LOG_EVERY_N(ERROR, n) << x;}       \
else if (strcmp(#severity, "F") == 0) {LOG_EVERY_N(FATAL, n) << x;} true

#define LOG_FIRST(severity, n, x) \
if      (strcmp(#severity, "I") == 0) {LOG_FIRST_N(INFO, n) << x;}        \
else if (strcmp(#severity, "W") == 0) {LOG_FIRST_N(WARNING, n) << x;}     \
else if (strcmp(#severity, "E") == 0) {LOG_FIRST_N(ERROR, n) << x;}       \
else if (strcmp(#severity, "F") == 0) {LOG_FIRST_N(FATAL, n) << x;} true


#define ROS_INFO(x) LOG(INFO) << x
#define ROS_INFO_STREAM(x) LOG(INFO) << x
#define ROS_WARN(x) LOG(WARNING) << x
#define ROS_WARN_STREAM(x) LOG(WARNING) << x
#define ROS_ERROR(x) LOG(ERROR) << x
#define ROS_ERROR_STREAM(x) LOG(ERROR) << x
#define ROS_FATAL(x) LOG(FATAL) << x
#define ROS_FATAL_STREAM(x) LOG(FATAL) << x
#define ROS_INFO_COND(cond, x) LOG_IF(INFO, cond) << x
#define ROS_INFO_STREAM_COND(cond, x) LOG_IF(INFO, cond) << x
#define ROS_WARN_COND(cond, x) LOG_IF(WARNING, cond) << x
#define ROS_WARN_STREAM_COND(cond, x) LOG_IF(WARNING, cond) << x
#define ROS_ERROR_COND(cond, x) LOG_IF(ERROR, cond) << x
#define ROS_ERROR_STREAM_COND(cond, x) LOG_IF(ERROR, cond) << x
#define ROS_FATAL_COND(cond, x) LOG_IF(ERROR, cond) << x
#define ROS_FATAL_STREAM_COND(cond, x) LOG_IF(ERROR, cond) << x

#pragma clang diagnostic pop
#endif


//! MODE_ROSLOG
#ifdef MODE_ROSLOG
#define LOG_IF(severity, cond) if (cond) InternalLog(#severity)

#define LOG_1(severity) InternalLog(#severity)
#define LOG_2(severity, x) InternalLog(#severity) << x
#define LOG_3(severity, cond, x) if (cond) InternalLog(#severity) << x

#endif


#if defined MODE_ROSLOG || defined MODE_LPP
#define LOG_EVERY(severity, n, x) InternalLogCount::getInstance().update(LPP_GET_KEY(), n, InternalLog() << x, #severity, PolicyType::EVERY_N)
#define LOG_EVERY_N(severity, n)  InternalPolicyLog(LPP_GET_KEY(), n, #severity, PolicyType::EVERY_N)
#define LOG_FIRST(severity, n, x) InternalLogCount::getInstance().update(LPP_GET_KEY(), n, InternalLog() << x, #severity, PolicyType::FIRST_N)
#define LOG_FIRST_N(severity, n)  InternalPolicyLog(LPP_GET_KEY(), n, #severity, PolicyType::FIRST_N)
#endif

//! MODE_LPP
#ifdef MODE_LPP

#define ROS_INFO(x) LOG_2(I, x)
#define ROS_INFO_STREAM(x) LOG_2(I, x)
#define ROS_WARN(x) LOG_2(W, x)
#define ROS_WARN_STREAM(x) LOG_2(W, x)
#define ROS_ERROR(x) LOG_2(E, x)
#define ROS_ERROR_STREAM(x) LOG_2(E, x)
#define ROS_FATAL(x) LOG_2(F, x)
#define ROS_FATAL_STREAM(x) LOG_2(F, x)

#define ROS_INFO_COND(cond, x) LOG_3(I, cond, x)
#define ROS_INFO_STREAM_COND(cond, x) LOG_3(I, cond, x)
#define ROS_WARN_COND(cond, x) LOG_3(W, cond, x)
#define ROS_WARN_STREAM_COND(cond, x) LOG_3(W, cond, x)
#define ROS_ERROR_COND(cond, x) LOG_3(E, cond, x)
#define ROS_ERROR_STREAM_COND(cond, x) LOG_3(E, cond, x)
#define ROS_FATAL_COND(cond, x) LOG_3(F, cond, x)
#define ROS_FATAL_STREAM_COND(cond, x) LOG_3(F, cond, x)

#define LOG_IF(severity, cond) if (cond) InternalLog(#severity)

#define LOG_1(severity) InternalLog(#severity)
#define LOG_2(severity, x) InternalLog(#severity) << x // NOLINT(bugprone-macro-parentheses)
#define LOG_3(severity, cond, x) if (cond) InternalLog(#severity) << x // NOLINT(bugprone-macro-parentheses)
#endif

enum SeverityType {
  INFO,
  WARN,
  ERROR,
  FATAL
};

//! Internal log class
class InternalLog {
 public:
  InternalLog() {
    should_print_ = false;
  };

  explicit InternalLog(SeverityType severity_type) : severity_(severity_type) {}
  explicit InternalLog(const std::string &severity) {
    severity_ = getSeverityFromString(severity);
  }

  InternalLog(InternalLog const &log) : severity_(log.severity_) {
    ss << log.ss.str();
  }
  SeverityType severity_{};
  std::stringstream ss{};
#ifdef MODE_ROSLOG
  ~InternalLog() {
    if (!should_print_) {
      return;
    }
    switch (severity_) {
      case SeverityType::INFO:ROS_INFO_STREAM(ss.str());
        break;
      case SeverityType::WARN:ROS_WARN_STREAM(ss.str());
        break;
      case SeverityType::ERROR:ROS_ERROR_STREAM(ss.str());
        break;
      case SeverityType::FATAL:ROS_FATAL_STREAM(ss.str());
        break;
    }
  }
#endif

#ifdef MODE_LPP
  ~InternalLog() {
    if (!should_print_) {
      return;
    }
    switch (severity_) {
      case SeverityType::INFO:std::cout << "INFO  " << ss.str() << std::endl;
        break;
      case SeverityType::WARN:std::cout << "WARN  " << ss.str() << std::endl;
        break;
      case SeverityType::ERROR:std::cout << "ERROR " << ss.str() << std::endl;
        break;
      case SeverityType::FATAL:std::cout << "FATAL " << ss.str() << std::endl;
        break;
    }
  }
#endif
 private:
  bool should_print_{true};
  static SeverityType getSeverityFromString(const std::string &str) {
    if (INFO.find(str) != INFO.end()) {
      return SeverityType::INFO;
    } else if (WARNING.find(str) != WARNING.end()) {
      return SeverityType::WARN;
    } else if (ERROR.find(str) != ERROR.end()) {
      return SeverityType::ERROR;
    } else if (FATAL.find(str) != FATAL.end()) {
      return SeverityType::FATAL;
    }
    abort();
  }

  inline static const std::set<std::string> INFO{"I", "INFO"};
  inline static const std::set<std::string> WARNING{"W", "WARNING"};
  inline static const std::set<std::string> ERROR{"E", "ERROR"};
  inline static const std::set<std::string> FATAL{"F", "FATAL"};
};

template<typename T>
InternalLog &&operator<<(InternalLog &&wrap, T const &whatever) {
  wrap.ss << whatever;
  return std::move(wrap);
}

class LogPolicy {
 public:
  virtual void update() = 0;
  virtual bool shouldLog() = 0;
  virtual ~LogPolicy() = default;
 protected:
  explicit LogPolicy(int max) : max_(max) {}
  int counter_{0};
  int max_{0};
};

class OccasionPolicy : public LogPolicy {
 public:
  explicit OccasionPolicy(int max) : LogPolicy(max) {
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

  inline bool shouldLog() override {
    return should_log_;
  }

 private:
  bool should_log_{false};
};

class FirstNOccurrencesPolicy : public LogPolicy {
 public:
  explicit FirstNOccurrencesPolicy(int max) : LogPolicy(max) {}
  inline void update() override {
    if (!is_n_occurences_reached) {
      counter_++;
    }

    if (counter_ > max_) {
      is_n_occurences_reached = true;
    }
  }

  inline bool shouldLog() override {
    return !is_n_occurences_reached;
  }

 private:
  bool is_n_occurences_reached = false;
};

enum PolicyType {
  FIRST_N,
  EVERY_N
};

class LogPolicyFactory {
 public:
  static LogPolicy *create(PolicyType policy_type, int max) {
    switch (policy_type) {
      case FIRST_N: return new FirstNOccurrencesPolicy(max);
      case EVERY_N: return new OccasionPolicy(max);
      default:abort();
    }
  }
};

struct LogStatementData {
  explicit LogStatementData(LogPolicy *log_policy) : log_policy_(log_policy) {}
  LogPolicy *log_policy_;
  std::string msg{};
  std::string severity_str;
};

class InternalLogCount {
 public:
  static InternalLogCount &getInstance() {
    static InternalLogCount instance;
    return instance;
  }

  inline void update(const std::string &key,
                     int max,
                     const InternalLog &internal_log,
                     const std::string &severity_str,
                     PolicyType policy_type) {
    update(key, max, internal_log.ss.str(), severity_str, policy_type);
  }

  inline void update(const std::string &key,
                     int max,
                     const std::string &log_msg,
                     const std::string &severity_str,
                     PolicyType policy_type) {
    if (!keyExists(key)) {
      LogStatementData data(LogPolicyFactory::create(policy_type, max));

      data.msg = log_msg;
      data.severity_str = severity_str;
      occurences_.insert({key, data});
    }
    process(key);
  }

 private:
  inline bool keyExists(const std::string &key) {
    return occurences_.find(key) != occurences_.end();
  }

  inline void process(const std::string &key) {
    LogStatementData *data = &occurences_.at(key);
    data->log_policy_->update();
    if (data->log_policy_->shouldLog()) {
      InternalLog(data->severity_str) << data->msg;
    }
  }

  InternalLogCount() = default;
  std::unordered_map<std::string, LogStatementData> occurences_{};
};

class InternalPolicyLog : public InternalLog {
 public:
  InternalPolicyLog(std::string key, int n, std::string severity, PolicyType policy_type) :
      severity_(std::move(severity)), key_(std::move(key)), n_(n), policy_type_(policy_type) {};

  ~InternalPolicyLog() {
    InternalLogCount::getInstance().update(key_, n_, ss.str(), severity_, policy_type_);
  }
 private:
  std::string severity_{};
  std::string key_{};
  int n_{};
  PolicyType policy_type_{};
};
#define LPP_GET_KEY() std::string(__FILE__) + std::to_string(__LINE__)

#endif //LOG__LOG_H_
