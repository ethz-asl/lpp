#define MODE_NOLOG 1
#include <log++.h>
#undef MODE_NOLOG

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <unordered_map>

#if defined(__has_include)
#if __has_include(<sys/socket.h>) && __has_include(<sys/un.h>) && __has_include(<syslog.h>) && __has_include(<unistd.h>)
#include <sys/socket.h>
#include <sys/un.h>
#include <syslog.h>
#include <unistd.h>
#define LPP_PY_SYSD_SUPPORTED
#endif
#endif

namespace nb = nanobind;
using namespace nb::literals;

namespace lpp::python {
namespace {

using BaseSeverity = lpp::internal::BaseSeverity;
using LppSeverity = lpp::internal::LppSeverity;

enum class LogMode {
  MODE_LPP,
  MODE_SYSD,
  MODE_NOLOG,
  MODE_DEFAULT,
  MODE_GLOG,
  MODE_ROSLOG
};

std::string severityPrefix(BaseSeverity severity) {
  switch (severity) {
    case BaseSeverity::DEBUG:
      return "DEBUG ";
    case BaseSeverity::INFO:
      return "INFO  ";
    case BaseSeverity::WARN:
      return "WARN  ";
    case BaseSeverity::ERROR:
      return "ERROR ";
    case BaseSeverity::FATAL:
      return "FATAL ";
  }
  return "INFO  ";
}

LppSeverity toLppSeverity(BaseSeverity severity) {
  switch (severity) {
    case BaseSeverity::DEBUG:
      return LppSeverity::D;
    case BaseSeverity::INFO:
      return LppSeverity::I;
    case BaseSeverity::WARN:
      return LppSeverity::W;
    case BaseSeverity::ERROR:
      return LppSeverity::E;
    case BaseSeverity::FATAL:
      return LppSeverity::F;
  }
  return LppSeverity::I;
}

std::string objectToString(const nb::object &object) {
  return nb::cast<std::string>(nb::str(object));
}

std::string callerKey(const std::string &policy_name) {
  PyFrameObject *frame = PyEval_GetFrame();
  if (frame == nullptr) {
    return policy_name + ":<unknown>:0";
  }

  const int line = PyFrame_GetLineNumber(frame);
  PyObject *code = reinterpret_cast<PyObject *>(PyFrame_GetCode(frame));
  if (code == nullptr) {
    return policy_name + ":<unknown>:" + std::to_string(line);
  }

  PyObject *filename_obj = PyObject_GetAttrString(code, "co_filename");
  std::string filename = "<unknown>";
  if (filename_obj != nullptr) {
    const char *filename_chars = PyUnicode_AsUTF8(filename_obj);
    if (filename_chars != nullptr) {
      filename = filename_chars;
    }
    Py_DECREF(filename_obj);
  } else {
    PyErr_Clear();
  }

  Py_DECREF(code);
  return policy_name + ":" + filename + ":" + std::to_string(line);
}

std::string policyKey(const nb::object &key, const std::string &policy_name) {
  if (key.ptr() != Py_None) {
    return nb::cast<std::string>(key);
  }
  return callerKey(policy_name);
}

nb::object formatMessage(const std::string &format, const nb::args &args) {
  nb::str py_format(format.c_str());
  PyObject *formatted = PyNumber_Remainder(py_format.ptr(), args.ptr());
  if (formatted == nullptr) {
    throw nb::python_error();
  }
  return nb::steal(formatted);
}

#ifdef LPP_PY_SYSD_SUPPORTED
int toSysdPriority(BaseSeverity severity) {
  switch (severity) {
    case BaseSeverity::DEBUG:
      return LOG_DEBUG;
    case BaseSeverity::INFO:
      return LOG_INFO;
    case BaseSeverity::WARN:
      return LOG_WARNING;
    case BaseSeverity::ERROR:
      return LOG_ERR;
    case BaseSeverity::FATAL:
      return LOG_CRIT;
  }
  return LOG_INFO;
}

void appendLittleEndianUint64(std::string *payload, std::uint64_t value) {
  for (unsigned int i = 0; i < sizeof(value); ++i) {
    payload->push_back(static_cast<char>((value >> (i * 8U)) & 0xffU));
  }
}

void appendJournalField(std::string *payload, const std::string &field, const std::string &value) {
  if (value.find('\n') == std::string::npos) {
    payload->append(field);
    payload->push_back('=');
    payload->append(value);
    payload->push_back('\n');
    return;
  }

  payload->append(field);
  payload->push_back('\n');
  appendLittleEndianUint64(payload, static_cast<std::uint64_t>(value.size()));
  payload->append(value);
  payload->push_back('\n');
}

std::string makeJournalPayload(BaseSeverity severity, const std::string &message, const std::string &identifier) {
  std::string payload;
  appendJournalField(&payload, "MESSAGE", message);
  appendJournalField(&payload, "PRIORITY", std::to_string(toSysdPriority(severity)));
  if (!identifier.empty()) {
    appendJournalField(&payload, "SYSLOG_IDENTIFIER", identifier);
  }
  return payload;
}

void sendToJournal(BaseSeverity severity, const std::string &message, const std::string &identifier) {
  constexpr const char *kJournalSocketPath = "/run/systemd/journal/socket";

  const int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    return;
  }

  const std::string payload = makeJournalPayload(severity, message, identifier);
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, kJournalSocketPath, sizeof(addr.sun_path) - 1U);

  int flags = MSG_DONTWAIT;
#ifdef MSG_NOSIGNAL
  flags |= MSG_NOSIGNAL;
#endif

  const auto addr_len = static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + std::strlen(kJournalSocketPath) + 1U);
  (void) sendto(fd, payload.data(), payload.size(), flags, reinterpret_cast<sockaddr *>(&addr), addr_len);
  (void) close(fd);
}
#endif

}  // namespace

class Logger {
 public:
  Logger(LogMode mode, nb::object identifier, nb::object callback, int verbosity, nb::object sysd_sender)
      : mode_(mode),
        identifier_(identifier.ptr() == Py_None ? "" : nb::cast<std::string>(identifier)),
        callback_(std::move(callback)),
        verbosity_(verbosity),
        sysd_sender_(std::move(sysd_sender)) {
    if (mode_ == LogMode::MODE_GLOG) {
      throw std::runtime_error("MODE_GLOG is not available in the Python bindings");
    }
    if (mode_ == LogMode::MODE_ROSLOG) {
      throw std::runtime_error("MODE_ROSLOG is not available in the Python bindings");
    }
#ifndef LPP_PY_SYSD_SUPPORTED
    if (mode_ == LogMode::MODE_SYSD && sysd_sender_.ptr() == Py_None) {
      throw std::runtime_error("MODE_SYSD is not supported on this platform without a sysd_sender callback");
    }
#endif
  }

  void log(LppSeverity severity, const nb::object &message) {
    emit(lpp::internal::toBase(severity), objectToString(message));
  }

  void logIf(LppSeverity severity, bool condition, const nb::object &message) {
    if (condition) {
      log(severity, message);
    }
  }

  void logEvery(LppSeverity severity, unsigned int n, const nb::object &message, const nb::object &key) {
    validateCount(n);
    const std::string resolved_key = policyKey(key, "every");
    auto &counter = every_counters_[resolved_key];
    const bool should_log = counter % n == 0U;
    ++counter;
    if (should_log) {
      log(severity, message);
    }
  }

  void logFirst(LppSeverity severity, unsigned int n, const nb::object &message, const nb::object &key) {
    const std::string resolved_key = policyKey(key, "first");
    auto &counter = first_counters_[resolved_key];
    if (counter < n) {
      ++counter;
      log(severity, message);
    }
  }

  void logTimed(LppSeverity severity, float seconds, const nb::object &message, const nb::object &key) {
    if (seconds < 0.0F) {
      throw std::invalid_argument("seconds must be non-negative");
    }

    const std::string resolved_key = policyKey(key, "timed");
    const auto now = std::chrono::steady_clock::now();
    auto it = timed_last_log_.find(resolved_key);
    if (it == timed_last_log_.end() ||
        now >= it->second + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<float>(seconds))) {
      timed_last_log_[resolved_key] = now;
      log(severity, message);
    }
  }

  void logString(LppSeverity severity, const nb::object &message, const nb::object &sink) {
    const std::string string_message = objectToString(message);
    if (sink.ptr() != Py_None) {
      sink.attr("append")(string_message);
      return;
    }
    emit(lpp::internal::toBase(severity), string_message);
  }

  void logFormat(LppSeverity severity, const std::string &format, const nb::args &args) {
    log(severity, formatMessage(format, args));
  }

  void vlog(LppSeverity severity, int verbose_level, const nb::object &message) {
    if (verbosity_ >= verbose_level) {
      log(severity, message);
    }
  }

  void vlogIf(LppSeverity severity, int verbose_level, bool condition, const nb::object &message) {
    if (condition) {
      vlog(severity, verbose_level, message);
    }
  }

  void vlogEvery(LppSeverity severity, int verbose_level, unsigned int n, const nb::object &message, const nb::object &key) {
    if (verbosity_ >= verbose_level) {
      logEvery(severity, n, message, key);
    }
  }

  void vlogIfEvery(LppSeverity severity,
                   int verbose_level,
                   bool condition,
                   unsigned int n,
                   const nb::object &message,
                   const nb::object &key) {
    if (condition) {
      vlogEvery(severity, verbose_level, n, message, key);
    }
  }

 private:
  static void validateCount(unsigned int n) {
    if (n == 0U) {
      throw std::invalid_argument("n must be greater than zero");
    }
  }

  void emit(BaseSeverity severity, const std::string &message) {
    if (mode_ == LogMode::MODE_NOLOG) {
      return;
    }

    if (mode_ == LogMode::MODE_SYSD) {
      emitSysd(severity, message);
      return;
    }

    if (mode_ == LogMode::MODE_LPP && callback_.ptr() != Py_None) {
      callback_(toLppSeverity(severity), message);
      return;
    }

    std::cout << severityPrefix(severity) << message << std::endl;
  }

  void emitSysd(BaseSeverity severity, const std::string &message) {
#ifdef NDEBUG
    if (severity == BaseSeverity::DEBUG) {
      return;
    }
#endif
    if (sysd_sender_.ptr() != Py_None) {
      sysd_sender_(toLppSeverity(severity), message, identifier_);
      return;
    }
#ifdef LPP_PY_SYSD_SUPPORTED
    sendToJournal(severity, message, identifier_);
#endif
  }

  LogMode mode_;
  std::string identifier_;
  nb::object callback_;
  int verbosity_;
  nb::object sysd_sender_;
  std::unordered_map<std::string, unsigned int> every_counters_;
  std::unordered_map<std::string, unsigned int> first_counters_;
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> timed_last_log_;
};

}  // namespace lpp::python

NB_MODULE(_lpp, m) {
  m.doc() = "Log++ Python bindings.";

  nb::enum_<lpp::internal::LppSeverity>(m, "LppSeverity")
      .value("D", lpp::internal::LppSeverity::D)
      .value("I", lpp::internal::LppSeverity::I)
      .value("W", lpp::internal::LppSeverity::W)
      .value("E", lpp::internal::LppSeverity::E)
      .value("F", lpp::internal::LppSeverity::F);

  nb::enum_<lpp::python::LogMode>(m, "LogMode")
      .value("MODE_LPP", lpp::python::LogMode::MODE_LPP)
      .value("MODE_SYSD", lpp::python::LogMode::MODE_SYSD)
      .value("MODE_NOLOG", lpp::python::LogMode::MODE_NOLOG)
      .value("MODE_DEFAULT", lpp::python::LogMode::MODE_DEFAULT)
      .value("MODE_GLOG", lpp::python::LogMode::MODE_GLOG)
      .value("MODE_ROSLOG", lpp::python::LogMode::MODE_ROSLOG);

  nb::class_<lpp::python::Logger>(m, "Logger")
      .def(nb::init<lpp::python::LogMode, nb::object, nb::object, int, nb::object>(),
           "mode"_a = lpp::python::LogMode::MODE_LPP,
           "identifier"_a = nb::none(),
           "callback"_a = nb::none(),
           "verbosity"_a = 0,
           "sysd_sender"_a = nb::none())
      .def("log", &lpp::python::Logger::log, "severity"_a, "message"_a)
      .def("log_if", &lpp::python::Logger::logIf, "severity"_a, "condition"_a, "message"_a)
      .def("log_every", &lpp::python::Logger::logEvery, "severity"_a, "n"_a, "message"_a, "key"_a = nb::none())
      .def("log_first", &lpp::python::Logger::logFirst, "severity"_a, "n"_a, "message"_a, "key"_a = nb::none())
      .def("log_timed", &lpp::python::Logger::logTimed, "severity"_a, "seconds"_a, "message"_a, "key"_a = nb::none())
      .def("log_string", &lpp::python::Logger::logString, "severity"_a, "message"_a, "sink"_a = nb::none())
      .def("log_format",
           [](lpp::python::Logger &self,
              lpp::internal::LppSeverity severity,
              const std::string &format,
              const nb::args &args) { self.logFormat(severity, format, args); })
      .def("vlog", &lpp::python::Logger::vlog, "severity"_a, "verbose_level"_a, "message"_a)
      .def("vlog_if", &lpp::python::Logger::vlogIf, "severity"_a, "verbose_level"_a, "condition"_a, "message"_a)
      .def("vlog_every",
           &lpp::python::Logger::vlogEvery,
           "severity"_a,
           "verbose_level"_a,
           "n"_a,
           "message"_a,
           "key"_a = nb::none())
      .def("vlog_if_every",
           &lpp::python::Logger::vlogIfEvery,
           "severity"_a,
           "verbose_level"_a,
           "condition"_a,
           "n"_a,
           "message"_a,
           "key"_a = nb::none());
}
