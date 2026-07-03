#define MODE_NOLOG 1
#include <log++.h>
#undef MODE_NOLOG

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

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

class LppEmitter {
 public:
  LppEmitter(LogMode mode, nb::object identifier, nb::object callback, nb::object sysd_sender)
      : mode_(mode),
        identifier_(identifier.ptr() == Py_None ? "" : nb::cast<std::string>(identifier)),
        callback_(std::move(callback)),
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

  void emit(LppSeverity severity, const nb::object &message) {
    emitMessage(lpp::internal::toBase(severity), objectToString(message));
  }

 private:
  void emitMessage(BaseSeverity severity, const std::string &message) {
    if (mode_ == LogMode::MODE_NOLOG) {
      return;
    }

    if (mode_ == LogMode::MODE_SYSD) {
      emitSysdMessage(severity, message);
      return;
    }

    if (mode_ == LogMode::MODE_LPP && callback_.ptr() != Py_None) {
      callback_(toLppSeverity(severity), message);
      return;
    }

    std::cout << severityPrefix(severity) << message << std::endl;
  }

  void emitSysdMessage(BaseSeverity severity, const std::string &message) {
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
  nb::object sysd_sender_;
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

  nb::class_<lpp::python::LppEmitter>(m, "_LppEmitter")
      .def(nb::init<lpp::python::LogMode, nb::object, nb::object, nb::object>(),
           "mode"_a = lpp::python::LogMode::MODE_LPP,
           "identifier"_a = nb::none(),
           "callback"_a = nb::none(),
           "sysd_sender"_a = nb::none())
      .def("emit", &lpp::python::LppEmitter::emit, "severity"_a, "message"_a);
}
