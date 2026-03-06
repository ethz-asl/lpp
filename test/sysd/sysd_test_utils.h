#ifndef LOG_TEST_SYSD_TEST_UTILS_H_
#define LOG_TEST_SYSD_TEST_UTILS_H_

#include <log++.h>
#include <string>
#include <vector>

namespace lpp {
namespace sysdtest {

struct Entry {
  BaseSeverity severity;
  std::string message;
  std::string identifier;
};

inline std::vector<Entry> &entries() {
  static std::vector<Entry> captured_entries;
  return captured_entries;
}

inline void clear() {
  entries().clear();
}

inline void sender(BaseSeverity severity, const std::string &message, const std::string &identifier) {
  entries().push_back({severity, message, identifier});
}

inline void init(char *argv) {
  clear();
  LOG_INIT(argv, nullptr, sender);
}

} // namespace sysdtest
} // namespace lpp

#endif // LOG_TEST_SYSD_TEST_UTILS_H_
