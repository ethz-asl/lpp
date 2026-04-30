#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdint>
#include <thread>
#include <unistd.h>

#include "sysd_test_utils.h"

using lpp::sysdtest::entries;

namespace {

inline void resetSysdInitState() {
  lpp::sysdtest::clear();
  lpp::internal::lppInit.lpp_initialized = false;
  lpp::internal::lppInit.glog_initialized = false;
  lpp::internal::lppInit.sysd_identifier.clear();
  lpp::internal::lppInit.sysd_sender = nullptr;
}

inline std::function<void(BaseSeverity, const std::string &, const std::string &)>
captureSender(std::vector<lpp::sysdtest::Entry> *store) {
  return [store](BaseSeverity severity, const std::string &message, const std::string &identifier) {
    store->push_back({severity, message, identifier});
  };
}

inline std::string readJournalForIdentifier(const std::string &identifier) {
  const std::string command = "journalctl --no-pager -o cat --since '-2 minutes' -t " + identifier + " 2>/dev/null";
  std::array<char, 256> buffer{};
  std::string output;

  FILE *pipe = popen(command.c_str(), "r");
  if (pipe == nullptr) {
    return output;
  }

  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    output += buffer.data();
  }
  (void) pclose(pipe);
  return output;
}

inline bool journalContains(const std::string &identifier, const std::string &message) {
  for (unsigned int attempt = 0; attempt < 20U; ++attempt) {
    if (readJournalForIdentifier(identifier).find(message) != std::string::npos) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}

inline bool journaldAcceptsDatagrams() {
  const int fd = socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
  if (fd < 0) {
    return false;
  }

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, lpp::internal::kJournalSocketPath, sizeof(addr.sun_path) - 1U);
  const auto addr_len = static_cast<socklen_t>(
      offsetof(sockaddr_un, sun_path) + std::strlen(lpp::internal::kJournalSocketPath) + 1U);
  const bool accepted = connect(fd, reinterpret_cast<sockaddr *>(&addr), addr_len) == 0;
  (void) close(fd);
  return accepted;
}

} // namespace

TEST(sysd_init, log_init_does_not_emit_log_messages) {
  resetSysdInitState();
  LOG_INIT(*test_argv, nullptr, lpp::sysdtest::sender);
  ASSERT_TRUE(entries().empty());
}

TEST(sysd_init, log_without_log_init_still_sends) {
  resetSysdInitState();
  lpp::internal::lppInit.sysd_sender = lpp::sysdtest::sender;

  LOG(I, "NoInit");
  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).severity, BaseSeverity::INFO);
  ASSERT_EQ(entries().at(0).message, "NoInit");
}

TEST(sysd_init, log_init_sets_identifier_for_sender) {
  resetSysdInitState();

  LOG_INIT(*test_argv, nullptr, lpp::sysdtest::sender);
  LOG(I, "Init");

  ASSERT_EQ(entries().size(), 1);
  ASSERT_EQ(entries().at(0).identifier, lpp::internal::filenameFromPath(*test_argv));
}

TEST(sysd_init, log_init_second_call_overrides_sender_and_identifier) {
  resetSysdInitState();

  std::vector<lpp::sysdtest::Entry> first_entries;
  std::vector<lpp::sysdtest::Entry> second_entries;

  char first_name[] = "/tmp/lpp-first";
  char second_name[] = "/tmp/lpp-second";

  LOG_INIT(first_name, nullptr, captureSender(&first_entries));
  LOG_INIT(second_name, nullptr, captureSender(&second_entries));

  LOG(W, "Stable");

  ASSERT_TRUE(first_entries.empty());
  ASSERT_EQ(second_entries.size(), 1);
  ASSERT_EQ(second_entries.at(0).identifier, "lpp-second");
  ASSERT_EQ(second_entries.at(0).severity, BaseSeverity::WARN);
  ASSERT_EQ(second_entries.at(0).message, "Stable");
}

TEST(sysd_init, logs_to_journald_without_injected_sender) {
  if (!journaldAcceptsDatagrams()) {
    GTEST_SKIP() << "journald socket does not accept datagrams in this environment";
  }

  resetSysdInitState();
  const std::string identifier = lpp::internal::filenameFromPath(*test_argv);
  const std::string message = "lpp_sysd_e2e_" + std::to_string(getpid());

  LOG_INIT(*test_argv);
  LOG(I, message);

  const bool found = journalContains(identifier, message);
  resetSysdInitState();
  ASSERT_TRUE(found);
}

TEST(sysd_init, journal_payload_uses_single_line_encoding_for_basic_message) {
  const std::string payload = lpp::internal::JournalSender::makePayload(BaseSeverity::WARN, "Basic", "lpp-test");

  ASSERT_NE(payload.find("MESSAGE=Basic\n"), std::string::npos);
  ASSERT_NE(payload.find("PRIORITY=" + std::to_string(lpp::internal::toSysdPriority(BaseSeverity::WARN)) + "\n"),
            std::string::npos);
  ASSERT_NE(payload.find("SYSLOG_IDENTIFIER=lpp-test\n"), std::string::npos);
}

TEST(sysd_init, journal_payload_uses_binary_encoding_for_multiline_message) {
  const std::string payload = lpp::internal::JournalSender::makePayload(BaseSeverity::INFO, "Line1\nLine2", "lpp-test");
  const std::string prefix = "MESSAGE\n";
  ASSERT_EQ(payload.substr(0, prefix.size()), prefix);

  std::uint64_t message_size = 0U;
  for (unsigned int i = 0; i < sizeof(message_size); ++i) {
    message_size |= static_cast<std::uint64_t>(static_cast<unsigned char>(payload.at(prefix.size() + i))) << (i * 8U);
  }

  ASSERT_EQ(message_size, std::string("Line1\nLine2").size());
  ASSERT_EQ(payload.substr(prefix.size() + sizeof(message_size), message_size), "Line1\nLine2");
  ASSERT_NE(payload.find("PRIORITY=" + std::to_string(lpp::internal::toSysdPriority(BaseSeverity::INFO)) + "\n"),
            std::string::npos);
  ASSERT_NE(payload.find("SYSLOG_IDENTIFIER=lpp-test\n"), std::string::npos);
}
