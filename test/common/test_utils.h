//
// Created by acey on 23.08.22.
//

#ifndef LOG_TEST_TEST_UTILS_H_
#define LOG_TEST_TEST_UTILS_H_

#include <string>
#include <gtest/gtest.h>

extern char **test_argv;

inline static bool isSubstring(const std::string &string, const std::string &substring) {
  return (string.find(substring) != std::string::npos);
}

inline static std::string removeNumbersFromString(std::string str) {
  int current = 0;
  for (int i = 0; i < str.length(); i++) {
    if (!isdigit(str[i])) {
      str[current] = str[i];
      current++;
    }
  }
  return str.substr(0, current);;
}

//! Macros to capture stdout and stderr and assign output directly to std::string or suppress output
#define LPP_CAPTURE_STDOUT(x) []() {testing::internal::CaptureStdout(); x; \
return testing::internal::GetCapturedStdout();}()

#define LPP_CAPTURE_STDERR(x) []() {testing::internal::CaptureStderr(); x; \
return testing::internal::GetCapturedStderr();}()

inline int ForkAndReap(int *Ws, const std::function<void()> &fn) {
  pid_t pid = fork();
  if (fork() < 0) {
    return -1;
  }

  if (pid == 0) {
    fn();
    _exit(0);
  }

  while (true) {
    if (waitpid(pid, Ws, WUNTRACED) < 0) {
      if (EINTR == errno) {
        continue;
      }

      abort();
    } else if (!WIFEXITED(*Ws) && !WIFSIGNALED(*Ws)) { //shouldn't have stopped
      if (kill(pid, SIGTERM) < 0 || kill(pid, SIGCONT) < 0) {
        abort();
      }
    }
    break;
  }
  return 0;
}

/**
 * Checks if function calls abort;
 * @param fn
 * @return true if function called abort(), otherwise false
 */

inline bool checkAbort(const std::function<void()> &fn) {
  int ws;
  return ForkAndReap(&ws, fn) >= 0 && WIFSIGNALED(ws) && WTERMSIG(SIGABRT);
}

//! Ros testing utils
inline static constexpr const char *ERROR_MESSAGE = "Base angle (%f) is less than the minimum angle (%f)";


namespace lpp {
namespace rostest{
inline static const std::string debug = "\x1B[m[DEBUG] [.]: Test\x1B[m\n";
inline static const std::string info = "\x1B[m[ INFO] [.]: Test\x1B[m\n";
inline static const std::string warning = "\x1B[m[ WARN] [.]: Test\x1B[m\n";
inline static const std::string error = "\x1B[m[ERROR] [.]: Test\x1B[m\n";
inline static const std::string fatal = "\x1B[m[FATAL] [.]: Test\x1B[m\n";
}


namespace rosprintf {
inline static const std::string debug = "\x1B[m[DEBUG] [.]: Base angle (.) is less than the minimum angle (.)\x1B[m\n";
inline static const std::string info = "\x1B[m[ INFO] [.]: Base angle (.) is less than the minimum angle (.)\x1B[m\n";
inline static const std::string warning = "\x1B[m[ WARN] [.]: Base angle (.) is less than the minimum angle (.)\x1B[m\n";
inline static const std::string error = "\x1B[m[ERROR] [.]: Base angle (.) is less than the minimum angle (.)\x1B[m\n";
inline static const std::string fatal = "\x1B[m[FATAL] [.]: Base angle (.) is less than the minimum angle (.)\x1B[m\n";
}
}


#endif //LOG_TEST_TEST_UTILS_H_
