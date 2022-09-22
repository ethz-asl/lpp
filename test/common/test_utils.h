//
// Created by acey on 23.08.22.
//

#ifndef LOG_TEST_TEST_UTILS_H_
#define LOG_TEST_TEST_UTILS_H_

#include <string>
#include <gtest/gtest.h>

extern char** test_argv;

inline static bool isSubstring(const std::string& string, const std::string& substring) {
  return (string.find(substring) != std::string::npos);
}

//! Macros to capture stdout and stderr and assign output directly to std::string
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
      if (kill(pid, SIGTERM) < 0 || kill(pid, SIGCONT) < 0)  {
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

#endif //LOG_TEST_TEST_UTILS_H_
