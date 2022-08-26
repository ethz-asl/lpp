//
// Created by acey on 23.08.22.
//

#ifndef LOG_TEST_TEST_UTILS_H_
#define LOG_TEST_TEST_UTILS_H_

#include <string>

extern char** test_argv;

inline static bool isSubstring(const std::string& string, const std::string& substring) {
  return (string.find(substring) != std::string::npos);
}



#endif //LOG_TEST_TEST_UTILS_H_
