//
// Created by 4c3y (acey) on 24.11.22.
//

#ifndef LOG_TEST_LPP_CUSTOM_CALLBACK_H_
#define LOG_TEST_LPP_CUSTOM_CALLBACK_H_

#include <log++.h>

namespace lpp{
namespace custom{
inline void logCallback(BaseSeverity severity, const std::string& str) {

  std::string severity_str;
  switch (severity) {

    case BaseSeverity::DEBUG:severity_str="debug"; break;
    case BaseSeverity::INFO:severity_str="info"; break;
    case BaseSeverity::WARN:severity_str="warning"; break;
    case BaseSeverity::ERROR:severity_str="error"; break;
    case BaseSeverity::FATAL:severity_str="fatal"; break;
  }


  std::cout << "Log++ [" << severity_str << "] " << str << std::endl;
}
}
}

#endif //LOG_TEST_LPP_CUSTOM_CALLBACK_H_
