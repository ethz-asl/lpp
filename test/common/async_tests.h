//
// Created by 4c3y (acey) on 21.09.22.
//

#ifndef LOG_TEST_COMMON_ASYNC_TESTS_H_
#define LOG_TEST_COMMON_ASYNC_TESTS_H_

#include <thread>
#include <test_utils.h>

#define GET_CLASS_NAME(class_ptr, status) abi::__cxa_demangle(typeid(class_ptr).name(), nullptr, nullptr, status)

enum CompareType {
  EQUAL,
  IS_SUBSTRING
};

enum StreamType {
  STDOUT,
  STDERR
};

struct AsyncTest {
  std::string class_name;
  std::string expected_output;
  std::function<void()> fn;
  CompareType compare_type;
  StreamType stream_type;
};

extern std::vector<AsyncTest> generateTests();
static std::vector<AsyncTest> tests = generateTests();

class TestResult {
 public:
  static inline TestResult &getInstance() {
    static TestResult instance{};
    return instance;
  }

  /**
   * Returns test result by test name
   * @param test_name
   * @return true on success otherwise false
   */
  inline bool get(const std::string &test_name) {
    std::scoped_lock<std::mutex> lock(test_result_mutex_);
    LOG_INIT(*test_argv);
    if (!started_) {
      started_ = true;
      startAll();
    }

    if (test_results.find(test_name) == test_results.end()) {
      return false;
    }

    bool res = test_results.at(test_name);
    return res;
  }

 private:
  static inline void startAll() {
    std::vector<std::thread> thread_pool{};
    for (const auto &t: tests) {
      thread_pool.emplace_back(&TestResult::startTimed, &TestResult::getInstance(), t);
    }

    for (auto &t: thread_pool) {
      t.join();
    }
  }

  static inline void startCapture(StreamType stream_type) {
    switch (stream_type) {
      case STDOUT: testing::internal::CaptureStdout();
        return;
      case STDERR: testing::internal::CaptureStderr();
        return;
    }
  }

  static inline std::string stopCapture(StreamType stream_type) {
    switch (stream_type) {
      case STDOUT: return testing::internal::GetCapturedStdout();

      case STDERR: return testing::internal::GetCapturedStderr();
    }
    abort();
  }

  inline void startTimed(const AsyncTest &a) {
    bool test_status = true;

    for (int i = 0; i < 30; i++) {
      stdout_capture_mutex_.lock();
      startCapture(a.stream_type);
      a.fn();
      std::string output = stopCapture(a.stream_type);
      stdout_capture_mutex_.unlock();

      if ((i == 0 || i % 4 == 0)) {
        if (!compare(a.compare_type, output, a.expected_output)) {
          std::cout << a.class_name << " failed. Output: " << output << " expected output: " << a.expected_output
                    << std::endl;
          test_status = false;
        }
      } else if (!compare(a.compare_type, output, "")) {
        std::cout << a.class_name << " failed. Output: " << output << " expected output: " << "\"\"" << std::endl;
        test_status = false;
      }
      usleep(250000.);
    }
    insert(a.class_name, test_status);
  }

  //gtest assertions only work on main thread
  static inline bool compare(CompareType compare_type, const std::string &output, const std::string &expected_output) {
    switch (compare_type) {
      case EQUAL:return compareEquality(output, expected_output);
      case IS_SUBSTRING: return compareSubstring(output, expected_output);
      default:return false;
    }
  }

  static inline bool compareSubstring(const std::string &output, const std::string &expected_output) {
    if (expected_output.empty()) {
      return output.empty();
    }
    return isSubstring(output, expected_output);
  };

  static inline bool compareEquality(const std::string &output, const std::string &expected_output) {
    return output == expected_output;
  };

  inline void insert(const std::string &test_name, bool test_status) {
    test_results.insert({test_name, test_status});
  }

  TestResult() = default;

  bool started_{false};
  std::mutex test_result_mutex_;
  std::mutex stdout_capture_mutex_;
  std::map<std::string, bool> test_results;
};

#endif //LOG_TEST_COMMON_ASYNC_TESTS_H_
