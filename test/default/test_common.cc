#include <gtest/gtest.h>
#include <log++.h>
#include <test_utils.h>

void aborting() {
  abort();
}

void not_aborting() {

}

TEST(common, testCheckAbort) {
  ASSERT_TRUE(checkAbort(aborting));
  ASSERT_FALSE(checkAbort(not_aborting));
}

TEST(common, fatal_test) {
  LOG_INIT(*test_argv);
  ASSERT_TRUE(checkAbort([]() { LPP_CAPTURE_STDERR(LOG(FATAL) << "Test"); }));
}

TEST(common, testRemoveNumbersFromString) {
  ASSERT_EQ("Test", removeNumbersFromString("Test123"));
  ASSERT_EQ("Test", removeNumbersFromString("Te123st123"));
  ASSERT_EQ("Test", removeNumbersFromString("123Te123st123"));
}
