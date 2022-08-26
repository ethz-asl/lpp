//
// Created by acey on 23.08.22.
//

#include <gtest/gtest.h>
#include "test_utils.h"

char** test_argv;

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  test_argv = argv;
  return RUN_ALL_TESTS();
}