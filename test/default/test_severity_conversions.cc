//
// Created by 4c3y (acey) on 29.09.22.
//

#include <log++.h>
#include <gtest/gtest.h>


TEST(default_enum_conversions, LppToBase) {
  ASSERT_EQ(toBase(LppSeverity::D), BaseSeverity::DEBUG);
  ASSERT_EQ(toBase(LppSeverity::I), BaseSeverity::INFO);
  ASSERT_EQ(toBase(LppSeverity::W), BaseSeverity::WARN);
  ASSERT_EQ(toBase(LppSeverity::E), BaseSeverity::ERROR);
  ASSERT_EQ(toBase(LppSeverity::F), BaseSeverity::FATAL);
}

TEST(default_enum_conversions, GlogToBase) {
  ASSERT_EQ(toBase(GlogSeverity::DEBUG), BaseSeverity::DEBUG);
  ASSERT_EQ(toBase(GlogSeverity::INFO), BaseSeverity::INFO);
  ASSERT_EQ(toBase(GlogSeverity::WARNING), BaseSeverity::WARN);
  ASSERT_EQ(toBase(GlogSeverity::ERROR), BaseSeverity::ERROR);
  ASSERT_EQ(toBase(GlogSeverity::FATAL), BaseSeverity::FATAL);
}