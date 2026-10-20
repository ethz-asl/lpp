#include <gtest/gtest.h>
#include <log++.h>

#include <cstring>
#include <link.h>
#include <stdexcept>
#include <string>

TEST(NonGlogLinking, DoesNotLoadGlogOrLibunwind) {
  // A plain throw/catch can succeed even with the wrong unwinder. Check the
  // dependency that caused Python-loaded consumers to crash during exceptions.
  dl_iterate_phdr(
      [](dl_phdr_info *info, std::size_t, void *) {
        const char *basename = std::strrchr(info->dlpi_name, '/');
        const std::string library = basename ? basename + 1 : info->dlpi_name;
        EXPECT_NE(library.find("libglog"), 0U) << library;
        EXPECT_NE(library.find("libunwind"), 0U) << library;
        return 0;
      },
      nullptr);
}

TEST(NonGlogLinking, ExceptionsAreCatchable) {
  EXPECT_THROW(
      {
        LOG(INFO) << "Throwing a test exception";
        throw std::runtime_error("expected");
      },
      std::runtime_error);
}
