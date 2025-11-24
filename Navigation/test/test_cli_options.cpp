#include "cli_options.hpp"

#include <gtest/gtest.h>
#include <cstdlib>

namespace
{
constexpr char kExecutable[] = "somanet_main";
}

TEST(CliOptionsTest, DefaultsWithoutEnv)
{
  char arg0[] = kExecutable;
  char * argv[] = {arg0};
  int argc = 1;

  std::vector<char *> filtered;
  auto options = somanet::parse_cli_arguments(argc, argv, filtered);

  EXPECT_FALSE(options.autostart);
  ASSERT_EQ(filtered.size(), 2u);  // argv[0] + terminating nullptr
  EXPECT_STREQ(filtered.front(), kExecutable);
  EXPECT_EQ(filtered.back(), nullptr);
}

TEST(CliOptionsTest, EnablesAutostartFlag)
{
  char arg0[] = kExecutable;
  char flag[] = "--autostart";
  char * argv[] = {arg0, flag};
  int argc = 2;

  std::vector<char *> filtered;
  auto options = somanet::parse_cli_arguments(argc, argv, filtered);

  EXPECT_TRUE(options.autostart);
  ASSERT_EQ(filtered.size(), 2u);
  EXPECT_STREQ(filtered.front(), kExecutable);
  EXPECT_EQ(filtered.back(), nullptr);
}

TEST(CliOptionsTest, RejectsConflictingFlags)
{
  char arg0[] = kExecutable;
  char flag1[] = "--autostart";
  char flag2[] = "--no-autostart";
  char * argv[] = {arg0, flag1, flag2};
  int argc = 3;

  std::vector<char *> filtered;
  EXPECT_THROW(somanet::parse_cli_arguments(argc, argv, filtered), std::invalid_argument);
}

TEST(CliOptionsTest, UsesEnvironmentFallback)
{
  char arg0[] = kExecutable;
  char * argv[] = {arg0};
  int argc = 1;

  ASSERT_EQ(setenv("ULTRABOT_AUTOSTART", "true", 1), 0);

  std::vector<char *> filtered;
  auto options = somanet::parse_cli_arguments(argc, argv, filtered);

  EXPECT_TRUE(options.autostart);

  unsetenv("ULTRABOT_AUTOSTART");
}

