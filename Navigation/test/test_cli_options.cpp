#include "cli_options.hpp"

#include <gtest/gtest.h>
#include <cstdlib>
#include <optional>
#include <string>

namespace
{
constexpr char kExecutable[] = "somanet_main";
}

class EnvGuard
{
public:
  explicit EnvGuard(const std::string & value)
  {
    const char * existing = std::getenv("ULTRABOT_AUTOSTART");
    if (existing) {
      previous_ = std::string(existing);
    }

    setenv("ULTRABOT_AUTOSTART", value.c_str(), 1);
  }

  EnvGuard(const EnvGuard &) = delete;
  EnvGuard & operator=(const EnvGuard &) = delete;

  ~EnvGuard()
  {
    if (previous_) {
      setenv("ULTRABOT_AUTOSTART", previous_->c_str(), 1);
    } else {
      unsetenv("ULTRABOT_AUTOSTART");
    }
  }

private:
  std::optional<std::string> previous_;
};

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

TEST(CliOptionsTest, ThrowsOnNullArgvWhenArgcPositive)
{
  int argc = 1;
  char ** argv = nullptr;

  std::vector<char *> filtered;
  EXPECT_THROW(somanet::parse_cli_arguments(argc, argv, filtered), std::invalid_argument);
}

TEST(CliOptionsTest, UsesEnvironmentFallback)
{
  char arg0[] = kExecutable;
  char * argv[] = {arg0};
  int argc = 1;

  EnvGuard guard{"true"};

  std::vector<char *> filtered;
  auto options = somanet::parse_cli_arguments(argc, argv, filtered);

  EXPECT_TRUE(options.autostart);
}

TEST(CliOptionsTest, UsesEnvironmentFallbackWithWhitespaceAndCase)
{
  char arg0[] = kExecutable;
  char * argv[] = {arg0};
  int argc = 1;

  EnvGuard guard{"  YeS  "};

  std::vector<char *> filtered;
  auto options = somanet::parse_cli_arguments(argc, argv, filtered);

  EXPECT_TRUE(options.autostart);
}

TEST(CliOptionsTest, EnvironmentFallbackDefaultsToFalseOnUnknown)
{
  char arg0[] = kExecutable;
  char * argv[] = {arg0};
  int argc = 1;

  EnvGuard guard{"maybe"};

  std::vector<char *> filtered;
  auto options = somanet::parse_cli_arguments(argc, argv, filtered);

  EXPECT_FALSE(options.autostart);
}

TEST(CliOptionsTest, CommandLineAutostartOverridesEnvironmentFalse)
{
  char arg0[] = kExecutable;
  char flag[] = "--autostart";
  char * argv[] = {arg0, flag};
  int argc = 2;

  EnvGuard guard{"0"};

  std::vector<char *> filtered;
  auto options = somanet::parse_cli_arguments(argc, argv, filtered);

  EXPECT_TRUE(options.autostart);
}

TEST(CliOptionsTest, CommandLineNoAutostartOverridesEnvironmentTrue)
{
  char arg0[] = kExecutable;
  char flag[] = "--no-autostart";
  char * argv[] = {arg0, flag};
  int argc = 2;

  EnvGuard guard{"true"};

  std::vector<char *> filtered;
  auto options = somanet::parse_cli_arguments(argc, argv, filtered);

  EXPECT_FALSE(options.autostart);
}

TEST(CliOptionsTest, FiltersThroughUnknownArguments)
{
  char arg0[] = kExecutable;
  char ros_flag[] = "--ros-args";
  char param[] = "-p";
  char setting[] = "foo:=bar";
  char * argv[] = {arg0, ros_flag, param, setting};
  int argc = 4;

  std::vector<char *> filtered;
  auto options = somanet::parse_cli_arguments(argc, argv, filtered);

  EXPECT_FALSE(options.autostart);
  ASSERT_EQ(filtered.size(), 5u);
  EXPECT_STREQ(filtered[0], kExecutable);
  EXPECT_STREQ(filtered[1], ros_flag);
  EXPECT_STREQ(filtered[2], param);
  EXPECT_STREQ(filtered[3], setting);
  EXPECT_EQ(filtered[4], nullptr);
}

