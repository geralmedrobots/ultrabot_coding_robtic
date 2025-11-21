#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "command_limiter.hpp"
#include "command_watchdog.hpp"

using namespace std::chrono_literals;

class RclcppGuard : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

::testing::Environment * const rclcpp_env = ::testing::AddGlobalTestEnvironment(new RclcppGuard);

TEST(CommandLimiterTest, AppliesClampingAndWheelPolarities)
{
  CommandLimiter limiter(0.6, 0.2, 1.0, 1.5, 1, -1);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 2.0;   // Above max_linear -> should clamp to 1.0 m/s
  cmd.angular.z = 3.0;  // Above max_angular -> should clamp to 1.5 rad/s

  const auto [left_mrpm, right_mrpm] = limiter.computeWheelSpeeds(cmd);

  EXPECT_LE(std::abs(left_mrpm), CommandLimiter::kMaxCommandMrpm);
  EXPECT_LE(std::abs(right_mrpm), CommandLimiter::kMaxCommandMrpm);
  EXPECT_LT(left_mrpm, 0) << "Left wheel should run backwards with positive angular velocity";
  EXPECT_GT(right_mrpm, 0) << "Right wheel should run forwards with positive angular velocity";
}

TEST(CommandLimiterTest, ReturnsZeroWhenWheelDiameterInvalid)
{
  CommandLimiter limiter;
  limiter.setParameters(0.5, 0.0, 1.0, 1.0, 1, 1);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.5;
  cmd.angular.z = 0.2;

  const auto speeds = limiter.computeWheelSpeeds(cmd);
  EXPECT_EQ(0, speeds.first);
  EXPECT_EQ(0, speeds.second);
}

TEST(CommandWatchdogTest, DetectsTimeouts)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  CommandWatchdog watchdog(clock);
  watchdog.setTimeout(0.05);

  watchdog.recordCommand(100, 100);
  std::this_thread::sleep_for(80ms);

  EXPECT_TRUE(watchdog.timedOut());
  EXPECT_EQ(std::make_pair(100, 100), watchdog.lastCommand());
}

TEST(CommandWatchdogTest, DoesNotTimeoutBeforeFirstCommand)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  CommandWatchdog watchdog(clock);
  watchdog.setTimeout(0.1);

  std::this_thread::sleep_for(50ms);

  EXPECT_FALSE(watchdog.timedOut());
}

TEST(CommandWatchdogTest, UpdatesTimestampOnNewCommand)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
  CommandWatchdog watchdog(clock);
  watchdog.setTimeout(10.0);

  watchdog.recordCommand(0, 0);
  EXPECT_LT(watchdog.timeSinceLastCommand(), 0.01);

  std::this_thread::sleep_for(10ms);
  watchdog.recordCommand(50, -50);
  EXPECT_LT(watchdog.timeSinceLastCommand(), 0.01);
  EXPECT_EQ(std::make_pair(50, -50), watchdog.lastCommand());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
