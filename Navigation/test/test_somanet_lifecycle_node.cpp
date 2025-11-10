#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

#include "mock_driver.hpp"
#include "somanet_lifecycle_node.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SomanetLifecycleFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  }

  void TearDown() override
  {
    executor_->cancel();
    executor_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<SomanetLifecycleNode> makeNode(
    std::shared_ptr<MockDriver> driver,
    const std::map<std::string, rclcpp::ParameterValue> & overrides = {})
  {
    rclcpp::NodeOptions options;
    for (const auto & kv : overrides) {
      options.append_parameter_override(kv.first, kv.second);
    }
    auto node = std::make_shared<SomanetLifecycleNode>(driver, options);
    executor_->add_node(node->get_node_base_interface());
    return node;
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(SomanetLifecycleFixture, ConfigureFailsWithoutInterface)
{
  auto driver = std::make_shared<MockDriver>();
  auto node = makeNode(driver);
  EXPECT_EQ(node->configure(), CallbackReturn::FAILURE);
}

TEST_F(SomanetLifecycleFixture, CommandConversionProducesMrpm)
{
  auto driver = std::make_shared<MockDriver>();
  auto node = makeNode(driver, {
    {"ethercat_interface", rclcpp::ParameterValue("mock0")}
  });

  ASSERT_EQ(node->configure(), CallbackReturn::SUCCESS);
  ASSERT_EQ(node->activate(), CallbackReturn::SUCCESS);

  auto helper = std::make_shared<rclcpp::Node>("test_helper");
  auto pub = helper->create_publisher<geometry_msgs::msg::Twist>("wheel_cmd_safe", 10);
  executor_->add_node(helper);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.4;  // m/s
  cmd.angular.z = 0.0;
  pub->publish(cmd);
  executor_->spin_some();

  const auto last_cmd = driver->getLastCommandRequest();
  EXPECT_NEAR(last_cmd.first, last_cmd.second, 1);
  EXPECT_GT(std::abs(last_cmd.first), 0);

  node->deactivate();
  node->cleanup();
}

TEST_F(SomanetLifecycleFixture, WatchdogStopsMotorsAfterTimeout)
{
  auto driver = std::make_shared<MockDriver>();
  auto node = makeNode(driver, {
    {"ethercat_interface", rclcpp::ParameterValue("mock0")},
    {"cmd_watchdog_timeout", rclcpp::ParameterValue(0.05)}
  });

  ASSERT_EQ(node->configure(), CallbackReturn::SUCCESS);
  ASSERT_EQ(node->activate(), CallbackReturn::SUCCESS);

  auto helper = std::make_shared<rclcpp::Node>("watchdog_helper");
  auto pub = helper->create_publisher<geometry_msgs::msg::Twist>("wheel_cmd_safe", 10);
  executor_->add_node(helper);

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.2;
  pub->publish(cmd);
  executor_->spin_some();

  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  executor_->spin_some();

  const auto last_cmd = driver->getLastCommandRequest();
  EXPECT_EQ(last_cmd.first, 0);
  EXPECT_EQ(last_cmd.second, 0);

  node->deactivate();
  node->cleanup();
}

TEST_F(SomanetLifecycleFixture, OdometryPublishesOnFeedback)
{
  auto driver = std::make_shared<MockDriver>();
  auto node = makeNode(driver, {
    {"ethercat_interface", rclcpp::ParameterValue("mock0")}
  });

  ASSERT_EQ(node->configure(), CallbackReturn::SUCCESS);
  ASSERT_EQ(node->activate(), CallbackReturn::SUCCESS);

  auto listener = std::make_shared<rclcpp::Node>("odom_listener");
  std::atomic<int> message_count{0};
  auto sub = listener->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    [&](const nav_msgs::msg::Odometry::SharedPtr &) {
      message_count++;
    });
  (void)sub;
  executor_->add_node(listener);

  driver->setVelocity(1000, 1000);
  for (int i = 0; i < 5; ++i) {
    driver->simulateCycle();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    executor_->spin_some();
  }

  EXPECT_GT(message_count.load(), 0);

  node->deactivate();
  node->cleanup();
}

// Standalone test main
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
