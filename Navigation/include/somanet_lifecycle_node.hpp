#ifndef SOMANET_LIFECYCLE_NODE_HPP
#define SOMANET_LIFECYCLE_NODE_HPP

#include <atomic>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include "drive_interface.hpp"
#include "command_limiter.hpp"
#include "command_watchdog.hpp"
#include "odometry_publisher.hpp"

/**
 * @brief Lifecycle-managed SOMANET driver node
 *
 * The node orchestrates the hardware-specific DriveInterface implementation,
 * performs parameter validation, exposes a watchdog for command freshness and
 * publishes fused odometry data.  It is purposely decoupled from the EtherCAT
 * transport so the class can be exercised with the MockDriver in unit tests.
 */
class SomanetLifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit SomanetLifecycleNode(
    std::shared_ptr<DriveInterface> drive,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
  // Parameter helpers
  void declareAndFetchParameters();
  bool validateParameters();

  // ROS callbacks
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void handleDriveFeedback(int left_mrpm, int right_mrpm);
  void watchdogTick();

  // Internal helpers
  void publishFault(const std::string & reason);
  void publishRecovery(const std::string & context = {});

  // Dependencies
  std::shared_ptr<DriveInterface> drive_;
  CommandLimiter command_limiter_;
  std::unique_ptr<CommandWatchdog> command_watchdog_;
  std::unique_ptr<OdometryPublisher> odom_publisher_;

  // ROS entities
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr fault_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  // Parameters
  double distance_wheels_{0.5};
  double wheel_diameter_{0.17};
  double max_linear_vel_{1.0};
  double max_angular_vel_{1.0};
  double cmd_watchdog_timeout_{0.5};
  double delta_t_warn_threshold_{0.1};
  std::string odom_frame_id_{"odom"};
  std::string child_frame_id_{"base_link"};
  std::string interface_name_;
  bool publish_tf_{true};
  int left_wheel_polarity_{1};
  int right_wheel_polarity_{1};

  // State
  std::atomic<int> commanded_left_mrpm_{0};
  std::atomic<int> commanded_right_mrpm_{0};
  std::atomic<bool> fault_active_{false};
};

#endif  // SOMANET_LIFECYCLE_NODE_HPP
