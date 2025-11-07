#ifndef ODOMETRY_PUBLISHER_HPP
#define ODOMETRY_PUBLISHER_HPP

#include <memory>
#include <mutex>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "odometry_calculator.hpp"

/**
 * @brief Encapsulates the odometry calculation and publication pipeline used
 *        by the SOMANET lifecycle node.
 */
class OdometryPublisher
{
public:
  struct Config
  {
    double distance_wheels{0.5};
    double wheel_diameter{0.17};
    int left_wheel_polarity{1};
    int right_wheel_polarity{1};
    double delta_t_warn_threshold{0.1};
    std::string odom_frame_id{"odom"};
    std::string child_frame_id{"base_link"};
    bool publish_tf{true};
  };

  explicit OdometryPublisher(rclcpp_lifecycle::LifecycleNode & node);

  void configure(const Config & config);

  void activate();
  void deactivate();
  void cleanup();

  void updateFromFeedback(int left_mrpm, int right_mrpm);

private:
  rclcpp_lifecycle::LifecycleNode & node_;
  Config config_{};
  std::unique_ptr<OdometryCalculator> calculator_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double prev_time_sec_{0.0};
  std::mutex mutex_;
  bool configured_{false};
};

#endif  // ODOMETRY_PUBLISHER_HPP
