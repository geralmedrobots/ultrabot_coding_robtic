#include "odometry_publisher.hpp"

#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
constexpr double kVarPositionBase = 0.001;
constexpr double kVarYawBase = 0.03;
constexpr double kVarUnused = 1'000'000.0;
}

OdometryPublisher::OdometryPublisher(rclcpp_lifecycle::LifecycleNode & node)
: node_(node)
{
}

void OdometryPublisher::configure(const Config & config)
{
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
  calculator_ = std::make_unique<OdometryCalculator>(
    config.distance_wheels,
    config.wheel_diameter,
    config.left_wheel_polarity,
    config.right_wheel_polarity);
  odom_pub_ = node_.create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(50));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_.shared_from_this());
  prev_time_sec_ = node_.now().seconds();
  configured_ = true;
}

void OdometryPublisher::activate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (odom_pub_) {
    odom_pub_->on_activate();
  }
}

void OdometryPublisher::deactivate()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (odom_pub_ && odom_pub_->is_activated()) {
    odom_pub_->on_deactivate();
  }
}

void OdometryPublisher::cleanup()
{
  std::lock_guard<std::mutex> lock(mutex_);
  tf_broadcaster_.reset();
  odom_pub_.reset();
  calculator_.reset();
  configured_ = false;
  prev_time_sec_ = 0.0;
}

void OdometryPublisher::updateFromFeedback(int left_mrpm, int right_mrpm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_ || !calculator_) {
    return;
  }

  const double now_sec = node_.now().seconds();
  double delta_t = now_sec - prev_time_sec_;
  if (delta_t <= 0.0) {
    prev_time_sec_ = now_sec;
    return;
  }

  if (delta_t > config_.delta_t_warn_threshold) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,
      "Large odometry delta_t %.3f s", delta_t);
  }

  prev_time_sec_ = now_sec;

  if (!calculator_->update(left_mrpm, right_mrpm, delta_t)) {
    return;
  }

  double x, y, yaw;
  calculator_->getPose(x, y, yaw);

  double linear_vel, angular_vel;
  calculator_->getTwist(linear_vel, angular_vel);

  double var_position, var_yaw;
  calculator_->getCovariance(var_position, var_yaw);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = node_.now();
  odom.header.frame_id = config_.odom_frame_id;
  odom.child_frame_id = config_.child_frame_id;

  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  odom.pose.pose.orientation = tf2::toMsg(q);

  odom.pose.covariance[0] = var_position;
  odom.pose.covariance[7] = var_position;
  odom.pose.covariance[14] = kVarUnused;
  odom.pose.covariance[21] = kVarUnused;
  odom.pose.covariance[28] = kVarUnused;
  odom.pose.covariance[35] = var_yaw;

  odom.twist.twist.linear.x = linear_vel;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.linear.z = 0.0;
  odom.twist.twist.angular.x = 0.0;
  odom.twist.twist.angular.y = 0.0;
  odom.twist.twist.angular.z = angular_vel;

  odom.twist.covariance[0] = kVarPositionBase;
  odom.twist.covariance[7] = kVarPositionBase;
  odom.twist.covariance[14] = kVarUnused;
  odom.twist.covariance[21] = kVarUnused;
  odom.twist.covariance[28] = kVarUnused;
  odom.twist.covariance[35] = kVarYawBase;

  if (odom_pub_ && odom_pub_->is_activated()) {
    odom_pub_->publish(odom);
  }

  if (config_.publish_tf && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom.header;
    tf_msg.child_frame_id = odom.child_frame_id;
    tf_msg.transform.translation.x = odom.pose.pose.position.x;
    tf_msg.transform.translation.y = odom.pose.pose.position.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  }
}
