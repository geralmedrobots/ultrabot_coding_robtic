#include "somanet_lifecycle_node.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <lifecycle_msgs/msg/state.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double kVarPositionBase = 0.001;   // ≈ (3 cm)^2
constexpr double kVarYawBase = 0.03;         // ≈ (10°)^2 in rad
constexpr double kVarUnused = 1'000'000.0;   // Unused covariance entries
constexpr int kMaxCommandMrpm = 20000;       // Matches EtherCAT safety limit
constexpr double kPi = 3.14159265358979323846;
}  // namespace

SomanetLifecycleNode::SomanetLifecycleNode(
  std::shared_ptr<DriveInterface> drive,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("somanet_driver", options),
  drive_(std::move(drive))
{
  if (!drive_) {
    throw std::invalid_argument("DriveInterface instance must not be null");
  }
}

void SomanetLifecycleNode::declareAndFetchParameters()
{
  this->declare_parameter("distance_wheels", distance_wheels_);
  this->declare_parameter("wheel_diameter", wheel_diameter_);
  this->declare_parameter("max_linear_vel", max_linear_vel_);
  this->declare_parameter("max_angular_vel", max_angular_vel_);
  this->declare_parameter("cmd_watchdog_timeout", cmd_watchdog_timeout_);
  this->declare_parameter("delta_t_warn_threshold", delta_t_warn_threshold_);
  this->declare_parameter("odom_frame_id", odom_frame_id_);
  this->declare_parameter("child_frame_id", child_frame_id_);
  this->declare_parameter("publish_tf", publish_tf_);
  this->declare_parameter("left_wheel_polarity", left_wheel_polarity_);
  this->declare_parameter("right_wheel_polarity", right_wheel_polarity_);
  this->declare_parameter("ethercat_interface", interface_name_);

  this->get_parameter("distance_wheels", distance_wheels_);
  this->get_parameter("wheel_diameter", wheel_diameter_);
  this->get_parameter("max_linear_vel", max_linear_vel_);
  this->get_parameter("max_angular_vel", max_angular_vel_);
  this->get_parameter("cmd_watchdog_timeout", cmd_watchdog_timeout_);
  this->get_parameter("delta_t_warn_threshold", delta_t_warn_threshold_);
  this->get_parameter("odom_frame_id", odom_frame_id_);
  this->get_parameter("child_frame_id", child_frame_id_);
  this->get_parameter("publish_tf", publish_tf_);
  this->get_parameter("left_wheel_polarity", left_wheel_polarity_);
  this->get_parameter("right_wheel_polarity", right_wheel_polarity_);
  this->get_parameter("ethercat_interface", interface_name_);
}

bool SomanetLifecycleNode::validateParameters()
{
  if (distance_wheels_ <= 0.1 || distance_wheels_ > 2.0) {
    RCLCPP_FATAL(get_logger(),
      "distance_wheels out of safe range [0.1, 2.0]m (got %.3f)", distance_wheels_);
    return false;
  }

  if (wheel_diameter_ <= 0.05 || wheel_diameter_ > 0.5) {
    RCLCPP_FATAL(get_logger(),
      "wheel_diameter out of safe range [0.05, 0.5]m (got %.3f)", wheel_diameter_);
    return false;
  }

  if (max_linear_vel_ <= 0.01 || max_linear_vel_ > 5.0) {
    RCLCPP_FATAL(get_logger(),
      "max_linear_vel out of safe range [0.01, 5.0] m/s (got %.3f)", max_linear_vel_);
    return false;
  }

  if (max_angular_vel_ <= 0.01 || max_angular_vel_ > 10.0) {
    RCLCPP_FATAL(get_logger(),
      "max_angular_vel out of safe range [0.01, 10.0] rad/s (got %.3f)", max_angular_vel_);
    return false;
  }

  if (cmd_watchdog_timeout_ <= 0.01 || cmd_watchdog_timeout_ > 10.0) {
    RCLCPP_FATAL(get_logger(),
      "cmd_watchdog_timeout out of safe range [0.01, 10.0]s (got %.3f)", cmd_watchdog_timeout_);
    return false;
  }

  if (delta_t_warn_threshold_ <= 0.001 || delta_t_warn_threshold_ > 1.0) {
    RCLCPP_FATAL(get_logger(),
      "delta_t_warn_threshold out of safe range [0.001, 1.0]s (got %.3f)", delta_t_warn_threshold_);
    return false;
  }

  if (left_wheel_polarity_ != 1 && left_wheel_polarity_ != -1) {
    RCLCPP_WARN(get_logger(), "Invalid left_wheel_polarity %d - forcing +1", left_wheel_polarity_);
    left_wheel_polarity_ = 1;
  }

  if (right_wheel_polarity_ != 1 && right_wheel_polarity_ != -1) {
    RCLCPP_WARN(get_logger(), "Invalid right_wheel_polarity %d - forcing +1", right_wheel_polarity_);
    right_wheel_polarity_ = 1;
  }

  if (interface_name_.empty()) {
    RCLCPP_ERROR(get_logger(),
      "EtherCAT interface parameter 'ethercat_interface' is REQUIRED but not set!");
    RCLCPP_ERROR(get_logger(), "Set it via launch parameter or ros2 param CLI.");
    return false;
  }

  if (interface_name_.length() > 15) {
    RCLCPP_ERROR(get_logger(),
      "Invalid interface name '%s' - too long (max 15 chars per IFNAMSIZ)",
      interface_name_.c_str());
    return false;
  }

  return true;
}

SomanetLifecycleNode::CallbackReturn SomanetLifecycleNode::on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;

  declareAndFetchParameters();
  if (!validateParameters()) {
    return CallbackReturn::FAILURE;
  }

  try {
    odometry_ = std::make_unique<OdometryCalculator>(
      distance_wheels_, wheel_diameter_, left_wheel_polarity_, right_wheel_polarity_);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to construct OdometryCalculator: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(50));
  fault_pub_ = this->create_publisher<std_msgs::msg::String>("safety/fault_events", rclcpp::QoS(10));
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "wheel_cmd_safe", rclcpp::QoS(10),
    std::bind(&SomanetLifecycleNode::cmdVelCallback, this, std::placeholders::_1));
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(shared_from_this());

  auto self_shared = std::static_pointer_cast<SomanetLifecycleNode>(shared_from_this());
  std::weak_ptr<SomanetLifecycleNode> weak_this(self_shared);
  drive_->registerUpdateCallback([weak_this](int left, int right) {
    if (auto self = weak_this.lock()) {
      self->handleDriveFeedback(left, right);
    }
  });

  prev_time_sec_ = this->now().seconds();
  last_cmd_time_sec_.store(prev_time_sec_);
  commanded_left_mrpm_.store(0);
  commanded_right_mrpm_.store(0);
  feedback_left_mrpm_.store(0);
  feedback_right_mrpm_.store(0);
  fault_active_.store(false);

  RCLCPP_INFO(get_logger(),
    "Configured Somanet driver (wheelbase=%.3f m, diameter=%.3f m, iface=%s)",
    distance_wheels_, wheel_diameter_, interface_name_.c_str());

  return CallbackReturn::SUCCESS;
}

SomanetLifecycleNode::CallbackReturn SomanetLifecycleNode::on_activate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Activating Somanet EtherCAT driver");

  if (odom_pub_) {
    odom_pub_->on_activate();
  }
  if (fault_pub_) {
    fault_pub_->on_activate();
  }

  prev_time_sec_ = this->now().seconds();
  last_cmd_time_sec_.store(prev_time_sec_);

  if (!drive_->initialize(interface_name_)) {
    publishFault("Drive failed to initialize");
    return CallbackReturn::FAILURE;
  }

  watchdog_timer_ = this->create_wall_timer(50ms, std::bind(&SomanetLifecycleNode::watchdogTick, this));
  RCLCPP_INFO(get_logger(), "Somanet driver ACTIVE - awaiting commands");
  return CallbackReturn::SUCCESS;
}

SomanetLifecycleNode::CallbackReturn SomanetLifecycleNode::on_deactivate(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Deactivating Somanet EtherCAT driver");

  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_.reset();
  }

  drive_->shutdown();

  if (odom_pub_ && odom_pub_->is_activated()) {
    odom_pub_->on_deactivate();
  }
  if (fault_pub_ && fault_pub_->is_activated()) {
    fault_pub_->on_deactivate();
  }

  return CallbackReturn::SUCCESS;
}

SomanetLifecycleNode::CallbackReturn SomanetLifecycleNode::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(get_logger(), "Cleaning up Somanet driver resources");

  if (watchdog_timer_) {
    watchdog_timer_->cancel();
    watchdog_timer_.reset();
  }

  drive_->shutdown();

  cmd_sub_.reset();
  odom_pub_.reset();
  fault_pub_.reset();
  tf_broadcaster_.reset();
  odometry_.reset();

  commanded_left_mrpm_.store(0);
  commanded_right_mrpm_.store(0);
  feedback_left_mrpm_.store(0);
  feedback_right_mrpm_.store(0);
  fault_active_.store(false);

  return CallbackReturn::SUCCESS;
}

SomanetLifecycleNode::CallbackReturn SomanetLifecycleNode::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Lifecycle shutdown requested from state %s", state.label().c_str());

  if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    on_deactivate(state);
  }

  on_cleanup(state);
  return CallbackReturn::SUCCESS;
}

SomanetLifecycleNode::CallbackReturn SomanetLifecycleNode::on_error(
  const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(get_logger(), "Lifecycle error in state %s", state.label().c_str());
  publishFault("Lifecycle error encountered");
  return CallbackReturn::SUCCESS;
}

void SomanetLifecycleNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    RCLCPP_DEBUG(get_logger(), "Ignoring wheel_cmd_safe while lifecycle node inactive");
    return;
  }

  auto clamp = [](double value, double min_value, double max_value) {
    return std::max(std::min(value, max_value), min_value);
  };

  double linear = clamp(msg->linear.x, -max_linear_vel_, max_linear_vel_);
  double angular = clamp(msg->angular.z, -max_angular_vel_, max_angular_vel_);

  double left_ms = linear - (distance_wheels_ * angular / 2.0);
  double right_ms = linear + (distance_wheels_ * angular / 2.0);

  int left_mrpm = metersPerSecondToMrpm(left_ms, left_wheel_polarity_);
  int right_mrpm = metersPerSecondToMrpm(right_ms, right_wheel_polarity_);

  left_mrpm = std::max(std::min(left_mrpm, kMaxCommandMrpm), -kMaxCommandMrpm);
  right_mrpm = std::max(std::min(right_mrpm, kMaxCommandMrpm), -kMaxCommandMrpm);

  commanded_left_mrpm_.store(left_mrpm);
  commanded_right_mrpm_.store(right_mrpm);
  last_cmd_time_sec_.store(this->now().seconds());

  if (!drive_->setVelocity(left_mrpm, right_mrpm)) {
    publishFault("Drive rejected velocity command");
  } else if (fault_active_.load()) {
    publishRecovery("Velocity commands accepted");
  }
}

void SomanetLifecycleNode::handleDriveFeedback(int left_mrpm, int right_mrpm)
{
  feedback_left_mrpm_.store(left_mrpm);
  feedback_right_mrpm_.store(right_mrpm);
  updateOdometry(left_mrpm, right_mrpm);
}

void SomanetLifecycleNode::watchdogTick()
{
  const double now_sec = this->now().seconds();
  const double last_cmd_sec = last_cmd_time_sec_.load();

  if ((now_sec - last_cmd_sec) > cmd_watchdog_timeout_) {
    if (commanded_left_mrpm_.exchange(0) != 0 || commanded_right_mrpm_.exchange(0) != 0) {
      drive_->setVelocity(0, 0);
    }
  }

  if (!drive_->isOperational()) {
    publishFault("Drive reported non-operational state");
  } else if (fault_active_.load()) {
    publishRecovery("Drive operational");
  }
}

void SomanetLifecycleNode::updateOdometry(int left_mrpm, int right_mrpm)
{
  std::lock_guard<std::mutex> lock(odom_mutex_);
  if (!odometry_) {
    return;
  }

  const double now_sec = this->now().seconds();
  double delta_t = now_sec - prev_time_sec_;
  if (delta_t <= 0.0) {
    prev_time_sec_ = now_sec;
    return;
  }

  if (delta_t > delta_t_warn_threshold_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "Large odometry delta_t %.3f s", delta_t);
  }

  prev_time_sec_ = now_sec;

  if (!odometry_->update(left_mrpm, right_mrpm, delta_t)) {
    return;
  }

  double x, y, yaw;
  odometry_->getPose(x, y, yaw);

  double linear_vel, angular_vel;
  odometry_->getTwist(linear_vel, angular_vel);

  double var_position, var_yaw;
  odometry_->getCovariance(var_position, var_yaw);

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = this->now();
  odom.header.frame_id = odom_frame_id_;
  odom.child_frame_id = child_frame_id_;

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

  if (publish_tf_ && tf_broadcaster_) {
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

void SomanetLifecycleNode::publishFault(const std::string & reason)
{
  if (!fault_pub_ || !fault_pub_->is_activated()) {
    return;
  }

  if (fault_active_.exchange(true)) {
    return;  // Already in fault state
  }

  RCLCPP_ERROR(get_logger(), "FAULT: %s", reason.c_str());
  std_msgs::msg::String msg;
  msg.data = "FAULT: " + reason;
  fault_pub_->publish(msg);
}

void SomanetLifecycleNode::publishRecovery(const std::string & context)
{
  if (!fault_pub_ || !fault_pub_->is_activated()) {
    return;
  }

  bool was_fault = fault_active_.exchange(false);
  if (!was_fault) {
    return;
  }

  if (context.empty()) {
    RCLCPP_INFO(get_logger(), "RECOVERED");
  } else {
    RCLCPP_INFO(get_logger(), "RECOVERED: %s", context.c_str());
  }
  std_msgs::msg::String msg;
  msg.data = context.empty() ? "RECOVERED" : "RECOVERED: " + context;
  fault_pub_->publish(msg);
}

int SomanetLifecycleNode::metersPerSecondToMrpm(double meters_per_second, int wheel_polarity) const
{
  double circumference = kPi * wheel_diameter_;
  if (circumference <= 0.0) {
    return 0;
  }

  double rpm = (meters_per_second / circumference) * 60.0;
  double mrpm = rpm * 1000.0;
  int result = static_cast<int>(std::lround(mrpm));
  return result * wheel_polarity;
}
