#include "somanet_lifecycle_node.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include <lifecycle_msgs/msg/state.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr int kMaxCommandMrpm = CommandLimiter::kMaxCommandMrpm;
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

  command_watchdog_ = std::make_unique<CommandWatchdog>(this->get_clock());
  odom_publisher_ = std::make_unique<OdometryPublisher>(*this);
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
    command_limiter_.setParameters(
      distance_wheels_,
      wheel_diameter_,
      max_linear_vel_,
      max_angular_vel_,
      left_wheel_polarity_,
      right_wheel_polarity_);

    if (command_watchdog_) {
      command_watchdog_->setTimeout(cmd_watchdog_timeout_);
      command_watchdog_->recordCommand(0, 0);
    }

    if (!odom_publisher_) {
      odom_publisher_ = std::make_unique<OdometryPublisher>(*this);
    }

    OdometryPublisher::Config odom_config;
    odom_config.distance_wheels = distance_wheels_;
    odom_config.wheel_diameter = wheel_diameter_;
    odom_config.left_wheel_polarity = left_wheel_polarity_;
    odom_config.right_wheel_polarity = right_wheel_polarity_;
    odom_config.delta_t_warn_threshold = delta_t_warn_threshold_;
    odom_config.odom_frame_id = odom_frame_id_;
    odom_config.child_frame_id = child_frame_id_;
    odom_config.publish_tf = publish_tf_;
    odom_publisher_->configure(odom_config);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "Failed to configure driver helpers: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  fault_pub_ = this->create_publisher<std_msgs::msg::String>("safety/fault_events", rclcpp::QoS(10));
  cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "wheel_cmd_safe", rclcpp::QoS(10),
    std::bind(&SomanetLifecycleNode::cmdVelCallback, this, std::placeholders::_1));

  auto self_shared = std::static_pointer_cast<SomanetLifecycleNode>(shared_from_this());
  std::weak_ptr<SomanetLifecycleNode> weak_this(self_shared);
  drive_->registerUpdateCallback([weak_this](int left, int right) {
    if (auto self = weak_this.lock()) {
      self->handleDriveFeedback(left, right);
    }
  });

  commanded_left_mrpm_.store(0);
  commanded_right_mrpm_.store(0);
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

  if (odom_publisher_) {
    odom_publisher_->activate();
  }
  if (fault_pub_) {
    fault_pub_->on_activate();
  }

  if (command_watchdog_) {
    command_watchdog_->recordCommand(0, 0);
  }

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

  if (odom_publisher_) {
    odom_publisher_->deactivate();
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
  fault_pub_.reset();
  if (odom_publisher_) {
    odom_publisher_->cleanup();
  }

  commanded_left_mrpm_.store(0);
  commanded_right_mrpm_.store(0);
  fault_active_.store(false);

  if (command_watchdog_) {
    command_watchdog_->recordCommand(0, 0);
  }

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

  const auto wheel_speeds = command_limiter_.computeWheelSpeeds(*msg);

  commanded_left_mrpm_.store(wheel_speeds.first);
  commanded_right_mrpm_.store(wheel_speeds.second);

  if (command_watchdog_) {
    command_watchdog_->recordCommand(wheel_speeds.first, wheel_speeds.second);
  }

  if (!drive_->setVelocity(wheel_speeds.first, wheel_speeds.second)) {
    publishFault("Drive rejected velocity command");
  } else if (fault_active_.load()) {
    publishRecovery("Velocity commands accepted");
  }
}

void SomanetLifecycleNode::handleDriveFeedback(int left_mrpm, int right_mrpm)
{
  if (odom_publisher_) {
    odom_publisher_->updateFromFeedback(left_mrpm, right_mrpm);
  }
}

void SomanetLifecycleNode::watchdogTick()
{
  if (command_watchdog_ && command_watchdog_->timedOut()) {
    if (commanded_left_mrpm_.exchange(0) != 0 || commanded_right_mrpm_.exchange(0) != 0) {
      drive_->setVelocity(0, 0);
      command_watchdog_->recordCommand(0, 0);
    }
  }

  if (!drive_->isOperational()) {
    publishFault("Drive reported non-operational state");
  } else if (fault_active_.load()) {
    publishRecovery("Drive operational");
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
