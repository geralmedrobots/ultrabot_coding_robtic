/**
 * @file teleop_joy.cpp
 * @brief Joystick teleoperation with safety features
 * 
 * Safety features:
 * - Deadman button requirement
 * - Command validation and saturation
 * - Watchdog timeout detection
 * - Operator action logging
 * - Parameter validation
 * 
 * Compliance: ISO 3691-4 (Manual mode operation)
 * 
 * @version 1.1.0
 * @date 2025-10-28
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

/**
 * @class TeleopJoy
 * @brief Joystick teleoperation node with comprehensive safety features
 * 
 * Features:
 * - Configurable axis mapping for different joystick types
 * - Deadman button enforcement
 * - Throttle control via R2 trigger
 * - Velocity scaling and saturation
 * - Watchdog timeout monitoring
 * - Operator action logging for audit trail
 */
class TeleopJoy : public rclcpp::Node
{
public:
  TeleopJoy();

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void watchdogTimerCallback();
  void logOperatorAction(const std::string& action, double linear, double angular);
  bool validateParameters();

  // Axis and button mapping
  size_t linear_;              // Joystick axis for linear velocity
  size_t angular_;             // Joystick axis for angular velocity
  size_t throttle_axis_;       // R2 trigger axis for throttle control
  int deadman_button_;         // Button that must be held to move
  
  // Safety configuration
  bool require_deadman_;       // Whether deadman button is required
  double max_linear_cmd_;      // Maximum linear velocity (m/s or mRPM)
  double max_angular_cmd_;     // Maximum angular velocity (rad/s or mRPM)
  double watchdog_timeout_;    // Timeout in seconds
  
  // Scaling factors
  double l_scale_;             // Linear velocity scale
  double a_scale_;             // Angular velocity scale
  double r2_vel;               // Current R2 trigger value
  
  // State tracking
  rclcpp::Time last_joy_time_;
  bool watchdog_triggered_;
  uint64_t command_count_;
  std::string operator_id_;
  
  // Publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deadman_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  
  // Timer for watchdog monitoring
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

TeleopJoy::TeleopJoy()
  : Node("teleop_joy"),
    linear_(1),
    angular_(3),
    throttle_axis_(5),
    deadman_button_(5),
    require_deadman_(true),
    max_linear_cmd_(1.0),
    max_angular_cmd_(1.0),
    watchdog_timeout_(0.5),
    l_scale_(0.0),
    a_scale_(0.0),
    r2_vel(0.0),
    watchdog_triggered_(false),
    command_count_(0),
    operator_id_("unknown")
{
  // Declare and get parameters
  this->declare_parameter("linear_axis", static_cast<int>(linear_));
  this->declare_parameter("angular_axis", static_cast<int>(angular_));
  this->declare_parameter("throttle_axis", static_cast<int>(throttle_axis_));
  this->declare_parameter("deadman_button", deadman_button_);
  this->declare_parameter("require_deadman", require_deadman_);
  this->declare_parameter("max_linear_cmd", max_linear_cmd_);
  this->declare_parameter("max_angular_cmd", max_angular_cmd_);
  this->declare_parameter("watchdog_timeout", watchdog_timeout_);
  this->declare_parameter("operator_id", operator_id_);
  this->declare_parameter("l_scale", 1.0);
  this->declare_parameter("a_scale", 1.0);

  linear_ = static_cast<size_t>(this->get_parameter("linear_axis").as_int());
  angular_ = static_cast<size_t>(this->get_parameter("angular_axis").as_int());
  throttle_axis_ = static_cast<size_t>(this->get_parameter("throttle_axis").as_int());
  deadman_button_ = this->get_parameter("deadman_button").as_int();
  require_deadman_ = this->get_parameter("require_deadman").as_bool();
  max_linear_cmd_ = this->get_parameter("max_linear_cmd").as_double();
  max_angular_cmd_ = this->get_parameter("max_angular_cmd").as_double();
  watchdog_timeout_ = this->get_parameter("watchdog_timeout").as_double();
  operator_id_ = this->get_parameter("operator_id").as_string();
  l_scale_ = this->get_parameter("l_scale").as_double();
  a_scale_ = this->get_parameter("a_scale").as_double();

  // Validate parameters
  if (!validateParameters()) {
    RCLCPP_FATAL(this->get_logger(), "Parameter validation failed!");
    throw std::runtime_error("Invalid parameters");
  }

  // Create publishers
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  deadman_pub_ = this->create_publisher<std_msgs::msg::Bool>("/deadman_status", 10);
  log_pub_ = this->create_publisher<std_msgs::msg::String>("/operator_log", 10);

  // Create subscriber
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&TeleopJoy::joyCallback, this, std::placeholders::_1));

  // Create watchdog timer
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(watchdog_timeout_ * 1000.0)),
    std::bind(&TeleopJoy::watchdogTimerCallback, this));

  last_joy_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "Teleop Joystick initialized - Operator: %s", operator_id_.c_str());
  RCLCPP_INFO(this->get_logger(),
    "Config: linear_axis=%zu, angular_axis=%zu, throttle_axis=%zu, deadman_button=%d",
    linear_, angular_, throttle_axis_, deadman_button_);
  RCLCPP_INFO(this->get_logger(),
    "Limits: max_linear=%.2f, max_angular=%.2f, watchdog=%.2fs",
    max_linear_cmd_, max_angular_cmd_, watchdog_timeout_);
}

bool TeleopJoy::validateParameters()
{
  bool valid = true;

  // Validate velocity limits
  if (max_linear_cmd_ <= 0.0 || max_linear_cmd_ > 5.0) {
    RCLCPP_ERROR(this->get_logger(),
      "Invalid max_linear_cmd: %.2f (must be in (0, 5.0])", max_linear_cmd_);
    valid = false;
  }

  if (max_angular_cmd_ <= 0.0 || max_angular_cmd_ > 10.0) {
    RCLCPP_ERROR(this->get_logger(),
      "Invalid max_angular_cmd: %.2f (must be in (0, 10.0])", max_angular_cmd_);
    valid = false;
  }

  // Validate watchdog timeout
  if (watchdog_timeout_ <= 0.01 || watchdog_timeout_ > 10.0) {
    RCLCPP_ERROR(this->get_logger(),
      "Invalid watchdog_timeout: %.2f (must be in (0.01, 10.0])", watchdog_timeout_);
    valid = false;
  }

  // Validate scales
  if (l_scale_ <= 0.0 || l_scale_ > 2.0) {
    RCLCPP_ERROR(this->get_logger(),
      "Invalid l_scale: %.2f (must be in (0, 2.0])", l_scale_);
    valid = false;
  }

  if (a_scale_ <= 0.0 || a_scale_ > 2.0) {
    RCLCPP_ERROR(this->get_logger(),
      "Invalid a_scale: %.2f (must be in (0, 2.0])", a_scale_);
    valid = false;
  }

  return valid;
}

void TeleopJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
  // Update last joy time
  last_joy_time_ = this->now();
  watchdog_triggered_ = false;

  // Check if deadman button is pressed
  bool deadman_pressed = false;
  if (deadman_button_ >= 0 && static_cast<size_t>(deadman_button_) < joy->buttons.size()) {
    deadman_pressed = joy->buttons[deadman_button_] > 0;
  }

  // Publish deadman status
  auto deadman_msg = std_msgs::msg::Bool();
  deadman_msg.data = deadman_pressed;
  deadman_pub_->publish(deadman_msg);

  // Create twist message
  auto twist = geometry_msgs::msg::Twist();

  // If deadman is required and not pressed, send zero velocity
  if (require_deadman_ && !deadman_pressed) {
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    vel_pub_->publish(twist);
    return;
  }

  // Get throttle value from R2 trigger (usually ranges from -1.0 to 1.0)
  double throttle_scale = 1.0;
  if (throttle_axis_ < joy->axes.size()) {
    // R2 trigger typically goes from 1.0 (not pressed) to -1.0 (fully pressed)
    // Convert to 0.0-1.0 scale
    double raw_throttle = joy->axes[throttle_axis_];
    throttle_scale = (1.0 - raw_throttle) / 2.0;  // Convert from [-1,1] to [0,1]
    throttle_scale = std::max(0.0, std::min(1.0, throttle_scale));  // Clamp
  }

  // Get raw axis values
  double raw_linear = 0.0;
  double raw_angular = 0.0;

  if (linear_ < joy->axes.size()) {
    raw_linear = joy->axes[linear_];
  }

  if (angular_ < joy->axes.size()) {
    raw_angular = joy->axes[angular_];
  }

  // Apply scaling
  raw_linear *= l_scale_;
  raw_angular *= a_scale_;

  // Apply throttle
  raw_linear *= throttle_scale;
  raw_angular *= throttle_scale;

  // Apply velocity limits (saturation)
  twist.linear.x = std::max(-max_linear_cmd_, std::min(max_linear_cmd_, raw_linear));
  twist.angular.z = std::max(-max_angular_cmd_, std::min(max_angular_cmd_, raw_angular));

  // Publish velocity command
  vel_pub_->publish(twist);

  // Log operator action (throttled to avoid log flooding)
  command_count_++;
  if (command_count_ % 50 == 0) {  // Log every 50th command
    logOperatorAction("joy_command", twist.linear.x, twist.angular.z);
  }
}

void TeleopJoy::watchdogTimerCallback()
{
  // Check if joystick messages are being received
  auto time_since_last_joy = (this->now() - last_joy_time_).seconds();

  if (time_since_last_joy > watchdog_timeout_ && !watchdog_triggered_) {
    watchdog_triggered_ = true;

    // Send zero velocity
    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    vel_pub_->publish(twist);

    RCLCPP_WARN(this->get_logger(),
      "Watchdog timeout! No joystick input for %.2fs. Sending zero velocity.",
      time_since_last_joy);

    logOperatorAction("watchdog_timeout", 0.0, 0.0);
  }
}

void TeleopJoy::logOperatorAction(const std::string& action, double linear, double angular)
{
  auto log_msg = std_msgs::msg::String();
  
  // Format: [timestamp] operator_id: action (linear=X, angular=Y)
  auto now = this->now();
  log_msg.data = "[" + std::to_string(now.seconds()) + "] " +
                 operator_id_ + ": " + action +
                 " (linear=" + std::to_string(linear) +
                 ", angular=" + std::to_string(angular) + ")";
  
  log_pub_->publish(log_msg);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto teleop_joy = std::make_shared<TeleopJoy>();
    rclcpp::spin(teleop_joy);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("teleop_joy"), 
      "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
