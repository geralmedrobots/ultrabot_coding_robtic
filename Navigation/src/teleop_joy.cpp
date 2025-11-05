/**/**

 * @file teleop_joy.cpp * @file teleop_joy.cpp

 * @brief Joystick teleoperation with safety features * @brief Joystick teleoperation with safety features

 *  * 

 * Safety features: * Safety features:

 * - Deadman button requirement * - Deadman button requirement

 * - Command validation and saturation * - Command validation and saturation

 * - Watchdog timeout detection * - Watchdog timeout detection

 * - Operator action logging * - Operator action logging

 * - Parameter validation * - Parameter validation

 *  * 

 * Compliance: ISO 3691-4 (Manual mode operation) * Compliance: ISO 3691-4 (Manual mode operation)

 *  * 

 * @version 1.1.0 * @version 1.1.0

 * @date 2025-10-28 * @date 2025-10-28

 */ */



#include <rclcpp/rclcpp.hpp>#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>#include <geometry_msgs/msg/twist.hpp>

#include <sensor_msgs/msg/joy.hpp>#include <sensor_msgs/msg/joy.hpp>

#include <std_msgs/msg/bool.hpp>#include <std_msgs/msg/bool.hpp>

#include <std_msgs/msg/string.hpp>#include <std_msgs/msg/string.hpp>

#include <memory>#include <memory>

#include <chrono>#include <chrono>

#include <algorithm>

using namespace std::chrono_literals;

using namespace std::chrono_literals;

/**

/** * @class TeleopJoy

 * @class TeleopJoy * @brief Joystick teleoperation node with comprehensive safety features

 * @brief Joystick teleoperation node with comprehensive safety features * 

 *  * Features:

 * Features: * - Configurable axis mapping for different joystick types

 * - Configurable axis mapping for different joystick types * - Deadman button enforcement

 * - Deadman button enforcement * - Throttle control via R2 trigger

 * - Throttle control via R2 trigger * - Velocity scaling and saturation

 * - Velocity scaling and saturation * - Watchdog timeout monitoring

 * - Watchdog timeout monitoring * - Operator action logging for audit trail

 * - Operator action logging for audit trail */

 */class TeleopJoy : public rclcpp::Node

class TeleopJoy : public rclcpp::Node{

{public:

public:  TeleopJoy();

  TeleopJoy();

private:

private:  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);  void watchdogTimerCallback();

  void watchdogTimerCallback();  void logOperatorAction(const std::string& action, double linear, double angular);

  void logOperatorAction(const std::string& action, double linear, double angular);  bool validateParameters();

  bool validateParameters();

  // Axis and button mapping

  // Axis and button mapping  size_t linear_;              // Joystick axis for linear velocity

  size_t linear_;              // Joystick axis for linear velocity  size_t angular_;             // Joystick axis for angular velocity

  size_t angular_;             // Joystick axis for angular velocity  size_t throttle_axis_;       // R2 trigger axis for throttle control

  size_t throttle_axis_;       // R2 trigger axis for throttle control  int deadman_button_;         // Button that must be held to move

  int deadman_button_;         // Button that must be held to move  

    // Safety configuration

  // Safety configuration  bool require_deadman_;       // Whether deadman button is required

  bool require_deadman_;       // Whether deadman button is required  double max_linear_cmd_;      // Maximum linear velocity (m/s or mRPM)

  double max_linear_cmd_;      // Maximum linear velocity (m/s or mRPM)  double max_angular_cmd_;     // Maximum angular velocity (rad/s or mRPM)

  double max_angular_cmd_;     // Maximum angular velocity (rad/s or mRPM)  double watchdog_timeout_;    // Timeout in seconds

  double watchdog_timeout_;    // Timeout in seconds  

    // Scaling factors

  // Scaling factors  double l_scale_;             // Linear velocity scale

  double l_scale_;             // Linear velocity scale  double a_scale_;             // Angular velocity scale

  double a_scale_;             // Angular velocity scale  double r2_vel;               // Current R2 trigger value

  double r2_vel;               // Current R2 trigger value  

    // State tracking

  // State tracking  rclcpp::Time last_joy_time_;

  rclcpp::Time last_joy_time_;  bool watchdog_triggered_;

  bool watchdog_triggered_;  uint64_t command_count_;

  uint64_t command_count_;  std::string operator_id_;

  std::string operator_id_;  

    // Publishers and subscribers

  // Publishers and subscribers  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deadman_pub_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deadman_pub_;  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;  

    // Timer

  // Timer  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;};

};



TeleopJoy::TeleopJoy() : Node("teleop_joy"),

TeleopJoy::TeleopJoy() : Node("teleop_joy"),  linear_(1),

  linear_(1),  angular_(2),

  angular_(2),  throttle_axis_(5),

  throttle_axis_(5),  deadman_button_(5),

  deadman_button_(5),  require_deadman_(true),

  require_deadman_(true),  l_scale_(0.0),

  l_scale_(0.0),  a_scale_(0.0),

  a_scale_(0.0),  r2_vel(0.0),

  r2_vel(0.0),  max_linear_cmd_(1.0),

  max_linear_cmd_(1.0),  max_angular_cmd_(1.0),

  max_angular_cmd_(1.0),  watchdog_timeout_(0.5),

  watchdog_timeout_(0.5),  watchdog_triggered_(false),

  watchdog_triggered_(false),  command_count_(0)

  command_count_(0){

{  // Declare parameters with defaults

  // Declare parameters with defaults  this->declare_parameter("axis_linear", static_cast<int>(linear_));

  this->declare_parameter("axis_linear", static_cast<int>(linear_));  this->declare_parameter("axis_angular", static_cast<int>(angular_));

  this->declare_parameter("axis_angular", static_cast<int>(angular_));  this->declare_parameter("axis_throttle", static_cast<int>(throttle_axis_));

  this->declare_parameter("axis_throttle", static_cast<int>(throttle_axis_));  this->declare_parameter("scale_angular", a_scale_);

  this->declare_parameter("scale_angular", a_scale_);  this->declare_parameter("scale_linear", l_scale_);

  this->declare_parameter("scale_linear", l_scale_);  this->declare_parameter("deadman_button", deadman_button_);

  this->declare_parameter("deadman_button", deadman_button_);  this->declare_parameter("require_deadman", require_deadman_);

  this->declare_parameter("require_deadman", require_deadman_);  this->declare_parameter("max_linear_cmd", max_linear_cmd_);

  this->declare_parameter("max_linear_cmd", max_linear_cmd_);  this->declare_parameter("max_angular_cmd", max_angular_cmd_);

  this->declare_parameter("max_angular_cmd", max_angular_cmd_);  this->declare_parameter("watchdog_timeout", watchdog_timeout_);

  this->declare_parameter("watchdog_timeout", watchdog_timeout_);  this->declare_parameter("operator_id", std::string("unknown"));

  this->declare_parameter("operator_id", std::string("unknown"));

  // Get parameters

  // Get parameters  int temp_linear, temp_angular, temp_throttle;

  int temp_linear, temp_angular, temp_throttle;  this->get_parameter("axis_linear", temp_linear);

  this->get_parameter("axis_linear", temp_linear);  this->get_parameter("axis_angular", temp_angular);

  this->get_parameter("axis_angular", temp_angular);  this->get_parameter("axis_throttle", temp_throttle);

  this->get_parameter("axis_throttle", temp_throttle);  this->get_parameter("scale_angular", a_scale_);

  this->get_parameter("scale_angular", a_scale_);  this->get_parameter("scale_linear", l_scale_);

  this->get_parameter("scale_linear", l_scale_);  this->get_parameter("deadman_button", deadman_button_);

  this->get_parameter("deadman_button", deadman_button_);  this->get_parameter("require_deadman", require_deadman_);

  this->get_parameter("require_deadman", require_deadman_);  this->get_parameter("max_linear_cmd", max_linear_cmd_);

  this->get_parameter("max_linear_cmd", max_linear_cmd_);  this->get_parameter("max_angular_cmd", max_angular_cmd_);

  this->get_parameter("max_angular_cmd", max_angular_cmd_);  this->get_parameter("watchdog_timeout", watchdog_timeout_);

  this->get_parameter("watchdog_timeout", watchdog_timeout_);  this->get_parameter("operator_id", operator_id_);

  this->get_parameter("operator_id", operator_id_);  

    // Convert to size_t after validation

  // Convert to size_t after validation  linear_ = static_cast<size_t>(temp_linear);

  linear_ = static_cast<size_t>(temp_linear);  angular_ = static_cast<size_t>(temp_angular);

  angular_ = static_cast<size_t>(temp_angular);  throttle_axis_ = static_cast<size_t>(temp_throttle);

  throttle_axis_ = static_cast<size_t>(temp_throttle);  

    // Validate parameters

  // Validate parameters  if (!validateParameters()) {

  if (!validateParameters()) {    throw std::invalid_argument("Invalid teleop_joy parameters");

    throw std::invalid_argument("Invalid teleop_joy parameters");  }

  }

  // Publishers

  // Publishers  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 1);

  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_manual", 1);  deadman_pub_ = this->create_publisher<std_msgs::msg::Bool>("deadman_status", 10);

  deadman_pub_ = this->create_publisher<std_msgs::msg::Bool>("deadman_status", 10);  log_pub_ = this->create_publisher<std_msgs::msg::String>("operator_log", 10);

  log_pub_ = this->create_publisher<std_msgs::msg::String>("operator_log", 10);

  // Subscribers

  // Subscribers  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(      "joy", 10, std::bind(&TeleopJoy::joyCallback, this, std::placeholders::_1));

      "joy", 10, std::bind(&TeleopJoy::joyCallback, this, std::placeholders::_1));

  // Watchdog timer

  // Watchdog timer  watchdog_timer_ = this->create_wall_timer(

  watchdog_timer_ = this->create_wall_timer(      std::chrono::milliseconds(static_cast<int>(watchdog_timeout_ * 1000 / 2)),

      std::chrono::milliseconds(static_cast<int>(watchdog_timeout_ * 1000 / 2)),      std::bind(&TeleopJoy::watchdogTimerCallback, this));

      std::bind(&TeleopJoy::watchdogTimerCallback, this));  

    last_joy_time_ = this->now();

  last_joy_time_ = this->now();  

    RCLCPP_INFO(this->get_logger(), 

  RCLCPP_INFO(this->get_logger(),     "Teleop Joystick initialized - Operator: %s", operator_id_.c_str());

    "Teleop Joystick initialized - Operator: %s", operator_id_.c_str());  RCLCPP_INFO(this->get_logger(),

  RCLCPP_INFO(this->get_logger(),    "Config: linear_axis=%zu, angular_axis=%zu, throttle_axis=%zu, deadman_button=%d",

    "Config: linear_axis=%zu, angular_axis=%zu, throttle_axis=%zu, deadman_button=%d",    linear_, angular_, throttle_axis_, deadman_button_);

    linear_, angular_, throttle_axis_, deadman_button_);  RCLCPP_INFO(this->get_logger(),

  RCLCPP_INFO(this->get_logger(),    "Limits: max_linear=%.2f, max_angular=%.2f, watchdog=%.2fs",

    "Limits: max_linear=%.2f, max_angular=%.2f, watchdog=%.2fs",    max_linear_cmd_, max_angular_cmd_, watchdog_timeout_);

    max_linear_cmd_, max_angular_cmd_, watchdog_timeout_);  

    logOperatorAction("TELEOP_STARTED", 0.0, 0.0);

  logOperatorAction("TELEOP_STARTED", 0.0, 0.0);}

}

bool TeleopJoy::validateParameters()

bool TeleopJoy::validateParameters(){

{  bool valid = true;

  bool valid = true;  

    if (max_linear_cmd_ <= 0.0) {

  if (max_linear_cmd_ <= 0.0) {    RCLCPP_ERROR(this->get_logger(), "max_linear_cmd must be > 0");

    RCLCPP_ERROR(this->get_logger(), "max_linear_cmd must be > 0");    valid = false;

    valid = false;  }

  }  

    if (max_angular_cmd_ <= 0.0) {

  if (max_angular_cmd_ <= 0.0) {    RCLCPP_ERROR(this->get_logger(), "max_angular_cmd must be > 0");

    RCLCPP_ERROR(this->get_logger(), "max_angular_cmd must be > 0");    valid = false;

    valid = false;  }

  }  

    if (watchdog_timeout_ <= 0.0) {

  if (watchdog_timeout_ <= 0.0) {    RCLCPP_ERROR(this->get_logger(), "watchdog_timeout must be > 0");

    RCLCPP_ERROR(this->get_logger(), "watchdog_timeout must be > 0");    valid = false;

    valid = false;  }

  }  

    if (deadman_button_ < -1) {

  if (deadman_button_ < -1) {    RCLCPP_ERROR(this->get_logger(), "deadman_button must be >= -1 (-1 to disable)");

    RCLCPP_ERROR(this->get_logger(), "deadman_button must be >= -1 (-1 to disable)");    valid = false;

    valid = false;  }

  }  

    return valid;

  return valid;}

}

void TeleopJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)

void TeleopJoy::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy){

{  last_joy_time_ = this->now();

  last_joy_time_ = this->now();  watchdog_triggered_ = false;

  watchdog_triggered_ = false;  

    geometry_msgs::msg::Twist twist;

  geometry_msgs::msg::Twist twist;  

    // Safety check: verify joy message has expected data

  // Safety check: verify joy message has expected data  if (joy->axes.empty() || joy->buttons.empty()) {

  if (joy->axes.empty() || joy->buttons.empty()) {    RCLCPP_WARN(this->get_logger(), "Received empty joystick message");

    RCLCPP_WARN(this->get_logger(), "Received empty joystick message");    vel_pub_->publish(twist); // Publish stop command

    vel_pub_->publish(twist); // Publish stop command    return;

    return;  }

  }  

    // Read throttle (R2 trigger: -1=released, 1=pressed on many controllers)

  // Read throttle (R2 trigger: -1=released, 1=pressed on many controllers)  // Some controllers use axis 5, others use different axes

  // Some controllers use axis 5, others use different axes  r2_vel = (joy->axes.size() > throttle_axis_ ? joy->axes[throttle_axis_] : 0.0);

  r2_vel = (joy->axes.size() > throttle_axis_ ? joy->axes[throttle_axis_] : 0.0);

  // Deadman logic: require a specific button to be held to move

  // Deadman logic: require a specific button to be held to move  bool deadman_pressed = false;

  bool deadman_pressed = false;  if (deadman_button_ >= 0 && static_cast<size_t>(deadman_button_) < joy->buttons.size()) {

  if (deadman_button_ >= 0 && static_cast<size_t>(deadman_button_) < joy->buttons.size()) {    deadman_pressed = (joy->buttons[deadman_button_] != 0);

    deadman_pressed = (joy->buttons[deadman_button_] != 0);  }

  }  

    // Publish deadman status for safety supervisor

  // Publish deadman status for safety supervisor  std_msgs::msg::Bool deadman_msg;

  std_msgs::msg::Bool deadman_msg;  deadman_msg.data = deadman_pressed;

  deadman_msg.data = deadman_pressed;  deadman_pub_->publish(deadman_msg);

  deadman_pub_->publish(deadman_msg);

  // Default to stop

  // Default to stop  twist.angular.z = 0.0;

  twist.angular.z = 0.0;  twist.linear.x = 0.0;

  twist.linear.x = 0.0;

  if (!require_deadman_ || deadman_pressed) {

  if (!require_deadman_ || deadman_pressed) {    // R2 trigger logic: negative values mean pressed on most controllers

    // R2 trigger logic: negative values mean pressed on most controllers    // Normalize to 0..1 range where 1 = full throttle

    // Normalize to 0..1 range where 1 = full throttle    if (r2_vel < 0) {

    if (r2_vel < 0) {      // R2 axis convention: -1 = full throttle, 0 = released

      // R2 axis convention: -1 = full throttle, 0 = released      double throttle_scale = -r2_vel; // Convert to 0..1 range

      double throttle_scale = -r2_vel; // Convert to 0..1 range

      // Read joystick axes with bounds checking

      // Read joystick axes with bounds checking      double raw_ang = 0.0;

      double raw_ang = 0.0;      double raw_lin = 0.0;

      double raw_lin = 0.0;      

            if (joy->axes.size() > angular_) {

      if (joy->axes.size() > angular_) {        raw_ang = a_scale_ * joy->axes[angular_];

        raw_ang = a_scale_ * joy->axes[angular_];      }

      }      

            if (joy->axes.size() > linear_) {

      if (joy->axes.size() > linear_) {        raw_lin = l_scale_ * joy->axes[linear_];

        raw_lin = l_scale_ * joy->axes[linear_];      }

      }

      // Apply throttle scaling

      // Apply throttle scaling      raw_ang *= throttle_scale;

      raw_ang *= throttle_scale;      raw_lin *= throttle_scale;

      raw_lin *= throttle_scale;

      // Clamp commands to safety limits

      // Clamp commands to safety limits      raw_lin = std::clamp(raw_lin, -max_linear_cmd_, max_linear_cmd_);

      raw_lin = std::clamp(raw_lin, -max_linear_cmd_, max_linear_cmd_);      raw_ang = std::clamp(raw_ang, -max_angular_cmd_, max_angular_cmd_);

      raw_ang = std::clamp(raw_ang, -max_angular_cmd_, max_angular_cmd_);

      twist.angular.z = raw_ang;

      twist.angular.z = raw_ang;      twist.linear.x = raw_lin;

      twist.linear.x = raw_lin;      

            // Log significant commands (not every tiny joystick movement)

      // Log significant commands (not every tiny joystick movement)      if (std::abs(raw_lin) > 0.01 || std::abs(raw_ang) > 0.01) {

      if (std::abs(raw_lin) > 0.01 || std::abs(raw_ang) > 0.01) {        command_count_++;

        command_count_++;        if (command_count_ % 50 == 0) { // Log every 50 commands to avoid spam

        if (command_count_ % 50 == 0) { // Log every 50 commands to avoid spam          logOperatorAction("COMMAND", raw_lin, raw_ang);

          logOperatorAction("COMMAND", raw_lin, raw_ang);        }

        }      }

      }    }

    }  } else {

  } else {    // Deadman not pressed - log this event periodically

    // Deadman not pressed - log this event periodically    static uint64_t deadman_warn_count = 0;

    static uint64_t deadman_warn_count = 0;    if (++deadman_warn_count % 100 == 0) {

    if (++deadman_warn_count % 100 == 0) {      RCLCPP_DEBUG(this->get_logger(), "Deadman button not pressed - robot stopped");

      RCLCPP_DEBUG(this->get_logger(), "Deadman button not pressed - robot stopped");    }

    }  }

  }

  vel_pub_->publish(twist);

  vel_pub_->publish(twist);  

    RCLCPP_DEBUG(this->get_logger(), 

  RCLCPP_DEBUG(this->get_logger(),     "joy -> lin=%.2f, ang=%.2f, throttle=%.2f, deadman=%s",

    "joy -> lin=%.2f, ang=%.2f, throttle=%.2f, deadman=%s",    twist.linear.x, twist.angular.z, r2_vel, deadman_pressed ? "YES" : "NO");

    twist.linear.x, twist.angular.z, r2_vel, deadman_pressed ? "YES" : "NO");}

}

void TeleopJoy::watchdogTimerCallback()

void TeleopJoy::watchdogTimerCallback(){

{  auto elapsed = (this->now() - last_joy_time_).seconds();

  auto elapsed = (this->now() - last_joy_time_).seconds();  

    if (elapsed > watchdog_timeout_ && !watchdog_triggered_) {

  if (elapsed > watchdog_timeout_ && !watchdog_triggered_) {    watchdog_triggered_ = true;

    watchdog_triggered_ = true;    RCLCPP_WARN(this->get_logger(), 

    RCLCPP_WARN(this->get_logger(),       "Joystick watchdog timeout: %.2fs since last message", elapsed);

      "Joystick watchdog timeout: %.2fs since last message", elapsed);    logOperatorAction("WATCHDOG_TIMEOUT", 0.0, 0.0);

    logOperatorAction("WATCHDOG_TIMEOUT", 0.0, 0.0);    

        // Publish stop command

    // Publish stop command    geometry_msgs::msg::Twist stop_cmd;

    geometry_msgs::msg::Twist stop_cmd;    vel_pub_->publish(stop_cmd);

    vel_pub_->publish(stop_cmd);    

        // Publish deadman inactive

    // Publish deadman inactive    std_msgs::msg::Bool deadman_msg;

    std_msgs::msg::Bool deadman_msg;    deadman_msg.data = false;

    deadman_msg.data = false;    deadman_pub_->publish(deadman_msg);

    deadman_pub_->publish(deadman_msg);  }

  }}

}

void TeleopJoy::logOperatorAction(const std::string& action, double linear, double angular)

void TeleopJoy::logOperatorAction(const std::string& action, double linear, double angular){

{  std_msgs::msg::String log_msg;

  std_msgs::msg::String log_msg;  

    // Format: TIMESTAMP | OPERATOR_ID | ACTION | LINEAR | ANGULAR

  // Format: TIMESTAMP | OPERATOR_ID | ACTION | LINEAR | ANGULAR  auto now = this->now();

  auto now = this->now();  log_msg.data = std::to_string(now.seconds()) + " | " +

  log_msg.data = std::to_string(now.seconds()) + " | " +                 operator_id_ + " | " +

                 operator_id_ + " | " +                 action + " | " +

                 action + " | " +                 "lin=" + std::to_string(linear) + " | " +

                 "lin=" + std::to_string(linear) + " | " +                 "ang=" + std::to_string(angular);

                 "ang=" + std::to_string(angular);  

    log_pub_->publish(log_msg);

  log_pub_->publish(log_msg);  

    RCLCPP_INFO(this->get_logger(), "Operator log: %s", log_msg.data.c_str());

  RCLCPP_INFO(this->get_logger(), "Operator log: %s", log_msg.data.c_str());}

}



int main(int argc, char** argv)

int main(int argc, char** argv){

{  rclcpp::init(argc, argv);

  rclcpp::init(argc, argv);  auto teleop_joy = std::make_shared<TeleopJoy>();

    rclcpp::spin(teleop_joy);

  try {  rclcpp::shutdown();

    auto teleop_joy = std::make_shared<TeleopJoy>();  return 0;

    rclcpp::spin(teleop_joy);}
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("teleop_joy"), 
      "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
