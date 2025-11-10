#ifndef COMMAND_WATCHDOG_HPP
#define COMMAND_WATCHDOG_HPP

#include <atomic>
#include <utility>

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Tracks freshness of velocity commands and exposes timeout checks
 *        without tying the logic to ROS timers directly.
 */
class CommandWatchdog
{
public:
  explicit CommandWatchdog(rclcpp::Clock::SharedPtr clock);

  void setTimeout(double timeout_seconds);

  void recordCommand(int left_mrpm, int right_mrpm);

  bool timedOut() const;

  double timeSinceLastCommand() const;

  std::pair<int, int> lastCommand() const;

private:
  rclcpp::Clock::SharedPtr clock_;
  std::atomic<double> timeout_{0.5};
  std::atomic<double> last_command_time_{0.0};
  std::atomic<int> last_left_{0};
  std::atomic<int> last_right_{0};
};

#endif  // COMMAND_WATCHDOG_HPP
