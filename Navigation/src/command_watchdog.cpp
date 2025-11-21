#include "command_watchdog.hpp"

#include <algorithm>
#include <stdexcept>

CommandWatchdog::CommandWatchdog(rclcpp::Clock::SharedPtr clock)
: clock_(std::move(clock))
{
  if (!clock_) {
    throw std::invalid_argument("CommandWatchdog requires a valid clock");
  }

  // Seed the last command time to "now" so freshly created watchdogs don't
  // immediately report a timeout before the first command arrives.
  last_command_time_.store(clock_->now().seconds());
}

void CommandWatchdog::setTimeout(double timeout_seconds)
{
  timeout_.store(std::max(0.0, timeout_seconds));
}

void CommandWatchdog::recordCommand(int left_mrpm, int right_mrpm)
{
  last_left_.store(left_mrpm);
  last_right_.store(right_mrpm);
  const double now = clock_->now().seconds();
  last_command_time_.store(now);
}

bool CommandWatchdog::timedOut() const
{
  const double elapsed = timeSinceLastCommand();
  return elapsed > timeout_.load();
}

double CommandWatchdog::timeSinceLastCommand() const
{
  const double last = last_command_time_.load();
  const double now = clock_->now().seconds();
  return now - last;
}

std::pair<int, int> CommandWatchdog::lastCommand() const
{
  return {last_left_.load(), last_right_.load()};
}
