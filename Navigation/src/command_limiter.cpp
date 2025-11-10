#include "command_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kPi = 3.14159265358979323846;

inline double clamp(double value, double min_value, double max_value)
{
  return std::max(std::min(value, max_value), min_value);
}
}  // namespace

void CommandLimiter::setParameters(
  double wheel_base,
  double wheel_diameter,
  double max_linear,
  double max_angular,
  int left_polarity,
  int right_polarity)
{
  wheel_base_ = wheel_base;
  wheel_diameter_ = wheel_diameter;
  max_linear_ = std::abs(max_linear);
  max_angular_ = std::abs(max_angular);
  left_polarity_ = (left_polarity >= 0) ? 1 : -1;
  right_polarity_ = (right_polarity >= 0) ? 1 : -1;
}

std::pair<int, int> CommandLimiter::computeWheelSpeeds(const geometry_msgs::msg::Twist & cmd) const
{
  if (wheel_diameter_ <= 0.0) {
    return {0, 0};
  }

  const double linear = clamp(cmd.linear.x, -max_linear_, max_linear_);
  const double angular = clamp(cmd.angular.z, -max_angular_, max_angular_);

  const double half_base = wheel_base_ / 2.0;
  const double left_ms = linear - (half_base * angular);
  const double right_ms = linear + (half_base * angular);

  int left_mrpm = metersPerSecondToMrpm(left_ms, left_polarity_);
  int right_mrpm = metersPerSecondToMrpm(right_ms, right_polarity_);

  left_mrpm = static_cast<int>(clamp(left_mrpm, -kMaxCommandMrpm, kMaxCommandMrpm));
  right_mrpm = static_cast<int>(clamp(right_mrpm, -kMaxCommandMrpm, kMaxCommandMrpm));

  return {left_mrpm, right_mrpm};
}

int CommandLimiter::metersPerSecondToMrpm(double meters_per_second, int wheel_polarity) const
{
  const double circumference = kPi * wheel_diameter_;
  if (circumference <= 0.0) {
    return 0;
  }

  const double rpm = (meters_per_second / circumference) * 60.0;
  const double mrpm = rpm * 1000.0;
  const int result = static_cast<int>(std::lround(mrpm));
  return result * wheel_polarity;
}
