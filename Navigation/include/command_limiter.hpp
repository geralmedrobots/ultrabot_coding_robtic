#ifndef COMMAND_LIMITER_HPP
#define COMMAND_LIMITER_HPP

#include <utility>

#include <geometry_msgs/msg/twist.hpp>

/**
 * @brief Utility responsible for clamping Twist commands and converting them
 *        to wheel velocities expressed in milli-RPM.
 *
 *  By isolating this logic from the lifecycle node we make it easier to unit
 *  test the conversion rules in isolation and to reason about the safety
 *  envelopes applied to incoming commands.
 */
class CommandLimiter
{
public:
  CommandLimiter() = default;

  CommandLimiter(
    double wheel_base,
    double wheel_diameter,
    double max_linear,
    double max_angular,
    int left_polarity,
    int right_polarity)
  {
    setParameters(wheel_base, wheel_diameter, max_linear, max_angular, left_polarity, right_polarity);
  }

  void setParameters(
    double wheel_base,
    double wheel_diameter,
    double max_linear,
    double max_angular,
    int left_polarity,
    int right_polarity);

  std::pair<int, int> computeWheelSpeeds(const geometry_msgs::msg::Twist & cmd) const;

  static constexpr int kMaxCommandMrpm = 20000;

private:
  int metersPerSecondToMrpm(double meters_per_second, int wheel_polarity) const;

  double wheel_base_{0.0};
  double wheel_diameter_{0.0};
  double max_linear_{0.0};
  double max_angular_{0.0};
  int left_polarity_{1};
  int right_polarity_{1};
};

#endif  // COMMAND_LIMITER_HPP
