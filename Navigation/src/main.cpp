#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "ethercat_driver.hpp"
#include "somanet_lifecycle_node.hpp"

namespace
{
struct CliOptions
{
  bool autostart{false};
};

CliOptions parse_cli_arguments(int argc, char * argv[], std::vector<char *> & filtered)
{
  CliOptions options;
  bool autostart_set = false;

  filtered.clear();
  filtered.reserve(static_cast<size_t>(argc) + 1);
  filtered.push_back(argv[0]);

  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--autostart") == 0) {
      options.autostart = true;
      autostart_set = true;
      continue;
    }

    if (std::strcmp(argv[i], "--no-autostart") == 0) {
      options.autostart = false;
      autostart_set = true;
      continue;
    }

    filtered.push_back(argv[i]);
  }

  filtered.push_back(nullptr);

  if (!autostart_set) {
    if (const char * env = std::getenv("ULTRABOT_AUTOSTART")) {
      std::string value(env);
      std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      options.autostart = (value == "1" || value == "true" || value == "yes" || value == "on");
    }
  }

  return options;
}
}

int main(int argc, char * argv[])
{
  std::vector<char *> filtered_args;
  CliOptions options;

  try {
    options = parse_cli_arguments(argc, argv, filtered_args);
  } catch (const std::exception & e) {
    std::cerr << "Failed to process command-line arguments: " << e.what() << std::endl;
    return 1;
  }

  int filtered_argc = static_cast<int>(filtered_args.size()) - 1;
  rclcpp::init(filtered_argc, filtered_args.data());

  auto drive = std::make_shared<EthercatDriver>();
  auto node = std::make_shared<SomanetLifecycleNode>(drive);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  if (options.autostart) {
    RCLCPP_WARN(rclcpp::get_logger("somanet_driver"),
      "Autostart enabled – configuring and activating driver immediately");

    if (node->configure() != SomanetLifecycleNode::CallbackReturn::SUCCESS) {
      RCLCPP_FATAL(rclcpp::get_logger("somanet_driver"),
        "Failed to configure Somanet lifecycle node");
      rclcpp::shutdown();
      return 1;
    }

    if (node->activate() != SomanetLifecycleNode::CallbackReturn::SUCCESS) {
      RCLCPP_FATAL(rclcpp::get_logger("somanet_driver"),
        "Failed to activate Somanet lifecycle node");
      rclcpp::shutdown();
      return 1;
    }
  } else {
    RCLCPP_INFO(rclcpp::get_logger("somanet_driver"),
      "Autostart disabled – waiting for external lifecycle transitions");
  }

  executor.spin();
  executor.remove_node(node->get_node_base_interface());

  int exit_code = 0;
  const auto current_state = node->get_current_state().id();
  if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    if (node->deactivate() != SomanetLifecycleNode::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("somanet_driver"),
        "Error deactivating Somanet lifecycle node during shutdown");
      exit_code = 1;
    }
  }

  if (node->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    if (node->cleanup() != SomanetLifecycleNode::CallbackReturn::SUCCESS) {
      RCLCPP_ERROR(rclcpp::get_logger("somanet_driver"),
        "Error cleaning up Somanet lifecycle node during shutdown");
      exit_code = 1;
    }
  }

  rclcpp::shutdown();
  return exit_code;
}
