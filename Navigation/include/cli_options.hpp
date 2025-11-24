#pragma once

#include <algorithm>
#include <cctype>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstdlib>

namespace somanet
{
struct CliOptions
{
  bool autostart{false};
};

inline CliOptions parse_cli_arguments(int argc, char * argv[], std::vector<char *> & filtered)
{
  if (argc < 0) {
    throw std::invalid_argument("argc cannot be negative");
  }

  CliOptions options;
  bool autostart_set = false;

  filtered.clear();
  filtered.reserve(static_cast<size_t>(argc) + 1);
  if (argc > 0) {
    filtered.push_back(argv[0]);
  }

  for (int i = 1; i < argc; ++i) {
    const bool is_autostart = std::strcmp(argv[i], "--autostart") == 0;
    const bool is_no_autostart = std::strcmp(argv[i], "--no-autostart") == 0;

    if (is_autostart || is_no_autostart) {
      const bool requested_autostart = is_autostart;
      if (autostart_set && options.autostart != requested_autostart) {
        throw std::invalid_argument(
          "Conflicting autostart flags detected (both --autostart and --no-autostart)");
      }

      options.autostart = requested_autostart;
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
}  // namespace somanet

