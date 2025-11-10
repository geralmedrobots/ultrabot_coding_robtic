#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "maintenance_mode_manager.hpp"

namespace
{
constexpr const char * kValidPinHash =
  "03ac674216f3e15c761ee1a5e255f067953623c8b388b4459e13f978d7c846f4";  // SHA-256("1234")
}

class MaintenanceModeSafetyTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    const auto unique_suffix = std::to_string(std::chrono::steady_clock::now().time_since_epoch().count());
    temp_dir_ = std::filesystem::temp_directory_path() / ("ultrabot_maintenance_test_" + unique_suffix);
    std::filesystem::create_directories(temp_dir_);

    audit_log_path_ = temp_dir_ / "audit.yaml";
    pin_file_path_ = temp_dir_ / "pins.yaml";

    std::ofstream pin_file(pin_file_path_);
    pin_file << "pins:\n";
    pin_file << "  - hash: \"" << kValidPinHash << "\"\n";
    pin_file.close();
  }

  void TearDown() override
  {
    std::error_code ec;
    std::filesystem::remove_all(temp_dir_, ec);
  }

  MaintenanceModeManager createManager(const std::string & logger_name)
  {
    return MaintenanceModeManager(
      audit_log_path_.string(), pin_file_path_.string(), rclcpp::get_logger(logger_name));
  }

  std::string readAuditLog() const
  {
    std::ifstream audit_file(audit_log_path_);
    if (!audit_file.is_open()) {
      return {};
    }
    std::stringstream buffer;
    buffer << audit_file.rdbuf();
    return buffer.str();
  }

  std::filesystem::path temp_dir_;
  std::filesystem::path audit_log_path_;
  std::filesystem::path pin_file_path_;
};

TEST_F(MaintenanceModeSafetyTest, InvalidPinDeniesMaintenanceAccess)
{
  auto manager = createManager("invalid_pin_test");

  EXPECT_FALSE(manager.enterMaintenanceMode("0000", "intruder", "Unauthorized access"));
  EXPECT_FALSE(manager.isInMaintenanceMode());

  const auto audit_contents = readAuditLog();
  EXPECT_NE(std::string::npos, audit_contents.find("FAILED AUTH"));
  EXPECT_NE(std::string::npos, audit_contents.find("Unauthorized access"));
}

TEST_F(MaintenanceModeSafetyTest, ValidSessionRequiresAuditApproval)
{
  auto manager = createManager("valid_session_test");

  ASSERT_TRUE(manager.enterMaintenanceMode("1234", "tech_ops", "Adjust velocity limits"));
  ASSERT_TRUE(manager.isInMaintenanceMode());

  manager.logParameterChange("max_linear_vel", "0.8", "0.6");

  const auto * session = manager.getCurrentSession();
  ASSERT_NE(nullptr, session);
  ASSERT_EQ(1u, session->changes_made.size());
  EXPECT_NE(std::string::npos, session->changes_made.front().find("max_linear_vel"));

  EXPECT_TRUE(manager.exitMaintenanceMode());
  EXPECT_FALSE(manager.isInMaintenanceMode());

  const auto audit_contents = readAuditLog();
  EXPECT_NE(std::string::npos, audit_contents.find("tech_ops"));
  EXPECT_NE(std::string::npos, audit_contents.find("Adjust velocity limits"));
  EXPECT_NE(std::string::npos, audit_contents.find("max_linear_vel"));
}

TEST_F(MaintenanceModeSafetyTest, ParameterChangesIgnoredWhenNotInMaintenance)
{
  auto manager = createManager("no_maintenance_change");

  manager.logParameterChange("max_angular_vel", "1.0", "1.5");
  EXPECT_FALSE(manager.isInMaintenanceMode());
  EXPECT_EQ(nullptr, manager.getCurrentSession());

  ASSERT_TRUE(manager.enterMaintenanceMode("1234", "auditor", "Test"));
  manager.logParameterChange("max_angular_vel", "1.0", "0.9");

  const auto * session = manager.getCurrentSession();
  ASSERT_NE(nullptr, session);
  EXPECT_EQ(1u, session->changes_made.size());
}
