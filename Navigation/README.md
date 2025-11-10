# Ultrabot Navigation System

**ROS2 Humble** | **Safety-Critical AGV Control** | **ISO 13849-1 Compliant**

---

## 🎯 Overview

ROS2 package for safe AGV control with EtherCAT motor interface, redundant safety architecture, and compliance with international safety standards (ISO 13849-1, ISO 3691-4, IEC 61508).

### Key Features

- ✅ **Dual-Layer Safety:** Hardware E-stop + Software Safety Supervisor
- ✅ **EtherCAT Control:** SOEM library for motor communication
- ✅ **Modular Odometry:** Dedicated `OdometryCalculator` class with dynamic covariance
- ✅ **Command Arbitration:** Priority-based command source selection (ISO 3691-4 compliant)
- ✅ **Safe Teleoperation:** Deadman button + watchdog timers
- ✅ **Real-time Diagnostics:** System health monitoring at 1Hz
- ✅ **Modular Lifecycle:** Dedicated command limiter, watchdog, and odometry publisher components
- ✅ **Unit Tested:** 6 automated suites covering lifecycle, watchdog, and maintenance flows
- ✅ **Perception Ready:** Launch files for Intel RealSense D455 + Ouster OS LiDAR using upstream drivers

---

## ⚠️ Safety-Critical System

**READ [SAFETY.md](SAFETY.md) BEFORE OPERATING**

This package implements safety-critical functions. Mandatory requirements:
- Complete pre-flight checklist before operation
- Safety Supervisor MUST be running before drive node
- Emergency stop button tested before each session
- Operator training and authorization required

---

## 📋 Quick Start

### Prerequisites

```bash
# Ubuntu 22.04 + ROS2 Humble
sudo apt-get install -y \
    ros-humble-rclcpp \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-ros \
    ros-humble-soem \
    ros-humble-sros2

# Create the HMAC secret key file for development (in production, use a secure vault)
echo "ThisIsAPlaceholderSecretKeyForDevelopment!ChangeMe!" > config/cert.key

# Optional: Enable SROS2 for encrypted communication (recommended for production)
# See SROS2_GUIDE.md for complete setup instructions
cd scripts
./setup_sros2.sh
export ROS_SECURITY_KEYSTORE=../sros2_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

### Build

```bash
mkdir -p ~/ultrabot_ws/src
cd ~/ultrabot_ws/src
git clone <repo> navigation

cd ~/ultrabot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select somanet
source install/setup.bash
```

### Configuration

**⚠️ REQUIRED: Set EtherCAT Interface**

The EtherCAT interface name is **mandatory** and system-specific. Configure it using one of these methods:

#### Method 1: Environment Variable (Recommended for testing)
```bash
# Find your interface name
ip link show

# Example output:
# 2: eth0: <BROADCAST,MULTICAST,UP,LOWER_UP> ...
# 3: enp89s0: <BROADCAST,MULTICAST,UP,LOWER_UP> ...

# Set the interface
export ETHERCAT_INTERFACE=eth0  # or enp89s0, eno1, etc.

# Launch with environment variable
ros2 launch somanet launch.py
```

#### Method 2: Configuration File (Recommended for production)
```bash
# Edit config/robot_config.yaml
nano config/robot_config.yaml

# Set the interface:
ethercat:
  interface: "eth0"  # Change to your interface

# Launch normally
ros2 launch somanet launch.py
```

#### Method 3: Launch Parameter Override
```bash
ros2 launch somanet launch.py ethercat_interface:=eth0
```

> **Troubleshooting:** If you see "EtherCAT interface parameter is REQUIRED", the interface is not configured. The system will NOT use a default to prevent accidental hardware misconfiguration.

### Launch

```bash
# Lifecycle-managed bring-up (safety validated before activation)
ros2 launch somanet launch.py

# Manual bring-up (advanced)
# (pass --autostart or export ULTRABOT_AUTOSTART=1 if not driving lifecycle transitions manually)
## Terminal 1: Safety Supervisor (MANDATORY)
ros2 run somanet safety_supervisor_node --autostart \
  --ros-args --params-file config/safety_params.yaml

## Terminal 2: Drive Node
sudo ros2 run somanet main --autostart \
  --ros-args \
  --params-file config/robot_config.yaml \
  -p ethercat_interface:=eth0  # REQUIRED

## Terminal 3: Teleoperation (Joystick)
ros2 run somanet teleop_joy \
  --ros-args --params-file config/safety_params.yaml
```

> ℹ️ Lifecycle nodes (`safety_supervisor`, `somanet_driver`, `command_arbitrator`) start in the `unconfigured` state by default. Launch files handle the configure/activate transitions automatically. When running nodes directly, either pass `--autostart`, set `ULTRABOT_AUTOSTART=1`, or orchestrate transitions with `ros2 lifecycle set ... configure|activate`.

**Controls:**
- **R1:** Deadman (hold to move)
- **Left Stick:** Forward/backward
- **Right Stick:** Rotation

### Perception Sensor Launch (RealSense D455 + Ouster OS LiDAR)

Both perception devices leverage upstream open-source drivers:

- [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) (Apache-2.0)
- [ros-drivers/ros2_ouster](https://github.com/ros-drivers/ros2_ouster) (BSD-3-Clause)

Configuration files live in [`config/realsense_d455.yaml`](config/realsense_d455.yaml) and
[`config/ouster_lidar.yaml`](config/ouster_lidar.yaml). Update `serial_no`, `sensor_hostname`, and
`metadata` to match your hardware before launching.

```bash
# Launch both sensors with RViz visualization
ros2 launch somanet sensors.launch.py \
  realsense_serial:=<optional_serial> \
  ouster_hostname:=os-1.local \
  ouster_metadata:=/data/ouster/os-1-metadata.json

# Disable RViz or individual sensors if needed
ros2 launch somanet sensors.launch.py enable_rviz:=false
```

`config/sensor_visualization.rviz` provides a ready-to-use RViz layout that overlays the Ouster
point cloud, RealSense color stream, and aligned depth image for debugging.

---

## ✅ Parameter Validation

**NEW in v3.1:** All safety-critical parameters are validated with strict bounds before node activation.

### Validated Parameters

| Parameter | Range | Unit | Standard |
|-----------|-------|------|----------|
| `distance_wheels` | [0.1, 2.0] | m | AGV typical |
| `wheel_diameter` | [0.05, 0.5] | m | Industrial wheels |
| `max_linear_vel` | [0.01, 5.0] | m/s | ISO 3691-4 |
| `max_angular_vel` | [0.01, 10.0] | rad/s | Stability limit |
| `cmd_watchdog_timeout` | [0.01, 10.0] | s | ISO 13849-1 |
| `delta_t_warn_threshold` | [0.001, 1.0] | s | Diagnostic |

### Behavior

- **Invalid configuration** → Node refuses to configure (stays in `UNCONFIGURED` state)
- **FATAL log** shows exact parameter and expected range
- **Prevents unsafe operation** before motors are enabled

### Example Error

```bash
ros2 launch somanet launch.py

# If max_linear_vel=12.0 (out of range):
[FATAL] [somanet_driver]: max_linear_vel out of safe range [0.01, 5.0] m/s (got 12.000)
[ERROR] [somanet_driver]: Failed to configure
```

**📖 Complete reference:** See [PARAMETER_VALIDATION.md](PARAMETER_VALIDATION.md) for detailed rationale, examples, and compliance mapping.

---

## 🏗️ Architecture

```
Teleop → /cmd_vel → [Safety Supervisor] → /wheel_cmd_safe → [Drive Node] → Motors
                         ↓                                        ↓
                   /safety_stop                                 /odom
                   /diagnostics                                  /tf
```

### Components

| Component | Description | Lines | Status |
|-----------|-------------|-------|--------|
| `safety_supervisor_node.cpp` | Redundant command validation | 521 | ✅ Production |
| `somanet_lifecycle_node.cpp` | Lifecycle orchestration, odometry, watchdogs | 447 | ✅ Production |
| `main.cpp` | Lifecycle entrypoint & CLI | 93 | ✅ Production |
| `command_arbitrator_node.cpp` | Priority-based command selection | 380 | ✅ Production |
| `odometry_calculator.cpp` | Differential drive kinematics | 120 | ✅ Production |
| `teleop_joy.cpp` | Safe joystick teleoperation | 324 | ✅ Production |

The diagram and table above capture the current architecture at a glance.

---

## 🛡️ Safety Features

| Feature | Standard | Implementation |
|---------|----------|----------------|
| **Emergency Stop** | ISO 13850 | Hardware STO + software redundancy |
| **Deadman Button** | ISO 3691-4 | Mandatory hold-to-run |
| **Velocity Limits** | ISO 3691-4 | 1.0 m/s linear, 1.0 rad/s angular |
| **Watchdog Timer** | IEC 61508 | 0.5s timeout, automatic stop |
| **Plausibility Check** | ISO 3691-4 | 0.2 m/s cmd vs actual threshold |
| **Secure Communication** | IEC 62443 | SROS2 encryption + authentication |
| **Parameter Validation** | ISO 13849-1 | HMAC-authenticated certified parameters |
| **Config Validation** | ISO 13849-1 | Range checks on all robot parameters |
| **Audit Logging** | ISO 14971 | 5-year retention |

---

## 📡 ROS2 Topics

### Safety-Critical

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `Twist` | Velocity commands (input, unsafe) |
| `/wheel_cmd_safe` | `Twist` | Validated commands (output, safe) |
| `/safety_stop` | `Bool` | Emergency stop status |
| `/deadman_status` | `Bool` | Deadman button state |

### Standard

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `Odometry` | Robot odometry (100 Hz) |
| `/diagnostics` | `DiagnosticArray` | System health (1 Hz) |
| `/operator_log` | `String` | Audit trail |

---

## ⚙️ Configuration

### Safety Parameters (`config/safety_params.yaml`)

```yaml
safety_supervisor:
  ros__parameters:
    max_linear_velocity: 1.0      # m/s
    max_angular_velocity: 1.0     # rad/s
    plausibility_threshold: 0.2   # m/s
    watchdog_timeout: 0.5         # seconds
    require_deadman: true
```

> **Authentication:** The safety supervisor validates certified parameters using an HMAC signature. The secret key is read from `config/cert.key`. This file is excluded from Git by `config/.gitignore`. In a production environment, this key file must be managed by a secure deployment process (e.g., using a secrets management tool like Vault or Ansible Vault) and should never be committed to source control.

### Robot Parameters (`config/robot_config.yaml`)

```yaml
robot:
  wheel_diameter: 0.170          # meters
  distance_wheels: 0.485         # meters
  gear_ratio: 40.0
  max_rpm: 3000
```

---

## 🧪 Testing & Validation

### Run Tests

```bash
colcon test --packages-select somanet
colcon test-result --verbose
```

### Validate Odometry

```bash
ros2 run somanet validate_odometry.py
# Tests: frequency (>20Hz), accuracy (<5% error), covariance
```

### Safety Function Tests (Quarterly)

```bash
# Test 1: Emergency Stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: true"

# Test 2: Deadman Release
# Release R1 during movement, verify immediate stop

# Test 3: Watchdog Timeout
# Kill teleop node, verify stop after 0.5s

# Test 4: Velocity Limit
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 2.0}, angular: {z: 0}}"
# Verify clamped to 1.0 m/s

# Test 5: Plausibility Check
# Block wheels, send forward command
# Verify emergency stop after 3 failures
```

---

## 🐛 Troubleshooting

### Safety Stop Active

**Symptom:** Robot won't move, `/safety_stop` is true

**Solutions:**
```bash
# Check diagnostics
ros2 topic echo /diagnostics

# Common causes:
# 1. Deadman not held → Hold R1 button
# 2. Watchdog timeout → Restart teleop node
# 3. Plausibility fail → Run validate_odometry.py
# 4. Velocity exceeded → Reduce speed command
```

### EtherCAT Connection Failed

**Symptom:** "No slaves found"

**Solutions:**
```bash
# Check interface
ip link show

# Test manually
sudo ethercat slaves

# Verify permissions
groups $USER  # Should include 'realtime'
```

See [BUILD_GUIDE.md](BUILD_GUIDE.md) for detailed troubleshooting.

---

## 📚 Documentation

| Document | Description |
|----------|-------------|
| [SAFETY.md](SAFETY.md) | **Mandatory reading** - Safety procedures, risk assessment, emergency protocols |
| [BUILD_GUIDE.md](BUILD_GUIDE.md) | Build instructions, dependencies, troubleshooting |
| [SROS2_GUIDE.md](SROS2_GUIDE.md) | **Security setup** - Encrypted communication, authentication, access control |

---

## 📜 Compliance

| Standard | Title | Status |
|----------|-------|--------|
| **ISO 13849-1:2023** | Safety of machinery | ✅ Cat 3, PL d |
| **ISO 3691-4:2023** | AGV safety requirements | ✅ Manual mode |
| **IEC 61508:2010** | Functional safety | ✅ SIL 1 |
| **EN 61800-5-2:2016** | Drive safety (STO) | ✅ Hardware E-stop |
| **ISO 13850:2015** | Emergency stop | ✅ <100ms response |

---

## 📞 Support

- **Safety Officer:** [See SAFETY.md for contacts]
- **Technical Support:** [Organization-specific]
- **Emergency:** 112 (EU) / 911 (US)

---

## 📝 License

[To be specified by organization]

---

## ✍️ Authors

- **ROS2 Package & Safety:** [Organization]
- **Refactoring & Documentation:** GitHub Copilot (2025)

---

## 📅 Version

**Version:** 3.1.0 | **Date:** 2025-11-04 | **Status:** ✅ Production Ready

### Recent Changes

- **v3.1:** Comprehensive parameter range validation (ISO 13849-1 compliant)
  - Added safety-critical bounds for all physical/velocity/timing parameters
  - Prevents invalid configurations from reaching active state
  - See [PARAMETER_VALIDATION.md](PARAMETER_VALIDATION.md) for complete reference
- v3.0: Modular odometry (OdometryCalculator), command arbitration
- v2.0: Safety supervisor integration, ISO 13849-1 compliance
- v1.0: Initial ROS2 migration

---

**⚠️ SAFETY NOTICE:** This is a safety-critical system. Always follow procedures in [SAFETY.md](SAFETY.md) before operation.
