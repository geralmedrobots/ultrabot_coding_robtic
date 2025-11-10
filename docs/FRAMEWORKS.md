# Ultrabot ROS 2 Frameworks & Libraries

**Version:** 0.2+ | **ROS Distribution:** Humble | **Ubuntu:** 22.04 LTS

---

## Table of Contents

- [Overview](#overview)
- [Framework Categories](#framework-categories)
  - [🧭 Navigation (v0.2)](#-navigation-v02)
  - [🗺️ Mapping/Localization (v0.2)](#️-mappinglocalization-v02)
  - [👁️ Perception (v0.2)](#-perception-v02)
  - [🛡️ Safety/Diagnostics (v0.2)](#️-safetydiagnostics-v02)
  - [🤝 Fleet Management (v0.3)](#-fleet-management-v03)
  - [📈 Sensor Fusion/Filters (v0.3)](#-sensor-fusionfilters-v03)
  - [🧰 Simulation (v0.3)](#-simulation-v03)
  - [🔐 Cybersecurity IEC 62443 (v0.3)](#-cybersecurity-iec-62443-v03)
  - [📜 Documentation & QA (Continuous)](#-documentation--qa-continuous)
- [Version Compatibility Matrix](#version-compatibility-matrix)
- [Installation Quick Reference](#installation-quick-reference)
- [Related Documentation](#related-documentation)

---

## Overview

This document outlines the recommended ROS 2 frameworks and libraries for the Ultrabot project evolution from v0.2 onwards. Each framework is categorized by functionality and tagged with the version milestone when it becomes necessary.

**Key Principles:**
- All packages are compatible with **ROS 2 Humble** on **Ubuntu 22.04 LTS**
- Frameworks are introduced progressively based on project maturity
- Safety and diagnostics are prioritized from v0.2
- Advanced features (fleet management, simulation) are introduced in v0.3

---

## Framework Categories

### 🧭 Navigation (v0.2)

Essential autonomous navigation stack for path planning and control.

| Package | Purpose | Installation |
|---------|---------|--------------|
| `ros-humble-nav2-bringup` | Complete Nav2 launch files and configuration | See below |
| `ros-humble-nav2-core` | Core Nav2 interfaces and base classes | See below |
| `ros-humble-nav2-util` | Common utilities for Nav2 components | See below |

**Purpose:** Provides path planning, behavior trees, waypoint following, and autonomous navigation control for hospital environments.

**Installation:**
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-core \
  ros-humble-nav2-util
```

**Official Documentation:**
- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 GitHub](https://github.com/ros-planning/navigation2)

**Version:** Nav2 1.1.x (ROS 2 Humble)

**Use Cases:**
- Autonomous navigation between hospital rooms
- Dynamic obstacle avoidance
- Path replanning with cost functions
- Integration with hospital maps

---

### 🗺️ Mapping/Localization (v0.2)

SLAM and localization packages for creating and using 2D maps.

| Package | Purpose | Installation |
|---------|---------|--------------|
| `ros-humble-slam-toolbox` | 2D SLAM with loop closure and map serialization | See below |
| `ros-humble-amcl` | Adaptive Monte Carlo Localization for map-based positioning | See below |

**Purpose:** Creates accurate 2D maps of hospital environments and localizes the robot within those maps using laser scan data.

**Installation:**
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-slam-toolbox \
  ros-humble-amcl
```

**Official Documentation:**
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [AMCL Documentation](https://navigation.ros.org/configuration/packages/configuring-amcl.html)

**Version:**
- SLAM Toolbox: 2.6.x
- AMCL: Part of Nav2 1.1.x

**Use Cases:**
- Initial hospital floor mapping
- Real-time localization during operation
- Map updates for dynamic environments
- Multi-floor navigation

---

### 👁️ Perception (v0.2)

RGB-D and 3D LiDAR sensing for obstacle detection, docking alignment, and SLAM inputs.

| Package | Purpose | Installation |
|---------|---------|--------------|
| `ros-humble-realsense2-camera` | Intel RealSense depth + RGB driver | See below |
| `ros-humble-ros2-ouster` | Ouster OS-series LiDAR driver | See below |

**Purpose:** Provides dense depth, RGB imagery, and 3D point clouds required for safe navigation in cluttered hospital corridors
and elevators.

**Installation:**
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-realsense2-camera \
  ros-humble-ros2-ouster
```

**Official Documentation:**
- [Intel RealSense ROS 2](https://github.com/IntelRealSense/realsense-ros)
- [Ouster ROS 2 Driver](https://github.com/ros-drivers/ros2_ouster)

**Use Cases:**
- Depth-assisted docking at charging stations
- 3D obstacle detection and SLAM map refinement
- Telepresence and situational awareness for operators
- Semantic perception prototypes using RGB-D data

**Prerequisites:**
- Calibration captured per robot for both sensors (see `docs/REMAINING_TASKS.md`)
- Time synchronization between sensors and the main controller for accurate fusion

---

### 🛡️ Safety/Diagnostics (v0.2)

Standardized diagnostics for safety compliance and system health monitoring.

| Package | Purpose | Installation |
|---------|---------|--------------|
| `ros-humble-diagnostic-updater` | Standardized diagnostics interface | See below |
| `ros-humble-diagnostic-aggregator` | Aggregate diagnostics from multiple sources | See below |

**Purpose:** Provides standardized diagnostic reporting required for CE marking and FDA compliance. Essential for safety-critical medical robotics.

**Installation:**
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-diagnostic-updater \
  ros-humble-diagnostic-aggregator
```

**Official Documentation:**
- [Diagnostics Documentation](https://github.com/ros/diagnostics)
- [diagnostic_updater Tutorial](https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html)

**Version:** ROS 2 Humble diagnostics stack

**Use Cases:**
- Real-time system health monitoring
- Compliance with ISO 13849-1 diagnostics requirements
- Battery status monitoring
- Motor controller diagnostics
- Emergency stop circuit verification
- Audit trail for certification

**Compliance Standards:**
- ISO 13849-1:2023 (Safety of machinery)
- ISO 14971:2019 (Medical device risk management)
- IEC 62304:2006 (Medical device software lifecycle)

---

### 🤝 Fleet Management (v0.3)

Multi-robot coordination for hospital fleet operations.

| Package | Purpose | Installation |
|---------|---------|--------------|
| `ros-humble-rmf-core` | RMF core interfaces and messages | See below |
| `ros-humble-rmf-fleet-adapter` | Fleet adapter for robot integration | See below |

**Purpose:** Coordinates multiple Ultrabot robots in hospital environments, managing task allocation, traffic flow, and shared resources (elevators, doors).

**Installation:**
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-rmf-core \
  ros-humble-rmf-fleet-adapter
```

**Official Documentation:**
- [Open-RMF Documentation](https://osrf.github.io/ros2multirobotbook/)
- [RMF GitHub](https://github.com/open-rmf/rmf)

**Version:** RMF 21.09+ (Humble compatible)

**Use Cases:**
- Multi-robot task scheduling
- Elevator and door coordination
- Traffic management in narrow corridors
- Centralized fleet monitoring
- Resource conflict resolution

**Prerequisites:**
- Nav2 navigation stack (v0.2)
- Map server with hospital layouts
- Building system integration (doors, elevators)

---

### 📈 Sensor Fusion/Filters (v0.3)

Advanced sensor fusion for improved localization accuracy.

| Package | Purpose | Installation |
|---------|---------|--------------|
| `ros-humble-robot-localization` | EKF/UKF sensor fusion (IMU + odometry) | See below |

**Purpose:** Fuses data from multiple sensors (IMU, wheel odometry, visual odometry) to provide robust state estimation in challenging hospital environments.

**Installation:**
```bash
sudo apt-get update
sudo apt-get install -y ros-humble-robot-localization
```

**Official Documentation:**
- [robot_localization Documentation](https://docs.ros.org/en/humble/p/robot_localization/)
- [robot_localization GitHub](https://github.com/cra-ros-pkg/robot_localization)

**Version:** 3.4.x (Humble)

**Use Cases:**
- Fusion of IMU and wheel odometry
- Improved localization accuracy
- Handling of slippery floors (common in hospitals)
- Recovery from temporary AMCL failures
- Smooth odometry estimates

**Configuration Notes:**
- Configure EKF for `odom->base_link` transformation
- Fuse wheel odometry (x, y, yaw) with IMU (angular velocity, linear acceleration)
- Tune covariance matrices based on sensor characteristics

---

### 🧰 Simulation (v0.3)

Virtual testing environment for safe development and validation.

| Package | Purpose | Installation |
|---------|---------|--------------|
| `ros-humble-gazebo-ros-pkgs` | Gazebo integration with ROS 2 | See below |
| `ros-humble-nav2-bringup` | Nav2 simulation configurations | Already installed in v0.2 |

**Purpose:** Enables virtual testing of navigation algorithms, safety features, and fleet coordination without physical hardware.

**Installation:**
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-gazebo-ros-pkgs \
  gazebo
```

**Official Documentation:**
- [Gazebo Documentation](https://gazebosim.org/docs)
- [Gazebo ROS 2 Integration](https://github.com/ros-simulation/gazebo_ros_pkgs)

**Version:** Gazebo 11 with ROS 2 Humble

**Use Cases:**
- Algorithm development without hardware
- Safety feature validation
- Multi-robot scenario testing
- Training neural networks with simulated data
- Regression testing in CI/CD pipelines

**Simulation Scenarios:**
- Hospital corridor navigation
- Elevator boarding
- Multi-robot coordination
- Emergency stop scenarios
- Sensor failure handling

---

### 🔐 Cybersecurity IEC 62443 (v0.3)

Secure communication for medical device compliance.

| Package | Purpose | Installation |
|---------|---------|--------------|
| `ros-humble-sros2` | ROS 2 security (SROS2) with DDS encryption | See below |
| `ros-humble-fastdds` | Fast DDS with security support | See below |

**Purpose:** Implements encrypted DDS communication and node authentication required for IEC 62443 compliance in medical environments.

**Installation:**
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-sros2 \
  ros-humble-fastrtps \
  ros-humble-rmw-fastrtps-cpp
```

**Official Documentation:**
- [SROS2 Documentation](https://design.ros2.org/articles/ros2_dds_security.html)
- [Fast DDS Security](https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/security.html)

**Version:** ROS 2 Humble SROS2

**Use Cases:**
- Encrypted topic communication (patient data)
- Node authentication and authorization
- Prevention of unauthorized robot control
- Secure parameter updates
- Audit logging of security events

**Compliance Standards:**
- IEC 62443-3-3 (System security requirements)
- IEC 62443-4-2 (Technical security requirements)
- FDA Cybersecurity Guidance (2023)

**Setup Notes:**
- Generate security certificates for each node
- Configure security policies (allow/deny rules)
- Enable DomainParticipant authentication
- See Navigation/SROS2_GUIDE.md for complete setup

---

### 📜 Documentation & QA (Continuous)

Code quality and documentation tools for maintainability.

| Package | Purpose | Installation |
|---------|---------|--------------|
| `clang-format` | C++ code formatting | See below |
| `cppcheck` | Static analysis for C/C++ | See below |
| `ros-humble-ament-lint` | ROS 2 linting tools (Python, XML, CMake) | See below |

**Purpose:** Maintains code quality, consistency, and documentation standards throughout the project lifecycle.

**Installation:**
```bash
sudo apt-get update
sudo apt-get install -y \
  clang-format \
  cppcheck \
  ros-humble-ament-lint \
  ros-humble-ament-lint-common \
  ros-humble-ament-cmake-clang-format \
  ros-humble-ament-cmake-cppcheck
```

**Official Documentation:**
- [clang-format Documentation](https://clang.llvm.org/docs/ClangFormat.html)
- [cppcheck Manual](https://cppcheck.sourceforge.io/)
- [ament_lint](https://github.com/ament/ament_lint)

**Version:**
- clang-format: 14.x (Ubuntu 22.04)
- cppcheck: 2.7+
- ament_lint: Humble

**Use Cases:**
- Automated code formatting in CI/CD
- Static analysis for bug detection
- Python PEP8 compliance
- XML schema validation
- CMake linting
- Documentation generation

**Integration:**
```bash
# Run in package directory
colcon test --packages-select somanet \
  --event-handlers console_direct+ \
  --pytest-args -v

# Format code
clang-format -i src/**/*.cpp include/**/*.hpp

# Static analysis
cppcheck --enable=all --inconclusive src/
```

---

## Version Compatibility Matrix

| Package | Ubuntu 22.04 | ROS 2 Humble | v0.2 | v0.3 | Continuous |
|---------|--------------|--------------|------|------|------------|
| nav2-bringup | ✅ | ✅ | ✅ | ✅ | ✅ |
| nav2-core | ✅ | ✅ | ✅ | ✅ | ✅ |
| nav2-util | ✅ | ✅ | ✅ | ✅ | ✅ |
| slam-toolbox | ✅ | ✅ | ✅ | ✅ | ✅ |
| amcl | ✅ | ✅ | ✅ | ✅ | ✅ |
| diagnostic-updater | ✅ | ✅ | ✅ | ✅ | ✅ |
| diagnostic-aggregator | ✅ | ✅ | ✅ | ✅ | ✅ |
| rmf-core | ✅ | ✅ | - | ✅ | ✅ |
| rmf-fleet-adapter | ✅ | ✅ | - | ✅ | ✅ |
| robot-localization | ✅ | ✅ | - | ✅ | ✅ |
| gazebo-ros-pkgs | ✅ | ✅ | - | ✅ | ✅ |
| sros2 | ✅ | ✅ | - | ✅ | ✅ |
| fastdds | ✅ | ✅ | - | ✅ | ✅ |
| clang-format | ✅ | N/A | ✅ | ✅ | ✅ |
| cppcheck | ✅ | N/A | ✅ | ✅ | ✅ |
| ament-lint | ✅ | ✅ | ✅ | ✅ | ✅ |

---

## Installation Quick Reference

### All v0.2 Packages
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-core \
  ros-humble-nav2-util \
  ros-humble-slam-toolbox \
  ros-humble-amcl \
  ros-humble-diagnostic-updater \
  ros-humble-diagnostic-aggregator \
  clang-format \
  cppcheck \
  ros-humble-ament-lint \
  ros-humble-ament-lint-common
```

### All v0.3 Packages
```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-rmf-core \
  ros-humble-rmf-fleet-adapter \
  ros-humble-robot-localization \
  ros-humble-gazebo-ros-pkgs \
  gazebo \
  ros-humble-sros2 \
  ros-humble-fastrtps \
  ros-humble-rmw-fastrtps-cpp
```

---

## Related Documentation

- **[ROADMAP.md](ROADMAP.md)** - Version-based roadmap with implementation priorities
- **[INSTALLATION_FRAMEWORKS.md](INSTALLATION_FRAMEWORKS.md)** - Detailed installation guide with troubleshooting
- **[FRAMEWORKS_QUICK_REFERENCE.md](FRAMEWORKS_QUICK_REFERENCE.md)** - One-page cheat sheet
- **[Navigation/SROS2_GUIDE.md](../Navigation/SROS2_GUIDE.md)** - Complete SROS2 security setup
- **[Navigation/SAFETY.md](../Navigation/SAFETY.md)** - Safety procedures and compliance

---

## Notes

**System Requirements:**
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- ROS 2 Humble Hawksbill
- Minimum 4GB RAM (8GB recommended for simulation)
- 20GB free disk space

**Update Frequency:**
- This document is updated with each minor version release
- Check for updates before starting new development phases
- Report missing or deprecated packages via issue tracker

**Support:**
- For package-specific issues, consult official documentation
- For Ultrabot integration questions, see Navigation/README.md
- For safety-critical concerns, contact Safety Officer (see SAFETY.md)

---

**Last Updated:** 2025-11-05 | **Version:** 1.0.0 | **Maintainer:** Ultrabot Team
