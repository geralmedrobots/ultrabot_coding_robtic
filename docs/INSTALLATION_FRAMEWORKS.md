# Ultrabot Frameworks Installation Guide

**Target Platform:** Ubuntu 22.04 LTS | **ROS Distribution:** Humble Hawksbill

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [System Requirements](#system-requirements)
- [Installation by Phase](#installation-by-phase)
  - [v0.2: Foundation Frameworks](#v02-foundation-frameworks)
  - [v0.3: Advanced Frameworks](#v03-advanced-frameworks)
- [Verification Steps](#verification-steps)
- [Troubleshooting](#troubleshooting)
- [Docker Installation](#docker-installation)
- [Development Environment Setup](#development-environment-setup)

---

## Prerequisites

### 1. Ubuntu 22.04 LTS Installation

Ensure you have a fresh Ubuntu 22.04 LTS installation:

```bash
# Verify Ubuntu version
lsb_release -a

# Expected output:
# Description:    Ubuntu 22.04.x LTS
# Release:        22.04
# Codename:       jammy
```

### 2. ROS 2 Humble Installation

If ROS 2 Humble is not already installed:

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

**Verify Installation:**
```bash
ros2 --version
# Expected: ros2 cli version 0.18.x or higher
```

### 3. Workspace Setup

```bash
# Create workspace
mkdir -p ~/ultrabot_ws/src
cd ~/ultrabot_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Initialize colcon workspace
colcon build --symlink-install
source install/setup.bash
```

---

## System Requirements

### Minimum Hardware
- **CPU:** Intel Core i5 (4 cores) or equivalent
- **RAM:** 8 GB (16 GB recommended for simulation)
- **Storage:** 50 GB free space (100 GB recommended)
- **Network:** 100 Mbps Ethernet or WiFi 5

### Recommended Hardware
- **CPU:** Intel Core i7 (8 cores) or AMD Ryzen 7
- **RAM:** 16 GB DDR4
- **Storage:** 256 GB SSD
- **GPU:** NVIDIA GTX 1060 or higher (for Gazebo simulation)
- **Network:** Gigabit Ethernet or WiFi 6

### Software Requirements
- Ubuntu 22.04 LTS (kernel 5.15+)
- ROS 2 Humble Hawksbill
- Python 3.10+
- CMake 3.22+
- GCC 11+ or Clang 14+

---

## Installation by Phase

### v0.2: Foundation Frameworks

Install all frameworks required for v0.2 milestone.

#### 1. Navigation Stack (Nav2)

```bash
# Install Nav2 packages
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-core \
  ros-humble-nav2-util \
  ros-humble-nav2-bt-navigator \
  ros-humble-nav2-controller \
  ros-humble-nav2-planner \
  ros-humble-nav2-lifecycle-manager

# Verify installation
ros2 pkg list | grep nav2

# Expected packages:
# nav2_amcl
# nav2_behavior_tree
# nav2_bringup
# nav2_bt_navigator
# nav2_controller
# nav2_core
# ... (20+ packages)
```

**Configuration:**
```bash
# Create nav2 config directory
mkdir -p ~/ultrabot_ws/src/navigation/config/nav2

# Copy default params (customize later)
cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml \
   ~/ultrabot_ws/src/navigation/config/nav2/
```

---

#### 2. SLAM & Localization

```bash
# Install SLAM Toolbox
sudo apt install -y ros-humble-slam-toolbox

# Install AMCL
sudo apt install -y ros-humble-amcl

# Install map server
sudo apt install -y ros-humble-nav2-map-server

# Verify installation
ros2 pkg list | grep -E "(slam|amcl|map_server)"

# Expected output:
# nav2_amcl
# slam_toolbox
# nav2_map_server
```

**Quick Test:**
```bash
# Test SLAM Toolbox (with dummy data)
ros2 launch slam_toolbox online_async_launch.py

# In another terminal, check if node is running
ros2 node list
# Expected: /slam_toolbox
```

---

#### 3. Safety & Diagnostics

```bash
# Install diagnostic tools
sudo apt install -y \
  ros-humble-diagnostic-updater \
  ros-humble-diagnostic-aggregator \
  ros-humble-diagnostic-msgs \
  ros-humble-diagnostic-common-diagnostics

# Install rqt diagnostic viewer
sudo apt install -y ros-humble-rqt-robot-monitor

# Verify installation
ros2 pkg list | grep diagnostic

# Expected output:
# diagnostic_aggregator
# diagnostic_common_diagnostics
# diagnostic_msgs
# diagnostic_updater
```

**Quick Test:**
```bash
# Run diagnostic aggregator
ros2 run diagnostic_aggregator aggregator_node

# In another terminal, view diagnostics
ros2 topic echo /diagnostics
```

---

#### 4. Code Quality Tools

```bash
# Install clang-format
sudo apt install -y clang-format-14

# Create symlink for convenience
sudo update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-14 100

# Install cppcheck
sudo apt install -y cppcheck

# Install ament lint tools
sudo apt install -y \
  ros-humble-ament-lint \
  ros-humble-ament-lint-common \
  ros-humble-ament-cmake-clang-format \
  ros-humble-ament-cmake-cppcheck \
  ros-humble-ament-cmake-cpplint \
  ros-humble-ament-cmake-flake8 \
  ros-humble-ament-cmake-pep257 \
  ros-humble-ament-cmake-xmllint

# Verify installation
clang-format --version
# Expected: clang-format version 14.x

cppcheck --version
# Expected: Cppcheck 2.7 or higher

ros2 pkg list | grep ament_lint
```

**Setup Pre-commit Hook:**
```bash
# Navigate to your git repository
cd ~/ultrabot_ws/src/navigation

# Create pre-commit hook
cat > .git/hooks/pre-commit << 'EOF'
#!/bin/bash
# Format all staged C++ files
for file in $(git diff --cached --name-only --diff-filter=ACM | grep -E '\.(cpp|hpp|h)$'); do
    clang-format -i "$file"
    git add "$file"
done
EOF

# Make executable
chmod +x .git/hooks/pre-commit
```

---

#### v0.2 Complete Installation (One Command)

```bash
# Install all v0.2 frameworks at once
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-core \
  ros-humble-nav2-util \
  ros-humble-slam-toolbox \
  ros-humble-amcl \
  ros-humble-diagnostic-updater \
  ros-humble-diagnostic-aggregator \
  ros-humble-rqt-robot-monitor \
  clang-format-14 \
  cppcheck \
  ros-humble-ament-lint \
  ros-humble-ament-lint-common

# Verify all packages installed
echo "Verifying v0.2 installation..."
ros2 pkg list | grep -E "(nav2|slam|amcl|diagnostic)" | wc -l
# Expected: 25+ packages
```

---

### v0.3: Advanced Frameworks

Install frameworks for advanced features in v0.3.

#### 1. Fleet Management (RMF)

```bash
# Install RMF core
sudo apt install -y \
  ros-humble-rmf-core \
  ros-humble-rmf-fleet-adapter \
  ros-humble-rmf-task \
  ros-humble-rmf-traffic \
  ros-humble-rmf-visualization

# Install RMF demos (optional, for learning)
sudo apt install -y ros-humble-rmf-demos

# Verify installation
ros2 pkg list | grep rmf

# Expected output:
# rmf_building_map_msgs
# rmf_dispenser_msgs
# rmf_door_msgs
# rmf_fleet_adapter
# ... (20+ packages)
```

**Quick Test:**
```bash
# Run RMF demo (if demos installed)
ros2 launch rmf_demos_maps office_launch.xml

# In another terminal, check nodes
ros2 node list | grep rmf
```

---

#### 2. Sensor Fusion (robot_localization)

```bash
# Install robot_localization
sudo apt install -y ros-humble-robot-localization

# Verify installation
ros2 pkg list | grep robot_localization
# Expected: robot_localization

# Check for EKF node
ros2 pkg executables robot_localization
# Expected: robot_localization ekf_node
#           robot_localization ukf_node
#           robot_localization navsat_transform_node
```

**Configuration Template:**
```bash
# Create config directory
mkdir -p ~/ultrabot_ws/src/navigation/config/robot_localization

# Create example EKF config
cat > ~/ultrabot_ws/src/navigation/config/robot_localization/ekf.yaml << 'EOF'
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: true
    
    # Odometry input
    odom0: /odom
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  true,  false,
                   false, false, true,
                   false, false, false]
    
    # IMU input
    imu0: /imu/data
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  true,  false, false]
EOF
```

---

#### 3. Simulation (Gazebo)

```bash
# Install Gazebo 11
sudo apt install -y gazebo

# Install Gazebo ROS packages
sudo apt install -y \
  ros-humble-gazebo-ros \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins

# Install additional simulation tools
sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro

# Verify installation
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x

ros2 pkg list | grep gazebo
# Expected: gazebo_plugins, gazebo_ros, gazebo_ros_pkgs
```

**GPU Configuration (NVIDIA):**
```bash
# Check for GPU
lspci | grep -i nvidia

# If NVIDIA GPU present, install drivers
ubuntu-drivers devices
sudo ubuntu-drivers autoinstall

# Install CUDA (optional, for better performance)
# See: https://developer.nvidia.com/cuda-downloads
```

**Quick Test:**
```bash
# Launch empty Gazebo world
gazebo

# In another terminal, verify ROS bridge
ros2 topic list | grep gazebo
```

---

#### 4. Cybersecurity (SROS2)

```bash
# Install SROS2
sudo apt install -y ros-humble-sros2

# Install Fast DDS with security
sudo apt install -y \
  ros-humble-fastrtps \
  ros-humble-rmw-fastrtps-cpp

# Install OpenSSL (should already be present)
sudo apt install -y openssl libssl-dev

# Verify installation
ros2 pkg list | grep -E "(sros2|fastrtps)"
# Expected: sros2, fastrtps, rmw_fastrtps_cpp
```

**Generate Security Artifacts:**
```bash
# Create keystore directory
cd ~/ultrabot_ws/src/navigation
mkdir -p sros2_keystore

# Create keystore
ros2 security create_keystore sros2_keystore

# Generate keys for nodes
ros2 security create_enclave sros2_keystore /ultrabot/safety_supervisor
ros2 security create_enclave sros2_keystore /ultrabot/somanet_driver
ros2 security create_enclave sros2_keystore /ultrabot/command_arbitrator

# Set environment variables
export ROS_SECURITY_KEYSTORE=~/ultrabot_ws/src/navigation/sros2_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# Add to bashrc for persistence
cat >> ~/.bashrc << 'EOF'
# SROS2 Security
export ROS_SECURITY_KEYSTORE=~/ultrabot_ws/src/navigation/sros2_keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
EOF
```

**For complete SROS2 setup, see:** [Navigation/SROS2_GUIDE.md](../Navigation/SROS2_GUIDE.md)

---

#### v0.3 Complete Installation (One Command)

```bash
# Install all v0.3 frameworks at once
sudo apt update
sudo apt install -y \
  ros-humble-rmf-core \
  ros-humble-rmf-fleet-adapter \
  ros-humble-robot-localization \
  gazebo \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-sros2 \
  ros-humble-fastrtps \
  ros-humble-rmw-fastrtps-cpp \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro

# Verify all packages installed
echo "Verifying v0.3 installation..."
ros2 pkg list | grep -E "(rmf|robot_localization|gazebo|sros2)" | wc -l
# Expected: 30+ packages
```

---

## Verification Steps

### 1. Check ROS 2 Installation

```bash
# Check ROS version
ros2 doctor

# Expected output:
# All checks passed

# List all installed packages
ros2 pkg list | wc -l
# Expected: 200+ packages (with v0.2+v0.3)
```

### 2. Verify Nav2 Installation

```bash
# Check Nav2 lifecycle nodes
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:=['controller']

# In another terminal
ros2 lifecycle list
# Expected: /lifecycle_manager
```

### 3. Verify Diagnostic Tools

```bash
# Launch diagnostic aggregator
ros2 run diagnostic_aggregator aggregator_node &

# Publish test diagnostic
ros2 topic pub /diagnostics diagnostic_msgs/DiagnosticArray \
  "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''}, status: [{level: 0, name: 'test', message: 'OK', hardware_id: 'test_hw'}]}"

# Echo aggregated diagnostics
ros2 topic echo /diagnostics_agg
```

### 4. Verify Code Quality Tools

```bash
# Test clang-format
echo "int main(){return 0;}" > /tmp/test.cpp
clang-format /tmp/test.cpp
# Expected: formatted code

# Test cppcheck
cppcheck /tmp/test.cpp
# Expected: Checking /tmp/test.cpp ...

# Test ament_lint
cd ~/ultrabot_ws
colcon test --packages-select somanet
```

### 5. Verify Simulation

```bash
# Launch Gazebo with ROS bridge
ros2 launch gazebo_ros gazebo.launch.py

# Check topics
ros2 topic list | grep gazebo
# Expected: /clock, /parameter_events, etc.
```

---

## Troubleshooting

### Issue: Package Not Found

**Symptom:**
```
Package 'ros-humble-nav2-bringup' has no installation candidate
```

**Solution:**
```bash
# Update package list
sudo apt update

# Check if ROS 2 repository is configured
cat /etc/apt/sources.list.d/ros2.list

# If missing, reconfigure repository (see Prerequisites section)

# Try installing again
sudo apt install ros-humble-nav2-bringup
```

---

### Issue: ROS 2 Command Not Found

**Symptom:**
```
ros2: command not found
```

**Solution:**
```bash
# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Add to bashrc for persistence
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Issue: Gazebo Crashes on Startup

**Symptom:**
```
[gazebo-1] Segmentation fault (core dumped)
```

**Solution:**
```bash
# Check OpenGL support
glxinfo | grep OpenGL

# If using virtual machine, enable 3D acceleration
# VirtualBox: Settings → Display → Enable 3D Acceleration

# Try software rendering
export SVGA_VGPU10=0
gazebo

# For NVIDIA GPU, install drivers
sudo ubuntu-drivers autoinstall
sudo reboot
```

---

### Issue: SROS2 Permission Denied

**Symptom:**
```
[ERROR] Failed to load security credentials
```

**Solution:**
```bash
# Check keystore permissions
ls -la ~/ultrabot_ws/src/navigation/sros2_keystore

# Fix permissions
chmod -R 600 ~/ultrabot_ws/src/navigation/sros2_keystore

# Verify environment variables
echo $ROS_SECURITY_KEYSTORE
echo $ROS_SECURITY_ENABLE
```

---

### Issue: Nav2 Lifecycle Manager Fails

**Symptom:**
```
[ERROR] [lifecycle_manager]: Failed to change state for node
```

**Solution:**
```bash
# Check if all Nav2 nodes are installed
ros2 pkg list | grep nav2

# Verify parameter file syntax
ros2 param list

# Check node name configuration
ros2 node list

# Run with verbose logging
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args --log-level debug
```

---

### Issue: Insufficient Disk Space

**Symptom:**
```
E: You don't have enough free space in /var/cache/apt/archives/
```

**Solution:**
```bash
# Clean apt cache
sudo apt clean

# Remove old kernels
sudo apt autoremove

# Check disk usage
df -h

# If still insufficient, consider mounting additional storage
```

---

## Docker Installation

For containerized deployment (recommended for production):

### Dockerfile for v0.2

```dockerfile
# Dockerfile.v02
FROM osrf/ros:humble-desktop

# Install v0.2 frameworks
RUN apt-get update && apt-get install -y \
  ros-humble-nav2-bringup \
  ros-humble-nav2-core \
  ros-humble-nav2-util \
  ros-humble-slam-toolbox \
  ros-humble-amcl \
  ros-humble-diagnostic-updater \
  ros-humble-diagnostic-aggregator \
  clang-format-14 \
  cppcheck \
  ros-humble-ament-lint \
  && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /ultrabot_ws/src
WORKDIR /ultrabot_ws

# Source ROS on entry
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
```

### Build and Run

```bash
# Build image
docker build -t ultrabot:v0.2 -f Dockerfile.v02 .

# Run container
docker run -it --rm \
  --network host \
  -v ~/ultrabot_ws:/ultrabot_ws \
  ultrabot:v0.2

# Inside container
source /opt/ros/humble/setup.bash
cd /ultrabot_ws
colcon build
```

### Docker Compose for Complete Stack

```yaml
# docker-compose.yml
version: '3.8'

services:
  ultrabot:
    image: ultrabot:v0.2
    container_name: ultrabot_nav
    network_mode: host
    volumes:
      - ./ultrabot_ws:/ultrabot_ws
    environment:
      - ROS_DOMAIN_ID=42
      - ROS_LOCALHOST_ONLY=1
    command: bash -c "source /opt/ros/humble/setup.bash && ros2 launch navigation launch.py"
```

---

## Development Environment Setup

### VS Code Integration

```bash
# Install VS Code extensions
code --install-extension ms-vscode.cpptools
code --install-extension ms-python.python
code --install-extension ms-iot.vscode-ros

# Configure clang-format
mkdir -p .vscode
cat > .vscode/settings.json << 'EOF'
{
  "C_Cpp.clang_format_style": "Google",
  "editor.formatOnSave": true,
  "[cpp]": {
    "editor.defaultFormatter": "ms-vscode.cpptools"
  }
}
EOF
```

### CMake Integration

```cmake
# Add to CMakeLists.txt
find_package(ament_cmake_clang_format REQUIRED)
find_package(ament_cmake_cppcheck REQUIRED)

ament_clang_format(
  CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format
)

ament_cppcheck(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
```

---

## Related Documentation

- **[FRAMEWORKS.md](FRAMEWORKS.md)** - Complete framework catalog
- **[ROADMAP.md](ROADMAP.md)** - Version-based development roadmap
- **[FRAMEWORKS_QUICK_REFERENCE.md](FRAMEWORKS_QUICK_REFERENCE.md)** - Quick reference cheat sheet
- **[Navigation/README.md](../Navigation/README.md)** - Navigation package documentation

---

## Support Resources

- **ROS 2 Documentation:** https://docs.ros.org/en/humble/
- **ROS Discourse:** https://discourse.ros.org/
- **GitHub Issues:** [Repository issue tracker]
- **Stack Overflow:** Tag `ros2`

---

**Last Updated:** 2025-11-05 | **Tested On:** Ubuntu 22.04.3 LTS | **Maintainer:** Ultrabot Team
