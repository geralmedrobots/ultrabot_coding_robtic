# Ultrabot Frameworks Quick Reference

**One-page cheat sheet for ROS 2 Humble frameworks**

---

## Installation One-Liners

### v0.2: Foundation
```bash
sudo apt install -y ros-humble-nav2-bringup ros-humble-nav2-core ros-humble-nav2-util ros-humble-slam-toolbox ros-humble-amcl ros-humble-diagnostic-updater ros-humble-diagnostic-aggregator clang-format-14 cppcheck ros-humble-ament-lint ros-humble-ament-lint-common
```

### v0.3: Advanced
```bash
sudo apt install -y ros-humble-rmf-core ros-humble-rmf-fleet-adapter ros-humble-robot-localization gazebo ros-humble-gazebo-ros-pkgs ros-humble-sros2 ros-humble-fastrtps ros-humble-rmw-fastrtps-cpp
```

---

## Framework Lookup Table

| Framework | Category | Version | Purpose | Package Name |
|-----------|----------|---------|---------|--------------|
| Nav2 | Navigation | v0.2 | Path planning, obstacle avoidance | `ros-humble-nav2-bringup` |
| SLAM Toolbox | Mapping | v0.2 | 2D SLAM with loop closure | `ros-humble-slam-toolbox` |
| AMCL | Localization | v0.2 | Monte Carlo localization | `ros-humble-amcl` |
| Diagnostics | Safety | v0.2 | System health monitoring | `ros-humble-diagnostic-updater` |
| RMF | Fleet | v0.3 | Multi-robot coordination | `ros-humble-rmf-core` |
| robot_localization | Fusion | v0.3 | IMU + odometry fusion | `ros-humble-robot-localization` |
| Gazebo | Simulation | v0.3 | Virtual testing environment | `gazebo` + `ros-humble-gazebo-ros-pkgs` |
| SROS2 | Security | v0.3 | DDS encryption + auth | `ros-humble-sros2` |
| clang-format | QA | All | Code formatting | `clang-format-14` |
| cppcheck | QA | All | Static analysis | `cppcheck` |
| ament_lint | QA | All | ROS 2 linting | `ros-humble-ament-lint` |

---

## Common Commands

### Navigation (Nav2)
```bash
# Launch Nav2 stack
ros2 launch nav2_bringup navigation_launch.py

# Set initial pose
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped "{...}"

# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{...}"

# View costmaps
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

### SLAM & Mapping
```bash
# Start SLAM Toolbox (online async)
ros2 launch slam_toolbox online_async_launch.py

# Save map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/tmp/my_map'}}"

# Start map server
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/path/to/map.yaml

# Start AMCL localization
ros2 run nav2_amcl amcl --ros-args --params-file /path/to/amcl_params.yaml
```

### Diagnostics
```bash
# View diagnostics in terminal
ros2 topic echo /diagnostics

# Launch rqt diagnostic viewer
ros2 run rqt_robot_monitor rqt_robot_monitor

# Aggregate diagnostics
ros2 run diagnostic_aggregator aggregator_node --ros-args --params-file /path/to/aggregator_params.yaml
```

### Fleet Management (RMF)
```bash
# Launch RMF traffic schedule
ros2 launch rmf_traffic_ros2 schedule.launch.xml

# Start fleet adapter
ros2 run rmf_fleet_adapter fleet_adapter --ros-args --params-file /path/to/fleet_config.yaml

# Submit task
ros2 run rmf_task_ros2 task_request_publisher --task-type delivery --start A --end B
```

### Sensor Fusion (robot_localization)
```bash
# Run EKF node
ros2 launch robot_localization ekf.launch.py

# Run UKF node
ros2 launch robot_localization ukf.launch.py

# Verify output
ros2 topic echo /odometry/filtered
```

### Simulation (Gazebo)
```bash
# Launch empty world
gazebo

# Launch world with ROS bridge
ros2 launch gazebo_ros gazebo.launch.py

# Spawn robot model
ros2 run gazebo_ros spawn_entity.py -file /path/to/robot.urdf -entity ultrabot

# Pause/unpause simulation
ros2 service call /pause_physics std_srvs/srv/Empty
ros2 service call /unpause_physics std_srvs/srv/Empty
```

### Security (SROS2)
```bash
# Create keystore
ros2 security create_keystore /path/to/keystore

# Create enclave (per node)
ros2 security create_enclave /path/to/keystore /namespace/node_name

# Generate security artifacts
ros2 security create_permission /path/to/keystore /namespace/node_name policy.xml

# Enable security (environment variables)
export ROS_SECURITY_KEYSTORE=/path/to/keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

# Launch secure node
ros2 run package_name node_name --ros-args --enclave /namespace/node_name
```

### Code Quality
```bash
# Format code
clang-format -i src/*.cpp include/*.hpp

# Static analysis
cppcheck --enable=all --inconclusive src/

# Run ament linters
colcon test --packages-select package_name

# View test results
colcon test-result --verbose
```

---

## Key Configuration Files

### Nav2
- **Main params:** `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml`
- **Controllers:** `controller_server` → DWB, TEB, MPPI
- **Planners:** `planner_server` → NavFn, Smac, Theta*
- **Costmaps:** `local_costmap`, `global_costmap`

### SLAM Toolbox
- **Config:** `slam_toolbox/config/mapper_params_online_async.yaml`
- **Key params:** `resolution`, `mode`, `minimum_travel_distance`

### AMCL
- **Config:** `nav2_bringup/params/nav2_params.yaml` (amcl section)
- **Key params:** `min_particles`, `max_particles`, `laser_model_type`

### robot_localization
- **EKF config:** `robot_localization/params/ekf.yaml`
- **Sensors:** `odom0`, `imu0`, `pose0`
- **Config format:** 15-element boolean array per sensor

### Gazebo
- **World files:** `~/.gazebo/worlds/`
- **Models:** `~/.gazebo/models/` or `GAZEBO_MODEL_PATH`
- **Plugins:** `libgazebo_ros_init.so`, `libgazebo_ros_factory.so`

### SROS2
- **Keystore:** `~/sros2_keystore/`
- **Policies:** `<keystore>/enclaves/<namespace>/<node>/`
- **Governance:** `<keystore>/governance.xml`

---

## Environment Variables

```bash
# ROS 2 Core
export ROS_DOMAIN_ID=42                  # Default: 0, range [0-101]
export ROS_LOCALHOST_ONLY=1              # Restrict to localhost
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET  # Discovery scope

# ROS 2 Security
export ROS_SECURITY_KEYSTORE=/path/to/keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce     # Or Permissive

# Gazebo
export GAZEBO_MODEL_PATH=/path/to/models
export GAZEBO_RESOURCE_PATH=/path/to/resources
export GAZEBO_PLUGIN_PATH=/path/to/plugins

# DDS (FastDDS)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastdds.xml

# Development
export COLCON_DEFAULTS_FILE=~/.colcon/defaults.yaml
export AMENT_PREFIX_PATH=/opt/ros/humble
```

---

## Troubleshooting Quick Fixes

| Issue | Quick Fix |
|-------|-----------|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` |
| Package not found | `sudo apt update && sudo apt install ros-humble-<package>` |
| Node not discovered | Check `ROS_DOMAIN_ID`, firewall, or use `ROS_LOCALHOST_ONLY=1` |
| Nav2 won't start | Verify all lifecycle nodes in params, check `node_names` list |
| AMCL kidnapping | Increase `max_particles`, add global localization service |
| Gazebo crashes | Enable GPU acceleration, or use `export SVGA_VGPU10=0` |
| SROS2 permission denied | `chmod -R 600 <keystore>`, verify env vars |
| Build errors | `rm -rf build/ install/ log/ && colcon build` |
| Diagnostic errors | Check `/diagnostics` topic, use `rqt_robot_monitor` |

---

## Performance Tuning

### Nav2
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Increase for smoother control
    FollowPath.max_vel_x: 0.5   # Reduce for safety

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased.use_astar: false  # Use Dijkstra for smoother paths
```

### SLAM Toolbox
```yaml
slam_toolbox:
  ros__parameters:
    minimum_travel_distance: 0.2  # Increase to reduce CPU load
    minimum_travel_heading: 0.2
    resolution: 0.05              # Increase for coarser maps
```

### robot_localization
```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0  # Balance between accuracy and CPU load
    sensor_timeout: 0.1
```

---

## Version Matrix (Quick)

| Feature | v0.2 | v0.3 | Continuous |
|---------|------|------|------------|
| Navigation | ✅ | ✅ | ✅ |
| SLAM/AMCL | ✅ | ✅ | ✅ |
| Diagnostics | ✅ | ✅ | ✅ |
| Fleet Management | - | ✅ | ✅ |
| Sensor Fusion | - | ✅ | ✅ |
| Simulation | - | ✅ | ✅ |
| Security (SROS2) | - | ✅ | ✅ |
| Code Quality | ✅ | ✅ | ✅ |

---

## Documentation Links

| Topic | Link |
|-------|------|
| Nav2 | https://navigation.ros.org/ |
| SLAM Toolbox | https://github.com/SteveMacenski/slam_toolbox |
| RMF | https://osrf.github.io/ros2multirobotbook/ |
| robot_localization | https://docs.ros.org/en/humble/p/robot_localization/ |
| Gazebo | https://gazebosim.org/docs |
| SROS2 | https://design.ros2.org/articles/ros2_dds_security.html |
| ROS 2 Humble | https://docs.ros.org/en/humble/ |

---

## Ultrabot-Specific Files

| File | Purpose |
|------|---------|
| `docs/FRAMEWORKS.md` | Complete framework documentation |
| `docs/ROADMAP.md` | Version-based roadmap |
| `docs/INSTALLATION_FRAMEWORKS.md` | Detailed installation guide |
| `requirements/ros2_packages.txt` | Dependency list |
| `Navigation/README.md` | Navigation package documentation |
| `Navigation/SAFETY.md` | Safety procedures |
| `Navigation/SROS2_GUIDE.md` | Complete SROS2 setup |

---

**Quick Start:**
```bash
# Install v0.2 frameworks
sudo apt install -y ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-amcl ros-humble-diagnostic-updater

# Clone Ultrabot
git clone https://github.com/geralmedrobots/ultrabot_coding_robtic ~/ultrabot_ws/src/ultrabot
cd ~/ultrabot_ws

# Build
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# Source workspace
source install/setup.bash

# Launch (see Navigation/README.md for details)
ros2 launch somanet launch.py
```

---

**Last Updated:** 2025-11-05 | **Version:** 1.0.0 | **Maintainer:** Ultrabot Team
