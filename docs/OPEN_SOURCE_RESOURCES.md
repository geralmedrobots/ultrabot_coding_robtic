# Open-Source Repositories for Ultrabot Integration

This catalog lists upstream projects that directly support Ultrabot's roadmap.
Each entry includes a short description, highlights the primary benefits for
the AGV platform, and notes the current integration status.

## Navigation & Autonomy

| Repository | Description | Value for Ultrabot | Integration Notes |
|------------|-------------|--------------------|-------------------|
| [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2) | ROS 2 Nav2 stack for autonomous navigation. | Provides path planning, behavior trees, and controller plugins that underpin Ultrabot's autonomous missions. | Core dependency; `somanet_driver_node` feeds odometry/cmd_vel into Nav2 behaviors. |
| [ros-planning/navigation_msgs](https://github.com/ros-planning/navigation_msgs) | Message definitions for navigation and localization. | Supplies the standardized message interfaces used across the navigation stack. | Already imported transitively via Nav2; ensure message versions stay aligned with ROS distribution. |
| [ros-controls/ros2_control](https://github.com/ros-controls/ros2_control) | Hardware abstraction, controller manager, and interfaces for ROS 2. | Enables future migration from the bespoke drive interface to a standard hardware interface with controller lifecycle management. | Targeted for v0.3 roadmap milestone before introducing multi-axis drive coordination. |
| [ros/robot_state_publisher](https://github.com/ros/robot_state_publisher) | Publishes TF frames from URDF descriptions. | Simplifies broadcasting the robot's kinematic tree derived from Ultrabot's URDF. | Recommended for near-term integration alongside TF publisher refactors. |

## Localization & Mapping

| Repository | Description | Value for Ultrabot | Integration Notes |
|------------|-------------|--------------------|-------------------|
| [ros2/robot_localization](https://github.com/ros2/robot_localization) | Extended Kalman Filter and UKF fusion for localization. | Offers sensor fusion for wheel odometry, IMU, and future sensors to improve pose estimation. | Roadmap v0.3 milestone; mock datasets can be used before hardware sensors are wired. |
| [SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | 2D SLAM and lifelong mapping for ROS 2. | Supplies mapping capabilities for hospitals where prior maps are unavailable. | Candidate for mapping workflows in v0.2/v0.3; ensure compatibility with Nav2 release train. |

## Fleet & Infrastructure

| Repository | Description | Value for Ultrabot | Integration Notes |
|------------|-------------|--------------------|-------------------|
| [open-rmf/rmf_core](https://github.com/open-rmf/rmf_core) | Core scheduling and fleet management services. | Enables multi-robot coordination, task allocation, and traffic scheduling aligned with hospital workflows. | Planned adoption in v0.3; requires harmonizing Ultrabot's API with RMF fleet adapters. |
| [open-rmf/rmf_ros2](https://github.com/open-rmf/rmf_ros2) | ROS 2 integration packages for RMF. | Bridges RMF core services with ROS 2 nodes, easing deployment into existing hospital infrastructure. | Evaluate once single-robot navigation is stable. |

## Safety & Diagnostics

| Repository | Description | Value for Ultrabot | Integration Notes |
|------------|-------------|--------------------|-------------------|
| [ros-tooling/system_metrics_collector](https://github.com/ros-tooling/system_metrics_collector) | Collects CPU, memory, and network metrics in ROS 2. | Provides real-time diagnostics essential for ISO 13849-1 evidence packs. | Prototype integration planned for diagnostic milestone; can run alongside watchdog helpers. |
| [ros-security/sros2](https://github.com/ros-security/sros2) | Security tooling for ROS 2 (policy generation, key handling). | Delivers encryption and authentication primitives required for IEC 62443 alignment. | Already referenced in security documentation; include policy templates during Jazzy migration. |

## Simulation & Testing

| Repository | Description | Value for Ultrabot | Integration Notes |
|------------|-------------|--------------------|-------------------|
| [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim) | Gazebo (Ignition) simulator for robotics. | Enables hardware-in-the-loop testing and virtual validation of hospital layouts. | Simulation environment scheduled for v0.3; start with basic world models. |
| [ros2/ros_testing](https://github.com/ros2/ros_testing) | Integration testing utilities for ROS 2. | Supports writing launch-based integration tests that exercise the lifecycle node with real middleware. | Incorporate once CI environment can execute `colcon test` end-to-end. |

## Perception & Sensors

| Repository | Description | Value for Ultrabot | Integration Notes |
|------------|-------------|--------------------|-------------------|
| [ros-drivers/ros2_ouster](https://github.com/ros-drivers/ros2_ouster) | Drivers and ROS 2 nodes for Ouster LiDAR sensors. | Provides high-resolution point clouds for corridor obstacle detection and SLAM refinement. | Optional for hardware rev that includes a roof-mounted LiDAR; align sensor launch files with Nav2 costmaps. |
| [ros-drivers/velodyne](https://github.com/ros-drivers/velodyne) | ROS drivers and point cloud utilities for Velodyne LiDARs. | Supplies alternative LiDAR support if procurement shifts vendors. | Evaluate once LiDAR hardware is finalized; the `velodyne_driver` and `velodyne_pointcloud` packages are Humble/Jazzy ready. |
| [ros-drivers/usb_cam](https://github.com/ros-drivers/usb_cam) | Simple ROS 2 camera driver for USB cameras. | Enables quick prototyping of vision-based patient/asset detection before bespoke perception stacks are added. | Useful for simulation or pilot deployments; migrate to industrial cameras as needed. |

## Human-Machine Interface & Telemetry

| Repository | Description | Value for Ultrabot | Integration Notes |
|------------|-------------|--------------------|-------------------|
| [ros-visualization/rqt](https://github.com/ros-visualization/rqt) | Qt-based GUI framework for ROS. | Offers operator dashboards for diagnostics, topic inspection, and teleop panels during trials. | Add curated perspectives for maintenance staff in the commissioning milestone. |
| [foxglove/studio](https://github.com/foxglove/studio) | Cross-platform visualization and debugging studio for robotics. | Provides modern dashboards, map overlays, and bag playback for hospital validation sessions. | Deploy via desktop app or web build; integrate with bagging workflows to review incidents. |
| [ros-visualization/webviz](https://github.com/ros-visualization/webviz) | Web-based ROS visualization suite. | Enables lightweight, browser-accessible monitoring stations inside hospitals. | Consider for nurse-station dashboards once security policies permit internal web apps. |

## Quality Assurance & CI

| Repository | Description | Value for Ultrabot | Integration Notes |
|------------|-------------|--------------------|-------------------|
| [ros-tooling/action-ros-ci](https://github.com/ros-tooling/action-ros-ci) | GitHub Action for building and testing ROS workspaces. | Automates colcon builds/tests across Humble and Jazzy, closing the current CI gap. | Configure once outbound network restrictions are resolved; pair with hardware-in-the-loop jobs later. |
| [ament/ament_lint](https://github.com/ament/ament_lint) | Suite of linting tools for ROS packages. | Ensures style and static-analysis compliance (ament_cpplint, uncrustify, xunit). | Already partially in use; extend to cover launch files and Python utilities. |
| [ros2/launch_testing](https://github.com/ros2/launch_testing) | Python framework for integration tests within ROS launches. | Enables end-to-end verification of lifecycle transitions, topic flow, and watchdog behaviors. | Introduce once the CI environment can spawn DDS middleware reliably. |

---

### How to Use This Catalog

1. **Prioritize by Roadmap:** Focus on Nav2, SLAM Toolbox, and robot_localization to close the v0.2 autonomy milestones before onboarding fleet management.
2. **Track Compatibility:** Align repository branches with the chosen ROS distribution (Humble vs. Jazzy) to avoid API drift.
3. **Leverage Community:** Monitor upstream issue trackers for security advisories and long-term support plans relevant to medical deployments.
4. **Contribute Back:** Where Ultrabot patches upstream components (e.g., hardware interfaces or safety plugins), upstream the changes to reduce maintenance overhead.

For additional frameworks and tooling, refer to [docs/FRAMEWORKS.md](FRAMEWORKS.md) and the roadmap-specific dependencies in [docs/ROADMAP.md](ROADMAP.md).
