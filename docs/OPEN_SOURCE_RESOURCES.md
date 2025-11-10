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
| [ros-planning/geometric_shapes](https://github.com/ros-planning/geometric_shapes) | Collision geometry utilities shared across ROS 2 planners. | Supplies mesh and primitive collision checking used by global/local planners and future manipulator payloads. | Import as part of Nav2 costmap tuning to ensure consistent obstacle inflation. |

## Localization & Mapping

| Repository | Description | Value for Ultrabot | Integration Notes |
|------------|-------------|--------------------|-------------------|
| [ros2/robot_localization](https://github.com/ros2/robot_localization) | Extended Kalman Filter and UKF fusion for localization. | Offers sensor fusion for wheel odometry, IMU, and future sensors to improve pose estimation. | Roadmap v0.3 milestone; mock datasets can be used before hardware sensors are wired. |
| [SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | 2D SLAM and lifelong mapping for ROS 2. | Supplies mapping capabilities for hospitals where prior maps are unavailable. | Candidate for mapping workflows in v0.2/v0.3; ensure compatibility with Nav2 release train. |
| [cartographer-project/cartographer](https://github.com/cartographer-project/cartographer) | Real-time SLAM framework for 2D/3D mapping. | Enables multi-floor mapping and localization using LiDAR + IMU, complementing SLAM Toolbox in complex environments. | Consider for research branches requiring dense 3D reconstructions from the Ouster sensor. |
| [octomap/octomap](https://github.com/OctoMap/octomap) | Probabilistic 3D mapping framework. | Provides occupancy trees for volumetric planning, useful for elevator vestibules and overhead obstacle avoidance. | Integrate with 3D LiDAR pipelines when moving beyond planar navigation. |

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
| [ros-drivers/diagnostics](https://github.com/ros-drivers/diagnostics) | Diagnostic updater and aggregator nodes for ROS. | Standardizes status reporting for drives, sensors, and safety controllers. | Tie into `CommandWatchdog` alerts to surface health in Nav2 dashboards. |
| [ros-tooling/system_test](https://github.com/ros-tooling/system_test) | End-to-end system test harness for ROS 2. | Supports black-box validation of safety behaviors before hospital trials. | Extend once CI networking allows running colcon-based integration suites. |

## Simulation & Testing

| Repository | Description | Value for Ultrabot | Integration Notes |
|------------|-------------|--------------------|-------------------|
| [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim) | Gazebo (Ignition) simulator for robotics. | Enables hardware-in-the-loop testing and virtual validation of hospital layouts. | Simulation environment scheduled for v0.3; start with basic world models. |
| [ros2/ros_testing](https://github.com/ros2/ros_testing) | Integration testing utilities for ROS 2. | Supports writing launch-based integration tests that exercise the lifecycle node with real middleware. | Incorporate once CI environment can execute `colcon test` end-to-end. |
| [ros2/rosbag2](https://github.com/ros2/rosbag2) | ROS 2 recording and playback tooling. | Facilitates regression testing with recorded hospital scenarios and sensor logs. | Incorporate into validation pipeline to replay safety incidents. |
| [ros2/demos](https://github.com/ros2/demos) | Example nodes and launch files for ROS 2 features. | Serves as reference implementations for DDS behaviors, lifecycle interactions, and composition. | Use as a baseline when extending Ultrabot-specific demos or tutorials. |

## Perception & Sensors

| Repository | Description | Value for Ultrabot | Integration Notes |
|------------|-------------|--------------------|-------------------|
| [IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros) | ROS 2 drivers for Intel RealSense depth cameras. | Supplies RGB-D data for obstacle avoidance, docking alignment, and semantic perception. | Included via `realsense2_camera`; configure D455 serial and frame conventions in `config/realsense_d455.yaml`. |
| [ros-drivers/ros2_ouster](https://github.com/ros-drivers/ros2_ouster) | Drivers and ROS 2 nodes for Ouster LiDAR sensors. | Provides high-resolution point clouds for corridor obstacle detection and SLAM refinement. | Optional for hardware rev that includes a roof-mounted LiDAR; align sensor launch files with Nav2 costmaps. |
| [ros-drivers/velodyne](https://github.com/ros-drivers/velodyne) | ROS drivers and point cloud utilities for Velodyne LiDARs. | Supplies alternative LiDAR support if procurement shifts vendors. | Evaluate once LiDAR hardware is finalized; the `velodyne_driver` and `velodyne_pointcloud` packages are Humble/Jazzy ready. |
| [ros-drivers/usb_cam](https://github.com/ros-drivers/usb_cam) | Simple ROS 2 camera driver for USB cameras. | Enables quick prototyping of vision-based patient/asset detection before bespoke perception stacks are added. | Useful for simulation or pilot deployments; migrate to industrial cameras as needed. |
| [ros-perception/image_pipeline](https://github.com/ros-perception/image_pipeline) | Camera processing nodes (rectification, debayering, stereo). | Provides essential preprocessing for RGB-D feeds before handing data to perception stacks. | Chain with RealSense depth images to improve obstacle segmentation. |
| [ros-perception/vision_msgs](https://github.com/ros-perception/vision_msgs) | Standardized messages for perception detections. | Establishes consistent interfaces for future semantic perception modules (people, assets). | Adopt alongside perception node development to ease RMF integration. |
| [pointcloudlibrary/pcl](https://github.com/PointCloudLibrary/pcl) | Library for 3D point cloud processing. | Offers filtering, segmentation, and registration algorithms for LiDAR data. | Use via ROS 2 wrappers when enhancing obstacle clustering and map cleanup. |

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
| [ros-industrial/industrial_ci](https://github.com/ros-industrial/industrial_ci) | Flexible CI pipeline for ROS 1/2 projects. | Provides containerized build/test workflows that can run on GitHub, GitLab, or local Jenkins. | Useful for staging pipeline until `action-ros-ci` is fully wired with hardware runners. |
| [ament/ament_cmake_ros](https://github.com/ament/ament_cmake_ros) | CMake extensions for ROS 2 packages. | Adds test, lint, and install helpers that keep Ultrabot's packages aligned with ROS 2 best practices. | Already indirect dependency; document usage to guide new contributors. |

---

### How to Use This Catalog

1. **Prioritize by Roadmap:** Focus on Nav2, SLAM Toolbox, and robot_localization to close the v0.2 autonomy milestones before onboarding fleet management.
2. **Track Compatibility:** Align repository branches with the chosen ROS distribution (Humble vs. Jazzy) to avoid API drift.
3. **Leverage Community:** Monitor upstream issue trackers for security advisories and long-term support plans relevant to medical deployments.
4. **Contribute Back:** Where Ultrabot patches upstream components (e.g., hardware interfaces or safety plugins), upstream the changes to reduce maintenance overhead.

For additional frameworks and tooling, refer to [docs/FRAMEWORKS.md](FRAMEWORKS.md) and the roadmap-specific dependencies in [docs/ROADMAP.md](ROADMAP.md).
