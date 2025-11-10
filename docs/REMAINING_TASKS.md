# Remaining Work Checklist for Ultrabot

This checklist consolidates the outstanding work required to reach full operational readiness.
It draws from the operational readiness snapshot, roadmap milestones, and safety guidance so
contributors can quickly identify the next actionable tasks.

## Hardware-in-the-Loop Validation

- [ ] Validate the `ethercat_driver` against production drives with SOEM installed, confirming
      watchdog timeouts, fault clearing, and emergency-stop handling on physical hardware.
- [ ] Capture oscilloscope traces or drive diagnostics during bring-up to document compliance with
      hospital safety requirements.
- [ ] Exercise battery management, charging, and docking workflows with the safety supervisor
      running alongside the navigation stack.

## Autonomy Stack Completion

- [ ] Configure Nav2 behavior trees and controllers using datasets collected from the hospital
      environment; document tuning parameters in `docs/ROADMAP.md`.
- [ ] Integrate SLAM Toolbox and AMCL pipelines to generate and consume facility maps, including
      validation runs that confirm localization accuracy.
- [ ] Fuse wheel odometry with IMU data through `robot_localization` to improve pose estimates in
      long corridors and elevators.

## Perception & Sensor Integration

- [ ] Capture Intel RealSense D455 intrinsics/extrinsics per robot and store the calibration in
      `config/realsense_d455.yaml` overrides.
- [ ] Export Ouster metadata for each LiDAR (`os_config.json`) and validate point-cloud fidelity
      against hospital corridors, updating `config/ouster_lidar.yaml` as required.
- [ ] Wire depth and point-cloud topics into Nav2 costmaps and obstacle layers, documenting the
      configuration in the navigation bring-up guide.

## Diagnostics and Observability

- [ ] Wire ROS 2 diagnostics (`diagnostic_updater`, `system_metrics_collector`) to publish CPU,
      memory, network, and drive-health metrics for the fleet dashboard.
- [ ] Add automated alerts for watchdog kicks, fault transitions, and command saturation events.
- [ ] Produce Grafana dashboards or equivalent visualizations so hospital staff can audit system
      health in real time.

## Security Hardening

- [ ] Replace development SROS2 keystores with production secrets and enforce policy generation as
      part of the deployment pipeline.
- [ ] Enable secure boot and disk encryption on the compute unit, documenting the flashing and key
      management process.
- [ ] Perform penetration testing focused on ROS 2 middleware interfaces, recording remediation
      actions in `Navigation/SECURITY_SUMMARY.md`.

## Continuous Integration & Testing

- [ ] Provision a CI runner with ROS 2 Humble/Jazzy and SOEM so that `colcon test` can execute on
      each merge request.
- [ ] Extend the existing unit-test suite with launch-based integration tests using
      `ros2/ros_testing` to cover lifecycle transitions and TF publication.
- [ ] Simulate hospital scenarios in Gazebo (gz-sim) to validate docking, obstacle avoidance, and
      fleet scheduling with RMF adapters.

Progress updates should be recorded in the roadmap and operational readiness snapshot to maintain a
single source of truth for stakeholders.
