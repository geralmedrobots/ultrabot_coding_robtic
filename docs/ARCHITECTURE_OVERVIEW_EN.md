# Recommended Architecture for Ultrabot AGV

## Layered Overview
- **Application & Operational Flows**: Use the `Navigation/` package as the functional core (navigation nodes, safety, drivers, and tests), keeping each domain in dedicated subfolders (`src`, `include`, `config`, `scripts`, `test`). This isolates C++/ROS logic, configurations, and operational/CI utilities.
- **Two-Layer Safety**: Combine a physical E-stop with a software Safety Supervisor, including watchdogs and a deadman switch. Ensure the supervisor starts before the drive node and that checklists run in every session.
- **Motion Control & EtherCAT**: Centralize motor communication via SOEM (EtherCAT) in a dedicated node (`somanet_driver`/`main`) with mandatory interface configuration (`ethercat_interface`).
- **Perception & Navigation**: Base navigation on Nav2, planning/control, SLAM Toolbox/AMCL for mapping/localization, and sensor support (RealSense/Ouster) using manifests `third_party/*.repos`. Each sensor has its own YAML in `Navigation/config/` and can be toggled through modular launch files.
- **Command Arbitration & Odometry**: Separate `command_arbitrator` to prioritize command sources (teleop, autonomy, maintenance) per ISO 3691-4, and the `OdometryCalculator` class to publish odometry with dynamic covariances.
- **Lifecycle & Bring-up**: Use lifecycle-managed nodes (safety_supervisor, driver, arbitrator) with controlled activation via launch, with the `--autostart` option or environment variables to automate transitions and maintain operational predictability.
- **Information Security**: Enable SROS2 (keystore, enforcements) in production for encryption and authentication of topics/services, following `scripts/setup_sros2.sh` and ROS_SECURITY.* environment variables.
- **Diagnostics & Compliance**: Publish periodic diagnostics (1 Hz) and integrate QA tools (clang-format, cppcheck, ament_lint) to maintain traceability and adherence to standards (ISO 13849-1, IEC 62443).
- **Observability & Simulation**: Use prepared RViz layouts (`config/sensor_visualization.rviz`) and plan simulation with Gazebo and RMF in v0.3 for multi-robot scenarios and building integration.

## Recommended Node/Service Split
- `safety_supervisor`: Monitors E-stop, limits, watchdogs; issues safe stop commands.
- `somanet_driver` (EtherCAT): Translates velocity commands to drives, applies limiters, and validates critical parameters.
- `command_arbitrator`: Selects command source by priority (teleop > maintenance > autonomy) and safety state.
- `odometry_publisher`: Computes and publishes odometry; integrates additional sensors in the fusion phase (v0.3).
- `nav2` stack + `slam_toolbox`/`amcl`: Planning, control, and localization; configured via YAML in `config/`.
- `diagnostics_updater`: Aggregates health checks for compliance and predictive maintenance.

## Integration & Configuration Notes
- **Mandatory EtherCAT configuration** via environment variables, YAML, or launch parameters to prevent unintended hardware activation.
- **VCS manifests** (`Navigation/third_party/*.repos`) for quickly vendoring sensor drivers.
- **ROS Security variables** (`ROS_SECURITY_*`) for cryptographic enforcement in production.

## Roadmap Aligned to the Architecture
- **v0.2** focuses on single-robot operation with navigation, SLAM/AMCL, safety, and diagnostics.
- **v0.3** expands to RMF (fleet), sensor fusion (`robot_localization`), Gazebo, and hardened SROS2.

Use this architecture as a baseline and adjust nodes and sensor configuration according to hardware targets and regulatory requirements of the deployment environment.
