# Ultrabot Development Roadmap

**Project:** Ultrabot AGV Platform | **Current Phase:** v0.2 | **Target:** Medical/Hospital Robotics

---

## Table of Contents

- [Overview](#overview)
- [Version Milestones](#version-milestones)
  - [v0.2: Foundation Phase](#v02-foundation-phase-current)
  - [v0.3: Advanced Features](#v03-advanced-features)
  - [Continuous: Code Quality](#continuous-code-quality)
- [Implementation Timeline](#implementation-timeline)
- [Dependencies Graph](#dependencies-graph)
- [Success Criteria](#success-criteria)
- [Risk Assessment](#risk-assessment)

---

## Overview

This roadmap outlines the phased integration of ROS 2 frameworks for the Ultrabot platform evolution. The approach prioritizes safety-critical features and compliance requirements while incrementally adding advanced capabilities.

**Key Principles:**
- **Safety First:** Core safety and diagnostics implemented in v0.2
- **Incremental Integration:** Each phase builds on previous capabilities
- **Standards Compliance:** ISO 13849-1, IEC 62443, FDA guidance
- **Hospital-Optimized:** Features tailored for medical environments

---

## Version Milestones

### v0.2: Foundation Phase (CURRENT)

**Target:** Q4 2025 | **Status:** In Development | **Priority:** HIGH

#### Objective
Establish core autonomous navigation and safety infrastructure for single-robot operation in hospital environments.

#### Core Requirements

##### 🧭 Navigation Stack
**Frameworks:**
- `ros-humble-nav2-bringup`
- `ros-humble-nav2-core`
- `ros-humble-nav2-util`

**Implementation Tasks:**
- [ ] Configure Nav2 behavior trees for hospital corridors
- [ ] Tune DWB controller for smooth navigation
- [ ] Set up waypoint navigation for room-to-room transport
- [ ] Implement dynamic obstacle avoidance
- [ ] Configure costmaps for hospital environment

**Dependencies:**
- Existing: EtherCAT motor control (somanet package)
- Existing: Differential drive kinematics
- Required: Laser scanner integration (e.g., SICK TiM561)

**Estimated Effort:** 3-4 weeks

**Documentation:**
- [Nav2 Tuning Guide](https://navigation.ros.org/tuning/)
- [Behavior Trees](https://navigation.ros.org/behavior_trees/)

---

##### 🗺️ SLAM & Localization
**Frameworks:**
- `ros-humble-slam-toolbox`
- `ros-humble-amcl`

**Implementation Tasks:**
- [ ] Map hospital floors using SLAM Toolbox
- [ ] Configure AMCL for real-time localization
- [ ] Set up map server with floor plans
- [ ] Tune particle filter parameters
- [ ] Implement map switching for multi-floor operation

**Dependencies:**
- Required: Lidar data at 20Hz minimum
- Required: Stable odometry (already implemented)
- Required: Static transform tree (base_link → laser_frame)

**Estimated Effort:** 2-3 weeks

**Documentation:**
- [SLAM Toolbox Configuration](https://github.com/SteveMacenski/slam_toolbox)
- [AMCL Configuration Guide](https://navigation.ros.org/configuration/packages/configuring-amcl.html)

---

##### 🛡️ Safety & Diagnostics
**Frameworks:**
- `ros-humble-diagnostic-updater`
- `ros-humble-diagnostic-aggregator`

**Implementation Tasks:**
- [ ] Integrate diagnostic_updater in safety_supervisor node
- [ ] Add battery level monitoring
- [ ] Add motor controller diagnostics
- [ ] Implement E-stop circuit diagnostics
- [ ] Configure diagnostic aggregator for system overview
- [ ] Set up diagnostic logging for certification

**Dependencies:**
- Existing: Safety supervisor (already implemented)
- Required: Battery management system interface
- Required: Motor controller status feedback

**Estimated Effort:** 1-2 weeks

**Compliance Requirements:**
- ISO 13849-1:2023 §5.2.1 (Diagnostic coverage)
- ISO 14971:2019 (Risk management file)
- FDA Guidance (Software validation)

**Documentation:**
- [diagnostic_updater Tutorial](https://github.com/ros/diagnostics)
- [Diagnostic Best Practices](https://wiki.ros.org/diagnostics)

---

##### 📜 Code Quality Tools
**Frameworks:**
- `clang-format`
- `cppcheck`
- `ros-humble-ament-lint`

**Implementation Tasks:**
- [ ] Configure clang-format with Google C++ style
- [ ] Set up pre-commit hooks for formatting
- [ ] Configure cppcheck in CI/CD pipeline
- [ ] Add ament_lint tests to CMakeLists.txt
- [ ] Document coding standards

**Dependencies:**
- Existing: Git repository
- Required: GitHub Actions or similar CI

**Estimated Effort:** 1 week

**Documentation:**
- [ROS 2 Code Style](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)

---

#### v0.2 Success Criteria

- [ ] Robot navigates autonomously between 10+ waypoints
- [ ] AMCL localization error < 10 cm in mapped environment
- [ ] Obstacle avoidance works reliably in dynamic environments
- [ ] Diagnostics report battery, motors, E-stop status
- [ ] Code passes all linting and static analysis checks
- [ ] Safety supervisor remains functional (existing feature)
- [ ] Documentation complete for all new features

#### v0.2 Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| Nav2 performance on low-power hardware | HIGH | Profile computational load, consider GPU acceleration |
| AMCL failure in symmetrical corridors | MEDIUM | Add visual fiducials, tune particle count |
| Lidar occlusion by hospital equipment | MEDIUM | Add secondary sensor (depth camera) |
| Incomplete diagnostic coverage | HIGH | Early compliance review with certification body |

---

### v0.3: Advanced Features

**Target:** Q2 2026 | **Status:** Planned | **Priority:** MEDIUM

#### Objective
Enable multi-robot fleet operation with advanced sensor fusion and secure communication for hospital-wide deployment.

#### Core Requirements

##### 🤝 Fleet Management
**Frameworks:**
- `ros-humble-rmf-core`
- `ros-humble-rmf-fleet-adapter`

**Implementation Tasks:**
- [ ] Set up RMF traffic scheduler
- [ ] Implement fleet adapter for Ultrabot
- [ ] Configure elevator integration
- [ ] Configure automatic door integration
- [ ] Set up centralized task dispatcher
- [ ] Implement conflict resolution for narrow corridors

**Dependencies:**
- Required: Nav2 navigation (v0.2)
- Required: Hospital building management system integration
- Required: Network infrastructure (WiFi 6 recommended)

**Estimated Effort:** 6-8 weeks

**Documentation:**
- [Open-RMF Integration Guide](https://osrf.github.io/ros2multirobotbook/)
- [Fleet Adapter Tutorial](https://github.com/open-rmf/free_fleet)

---

##### 📈 Sensor Fusion
**Frameworks:**
- `ros-humble-robot-localization`

**Implementation Tasks:**
- [ ] Configure EKF for odom→base_link fusion
- [ ] Integrate IMU data (angular velocity, acceleration)
- [ ] Tune covariance matrices for wheel slip
- [ ] Validate fusion accuracy on slippery floors
- [ ] Configure UKF for advanced scenarios (optional)

**Dependencies:**
- Required: IMU integration (e.g., Xsens MTi or Bosch BMI088)
- Required: Stable odometry (v0.2)
- Optional: Visual odometry for redundancy

**Estimated Effort:** 2-3 weeks

**Documentation:**
- [robot_localization Configuration](https://docs.ros.org/en/humble/p/robot_localization/)

---

##### 🧰 Simulation Environment
**Frameworks:**
- `ros-humble-gazebo-ros-pkgs`
- Gazebo 11

**Implementation Tasks:**
- [ ] Create Ultrabot URDF model for Gazebo
- [ ] Model hospital corridor environment
- [ ] Integrate Nav2 with Gazebo simulation
- [ ] Create test scenarios (narrow corridors, elevators)
- [ ] Set up multi-robot simulation
- [ ] Automate regression testing in CI/CD

**Dependencies:**
- Required: Accurate robot URDF
- Required: Hospital CAD models (simplified)
- Optional: GPU for rendering (NVIDIA recommended)

**Estimated Effort:** 4-5 weeks

**Documentation:**
- [Gazebo ROS 2 Integration](https://github.com/ros-simulation/gazebo_ros_pkgs)
- [URDF Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)

---

##### 🔐 Cybersecurity (IEC 62443)
**Frameworks:**
- `ros-humble-sros2`
- `ros-humble-fastdds`

**Implementation Tasks:**
- [ ] Generate security certificates for all nodes
- [ ] Configure DDS security policies
- [ ] Enable node authentication
- [ ] Enable topic encryption
- [ ] Set up secure parameter server
- [ ] Implement security audit logging
- [ ] Conduct penetration testing

**Dependencies:**
- Required: PKI infrastructure (e.g., OpenSSL CA)
- Required: Secure key storage (HSM or TPM recommended)
- Required: Network segmentation (separate VLAN for robots)

**Estimated Effort:** 3-4 weeks

**Compliance Requirements:**
- IEC 62443-3-3 (System security requirements)
- IEC 62443-4-2 (Technical security requirements)
- FDA Cybersecurity Guidance (2023)

**Documentation:**
- [SROS2 Tutorial](https://design.ros2.org/articles/ros2_dds_security.html)
- [Fast DDS Security](https://fast-dds.docs.eprosima.com/en/latest/fastdds/security/security.html)
- Existing: Navigation/SROS2_GUIDE.md

---

#### v0.3 Success Criteria

- [ ] 3+ robots coordinate autonomously via RMF
- [ ] Zero robot-robot collisions in narrow corridors
- [ ] IMU+odometry fusion reduces localization error by 30%
- [ ] Gazebo simulation matches real-world performance
- [ ] All DDS communication encrypted and authenticated
- [ ] Security audit passes penetration testing
- [ ] Fleet operates for 8+ hours without human intervention

#### v0.3 Risks

| Risk | Impact | Mitigation |
|------|--------|------------|
| RMF learning curve | HIGH | Hire consultant, allocate extra training time |
| Building integration complexity | HIGH | Early engagement with facility IT department |
| Network latency for fleet coordination | MEDIUM | Deploy edge computing, redundant WiFi APs |
| SROS2 performance overhead | MEDIUM | Benchmark early, optimize cryptography settings |
| Simulation-reality gap | MEDIUM | Validate in real environment early and often |

---

### Continuous: Code Quality

**Target:** Ongoing | **Status:** Active | **Priority:** HIGH

#### Objective
Maintain code quality, security, and documentation throughout the project lifecycle.

#### Core Requirements

##### 📜 Linting & Formatting
**Frameworks:**
- `clang-format`
- `cppcheck`
- `ros-humble-ament-lint`

**Ongoing Tasks:**
- [ ] Run clang-format on every commit (pre-commit hook)
- [ ] Weekly cppcheck static analysis
- [ ] Monthly dependency vulnerability scans
- [ ] Quarterly code reviews
- [ ] Annual external security audit

**Automation:**
```yaml
# .github/workflows/code_quality.yml
name: Code Quality
on: [push, pull_request]
jobs:
  lint:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3
      - name: Install tools
        run: |
          sudo apt-get install -y clang-format cppcheck
      - name: Run clang-format
        run: |
          clang-format --dry-run --Werror src/**/*.cpp
      - name: Run cppcheck
        run: |
          cppcheck --enable=all --error-exitcode=1 src/
      - name: Run ament_lint
        run: |
          source /opt/ros/humble/setup.bash
          colcon test --packages-select somanet
```

**Documentation:**
- [CI/CD Best Practices](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/CLI.html)

---

## Implementation Timeline

```
v0.2 (Q4 2025)
├─ Week 1-4:   Nav2 integration and tuning
├─ Week 5-7:   SLAM + AMCL setup
├─ Week 8-9:   Diagnostics integration
├─ Week 10:    Code quality tools
└─ Week 11-12: Testing and documentation

v0.3 (Q2 2026)
├─ Month 1-2:  RMF fleet management
├─ Month 3:    Sensor fusion (IMU)
├─ Month 4-5:  Gazebo simulation
├─ Month 6:    SROS2 cybersecurity
└─ Month 7-8:  Integration testing and compliance
```

---

## Dependencies Graph

```
v0.2 Foundation
  ├─ Nav2 Stack
  │   ├─ Requires: Lidar, Odometry (existing)
  │   └─ Enables: Autonomous navigation
  │
  ├─ SLAM/AMCL
  │   ├─ Requires: Lidar, Odometry
  │   └─ Enables: Map-based operation
  │
  ├─ Diagnostics
  │   ├─ Requires: Safety supervisor (existing)
  │   └─ Enables: Certification compliance
  │
  └─ Code Quality
      └─ Enables: Maintainability

v0.3 Advanced Features
  ├─ Fleet Management (RMF)
  │   ├─ Requires: Nav2, Building integration
  │   └─ Enables: Multi-robot operation
  │
  ├─ Sensor Fusion
  │   ├─ Requires: IMU, Nav2
  │   └─ Enables: Robust localization
  │
  ├─ Simulation (Gazebo)
  │   ├─ Requires: URDF, Nav2
  │   └─ Enables: Virtual testing
  │
  └─ Cybersecurity (SROS2)
      ├─ Requires: PKI infrastructure
      └─ Enables: IEC 62443 compliance
```

---

## Success Criteria

### v0.2 Completion Checklist
- [ ] Navigation: 95% successful waypoint completion
- [ ] Localization: <10cm error in mapped areas
- [ ] Diagnostics: 100% safety-critical system coverage
- [ ] Code Quality: 90%+ test coverage, zero critical static analysis warnings
- [ ] Documentation: All APIs documented, installation guide validated
- [ ] Compliance: ISO 13849-1 checklist complete

### v0.3 Completion Checklist
- [ ] Fleet: 3+ robots coordinate for 8 hours without collision
- [ ] Sensor Fusion: 30% improvement in localization accuracy
- [ ] Simulation: 90% fidelity with real-world performance
- [ ] Cybersecurity: Pass penetration testing, IEC 62443-4-2 compliance
- [ ] Documentation: Security architecture document, threat model complete
- [ ] Compliance: FDA cybersecurity guidance checklist complete

---

## Risk Assessment

### Technical Risks

| Risk | Phase | Severity | Mitigation Strategy |
|------|-------|----------|---------------------|
| Computational limitations | v0.2 | HIGH | Optimize Nav2 parameters, consider edge GPU |
| AMCL kidnapping problem | v0.2 | MEDIUM | Add visual fiducials, recovery behaviors |
| RMF integration complexity | v0.3 | HIGH | Allocate consulting budget, pilot testing |
| Network reliability | v0.3 | HIGH | Redundant WiFi, graceful degradation |
| SROS2 key management | v0.3 | MEDIUM | Use HSM/TPM, automate certificate rotation |

### Compliance Risks

| Risk | Phase | Severity | Mitigation Strategy |
|------|-------|----------|---------------------|
| Incomplete diagnostic coverage | v0.2 | CRITICAL | Early certification body engagement |
| Security audit failure | v0.3 | HIGH | External security review in development |
| FDA software validation gaps | Both | HIGH | Follow IEC 62304 throughout development |
| ISO 13849-1 PLr calculation | v0.2 | MEDIUM | Independent safety consultant review |

### Schedule Risks

| Risk | Phase | Severity | Mitigation Strategy |
|------|-------|----------|---------------------|
| Nav2 tuning delays | v0.2 | MEDIUM | Start tuning early, budget extra time |
| Building integration delays | v0.3 | HIGH | Engage facility IT in planning phase |
| Vendor hardware delays | Both | MEDIUM | Order long-lead sensors early |
| Staff turnover/knowledge loss | Both | MEDIUM | Comprehensive documentation, pair programming |

---

## Related Documentation

- **[FRAMEWORKS.md](FRAMEWORKS.md)** - Complete framework catalog with installation instructions
- **[INSTALLATION_FRAMEWORKS.md](INSTALLATION_FRAMEWORKS.md)** - Step-by-step installation guide
- **[Navigation/SAFETY.md](../Navigation/SAFETY.md)** - Safety procedures and compliance
- **[Navigation/SROS2_GUIDE.md](../Navigation/SROS2_GUIDE.md)** - Security setup guide

---

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2025-11-05 | Initial roadmap for v0.2 and v0.3 |

---

**Maintained by:** Ultrabot Team | **Review Cycle:** Quarterly | **Next Review:** 2026-02-05
