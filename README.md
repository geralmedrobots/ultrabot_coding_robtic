# Ultrabot AGV Platform

**Autonomous Guided Vehicle for Medical and Hospital Environments**

**Version:** 0.2 | **ROS Distribution:** Humble / Jazzy | **Platforms:** Ubuntu 22.04 LTS (Jammy) / 24.04 LTS (Noble)

---

## 🎯 Overview

Ultrabot is a safety-critical AGV platform designed for autonomous navigation in hospital and medical environments. The system implements international safety standards (ISO 13849-1, ISO 3691-4, IEC 61508) and features dual-layer safety architecture with compliance-ready diagnostics.

### Key Features

- ✅ **Autonomous Navigation:** Nav2 stack with dynamic obstacle avoidance
- ✅ **2D SLAM & Localization:** Real-time mapping and position tracking
- ✅ **Safety-Critical Control:** Dual-layer safety (hardware + software)
- ✅ **Standards Compliant:** ISO 13849-1, ISO 3691-4, IEC 61508, IEC 62443
- ✅ **EtherCAT Motor Control:** Industrial-grade drive interface
- ✅ **Secure Communication:** SROS2 encryption and authentication
- ✅ **Comprehensive Diagnostics:** CE/FDA compliance ready

---

## 📋 Quick Start

### Prerequisites

- **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish) **or** Ubuntu 24.04 LTS (Noble Numbat)
- **ROS:** ROS 2 Humble Hawksbill **or** ROS 2 Jazzy Jalisco (installed via the helper script below)
- **Hardware:** Minimum 8GB RAM, 4-core CPU
- **Network:** WiFi 5/6 or Gigabit Ethernet

### Installation

```bash
# 1. Clone repository
mkdir -p ~/ultrabot_ws/src
cd ~/ultrabot_ws/src
git clone https://github.com/geralmedrobots/ultrabot_coding_robtic.git

# 2. Install ROS 2 + system dependencies (Jammy → Humble, Noble → Jazzy)
cd ultrabot_coding_robtic
xargs -a requirements/ros2_packages.txt sudo apt install -y    # Jammy
# or, run the guided installer for automatic repo setup (supports Jammy + Noble)
sudo ./Navigation/scripts/install_dependencies.sh

# 3. Source the ROS distribution installed by the script
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash

# 4. Build workspace
cd ~/ultrabot_ws
colcon build --symlink-install

# 5. Source workspace
source install/setup.bash
```

> **Note:** The helper script performs `apt-get update` and fetches the ROS 2 GPG keys.
> If you are behind a proxy/firewall that blocks outbound HTTP(S), the script exits early
> and records logs in `/tmp/somanet_apt.<pid>.log` so you can remediate network access
> before retrying.

**For detailed installation instructions, see:** [docs/INSTALLATION_FRAMEWORKS.md](docs/INSTALLATION_FRAMEWORKS.md)

### Launch

```bash
# Configure EtherCAT interface (required)
export ETHERCAT_INTERFACE=eth0  # Replace with your interface name

# Launch the system
ros2 launch somanet launch.py

# In another terminal, verify nodes are running
ros2 node list
```

**For complete usage guide, see:** [Navigation/README.md](Navigation/README.md)

---

## 🏗️ Project Structure

```
ultrabot_coding_robtic/
├── Navigation/              # Main navigation package (somanet)
│   ├── src/                 # C++ source files
│   ├── include/             # Header files
│   ├── config/              # Configuration files (YAML)
│   ├── scripts/             # Python scripts and utilities
│   ├── test/                # Unit tests (GTest)
│   ├── README.md            # Navigation package documentation
│   ├── SAFETY.md            # Safety procedures and compliance
│   ├── SROS2_GUIDE.md       # Security setup guide
│   └── package.xml          # ROS 2 package manifest
├── docs/                    # Project documentation
│   ├── FRAMEWORKS.md        # ROS 2 frameworks catalog
│   ├── ROADMAP.md           # Development roadmap (v0.2, v0.3)
│   ├── INSTALLATION_FRAMEWORKS.md  # Installation guide
│   ├── OPERATIONAL_READINESS.md    # Snapshot of deployment/testing gaps
│   └── FRAMEWORKS_QUICK_REFERENCE.md  # Quick reference cheat sheet
├── requirements/            # Dependency lists
│   └── ros2_packages.txt    # ROS 2 package dependencies
├── LICENSE                  # Project license
└── README.md                # This file
```

---

## 🗺️ Project Roadmap

Ultrabot follows a phased development approach with clear milestones:

### v0.2: Foundation Phase (Current - Q4 2025)

**Status:** In Development | **Priority:** HIGH

**Core Features:**
- ✅ EtherCAT motor control (completed)
- ✅ Safety supervisor system (completed)
- ✅ Parameter validation (completed)
- 🔄 Nav2 autonomous navigation (in progress)
- 🔄 SLAM & AMCL localization (in progress)
- 🔄 Diagnostic system integration (in progress)
- 🔄 Code quality tools setup (in progress)

**Target:** Single-robot autonomous operation in mapped hospital environments

---

### v0.3: Advanced Features (Q2 2026)

**Status:** Planned | **Priority:** MEDIUM

**Core Features:**
- 📅 RMF fleet management (multi-robot coordination)
- 📅 Sensor fusion (IMU + odometry via robot_localization)
- 📅 Gazebo simulation environment
- 📅 SROS2 cybersecurity (IEC 62443 compliance)
- 📅 Building system integration (elevators, doors)

**Target:** Multi-robot fleet operation with secure communication

---

### Continuous: Code Quality & Compliance

**Status:** Active | **Priority:** HIGH

**Ongoing Activities:**
- ✅ Automated testing (6 focused unit tests covering safety, odometry, and lifecycle helpers)
- ✅ Automated testing (5 focused unit tests covering safety, odometry, and lifecycle helpers)
- ✅ Static analysis (cppcheck)
- ✅ Code formatting (clang-format)
- 🔄 Compliance documentation (ISO 13849-1, IEC 62443)
- 🔄 Security audits (quarterly)

---

**For complete roadmap with timelines and dependencies, see:** [docs/ROADMAP.md](docs/ROADMAP.md)

---

## 🧭 Framework Dependencies

Ultrabot leverages industry-standard ROS 2 frameworks organized by functionality:

### v0.2 Requirements

| Category | Frameworks | Purpose |
|----------|------------|---------|
| **Navigation** | Nav2 (bringup, core, util) | Path planning and control |
| **Mapping** | SLAM Toolbox, AMCL | 2D SLAM and localization |
| **Safety** | diagnostic_updater, diagnostic_aggregator | System health monitoring |
| **QA** | clang-format, cppcheck, ament_lint | Code quality tools |

### v0.3 Requirements

| Category | Frameworks | Purpose |
|----------|------------|---------|
| **Fleet** | RMF (core, fleet_adapter) | Multi-robot coordination |
| **Fusion** | robot_localization | IMU + odometry sensor fusion |
| **Simulation** | Gazebo 11 + ROS plugins | Virtual testing environment |
| **Security** | SROS2, Fast DDS | Encrypted communication |

**For complete framework catalog with installation, see:** [docs/FRAMEWORKS.md](docs/FRAMEWORKS.md)

---

## 📚 Documentation

### Getting Started
- **[Installation Guide](docs/INSTALLATION_FRAMEWORKS.md)** - Step-by-step setup with troubleshooting
- **[Quick Reference](docs/FRAMEWORKS_QUICK_REFERENCE.md)** - One-page cheat sheet for common commands
- **[Navigation Package](Navigation/README.md)** - Detailed package documentation

### Planning & Architecture
- **[Development Roadmap](docs/ROADMAP.md)** - Version milestones, timelines, and dependencies
- **[Framework Catalog](docs/FRAMEWORKS.md)** - Complete ROS 2 frameworks reference

### Safety & Compliance
- **[Safety Manual](Navigation/SAFETY.md)** - ⚠️ **Mandatory reading before operation**
- **[Security Setup](Navigation/SROS2_GUIDE.md)** - SROS2 configuration for IEC 62443
- **[Parameter Validation](Navigation/PARAMETER_VALIDATION.md)** - Safety-critical parameter bounds

### Additional Documentation
- **[Calibration Guide](Navigation/CALIBRATION_GUIDE.md)** - Robot calibration procedures
- **[Security Summary](Navigation/SECURITY_SUMMARY.md)** - Security audit findings

---

## 🛡️ Safety Notice

**⚠️ THIS IS A SAFETY-CRITICAL SYSTEM**

Before operating the Ultrabot AGV:
1. **READ** [Navigation/SAFETY.md](Navigation/SAFETY.md) completely
2. **COMPLETE** pre-flight safety checklist
3. **VERIFY** emergency stop functionality
4. **ENSURE** Safety Supervisor is running before Drive Node
5. **OBTAIN** operator training and authorization

**Non-compliance with safety procedures may result in:**
- Equipment damage
- Personnel injury
- Regulatory non-compliance
- Certification invalidation

---

## 🔐 Security

Ultrabot implements IEC 62443 cybersecurity standards:

- **Encrypted Communication:** DDS security with SROS2
- **Node Authentication:** X.509 certificates per node
- **Access Control:** Permission-based topic access
- **Parameter Security:** HMAC-authenticated certified parameters
- **Audit Logging:** 5-year retention for compliance

**For complete security setup, see:** [Navigation/SROS2_GUIDE.md](Navigation/SROS2_GUIDE.md)

---

## 🧪 Testing & Quality Assurance

### Automated Testing

```bash
# Run all tests
cd ~/ultrabot_ws
colcon test --packages-select somanet

# View test results
colcon test-result --verbose

# Generate coverage report
colcon test --packages-select somanet --event-handlers console_direct+ \
  --pytest-args --cov=somanet --cov-report=html
```

**Test Coverage:**
- 6 unit/integration tests (C++ GTest)
- Lifecycle driver coverage with watchdog, odometry, and maintenance checks
- HTML coverage reports generated via `--cov-report=html`

- 95%+ code coverage
- Lifecycle driver coverage with watchdog + odometry checks
- Odometry validation scripts

### Code Quality

```bash
# Format code
clang-format -i Navigation/src/*.cpp Navigation/include/**/*.hpp

# Static analysis
cppcheck --enable=all --inconclusive Navigation/src/

# Lint checks
colcon test --packages-select somanet --event-handlers console_direct+
```

---

## 📜 Compliance & Standards

| Standard | Title | Status |
|----------|-------|--------|
| **ISO 13849-1:2023** | Safety of machinery - Safety-related parts of control systems | ✅ Category 3, PL d |
| **ISO 3691-4:2023** | Industrial trucks - Safety requirements (AGVs) | ✅ Manual mode compliant |
| **IEC 61508:2010** | Functional safety of electrical/electronic systems | ✅ SIL 1 |
| **IEC 62443-3-3** | System security requirements and security levels | 🔄 In progress (v0.3) |
| **IEC 62443-4-2** | Technical security requirements for components | 🔄 In progress (v0.3) |
| **EN 61800-5-2:2016** | Adjustable speed electrical power drive systems - Safety (STO) | ✅ Hardware E-stop |
| **ISO 13850:2015** | Emergency stop function - Principles for design | ✅ <100ms response |
| **ISO 14971:2019** | Medical devices - Risk management | 🔄 Risk file maintenance |
| **IEC 62304:2006** | Medical device software - Software lifecycle processes | 🔄 Documentation ongoing |

✅ = Implemented | 🔄 = In Progress | 📅 = Planned

---

## 🤝 Contributing

We welcome contributions to the Ultrabot project! Please follow these guidelines:

### Code Style
- **C++:** Google C++ Style Guide (enforced by clang-format)
- **Python:** PEP 8 (enforced by flake8)
- **ROS 2:** [ROS 2 Developer Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Developer-Guide.html)

### Pull Request Process
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit changes (`git commit -m 'Add amazing feature'`)
4. Format code (`clang-format -i <files>`)
5. Run tests (`colcon test --packages-select somanet`)
6. Push to branch (`git push origin feature/amazing-feature`)
7. Open a Pull Request

### Safety-Critical Changes
For changes affecting safety functions:
1. Document risk assessment
2. Update SAFETY.md if procedures change
3. Request review from Safety Officer
4. Update compliance documentation

---

## 📞 Support

### Documentation
- Check [docs/](docs/) directory for guides
- See [Navigation/README.md](Navigation/README.md) for package details
- Review [docs/FRAMEWORKS_QUICK_REFERENCE.md](docs/FRAMEWORKS_QUICK_REFERENCE.md) for commands

### Community
- **ROS Discourse:** [https://discourse.ros.org/](https://discourse.ros.org/)
- **GitHub Issues:** [Repository Issues](https://github.com/geralmedrobots/ultrabot_coding_robtic/issues)
- **Stack Overflow:** Tag `ros2` + `ultrabot`

### Emergency
- **Safety Officer:** See [Navigation/SAFETY.md](Navigation/SAFETY.md) for contacts
- **Emergency Services:** 112 (EU) / 911 (US)

---

## 📝 License

[To be specified by organization]

See [LICENSE](LICENSE) file for details.

---

## ✍️ Authors & Acknowledgments

- **Ultrabot Team** - Initial development and ROS 2 migration
- **GitHub Copilot** - Refactoring, documentation, and safety enhancements (2025)

### Third-Party Components
- **ROS 2 Humble** - Open Robotics
- **Nav2** - Samsung Research, Steve Macenski
- **SLAM Toolbox** - Steve Macenski
- **Open-RMF** - Open Robotics, Singapore-MIT Alliance for Research and Technology (SMART)
- **SOEM (Simple Open EtherCAT Master)** - Arthur Ketels and contributors

---

## 📅 Version History

| Version | Date | Description |
|---------|------|-------------|
| **0.2** | 2025-11 (Current) | Nav2 integration, SLAM/AMCL, diagnostics, code quality |
| **0.1** | 2025-10 | EtherCAT control, safety supervisor, parameter validation |

**Current Status:** v0.2 Development (Foundation Phase)

**Next Release:** v0.3 (Q2 2026) - Fleet management, sensor fusion, simulation, cybersecurity

---

## 🔗 Related Resources

### Official Documentation
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Gazebo Documentation](https://gazebosim.org/docs)
- [Open-RMF Documentation](https://osrf.github.io/ros2multirobotbook/)

### Standards & Compliance
- [ISO 13849-1:2023 (Safety of machinery)](https://www.iso.org/standard/69883.html)
- [ISO 3691-4:2023 (AGV safety)](https://www.iso.org/standard/75568.html)
- [IEC 62443 (Industrial cybersecurity)](https://www.isa.org/standards-and-publications/isa-standards/isa-iec-62443-series-of-standards)
- [FDA Cybersecurity Guidance (2023)](https://www.fda.gov/medical-devices/digital-health-center-excellence/cybersecurity)

### Hardware References
- [Somanet EtherCAT Drives](https://www.synapticon.com/)
- [SICK LiDAR Sensors](https://www.sick.com/)

---

**⚠️ SAFETY REMINDER:** Always follow procedures in [Navigation/SAFETY.md](Navigation/SAFETY.md) before operating the Ultrabot AGV.

---

**Last Updated:** 2025-11-05 | **Maintainer:** Ultrabot Team | **Website:** [To be added]
