#!/bin/bash
# Install Dependencies for GAP 2: Immutable Parameters
# ISO 13849-1 §5.2.2 Compliance

set -euo pipefail

echo "═══════════════════════════════════════════════════════════════"
echo "  Installing ROS 2 + Certified Parameter Validation Dependencies"
echo "  (ISO 13849-1 §5.2.2 - Immutable Safety Parameters)"
echo "═══════════════════════════════════════════════════════════════"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
  echo "❌ Please run as root (sudo)"
  exit 1
fi

APT_LOG=/tmp/somanet_apt.$$.log

check_apt()
{
  echo ""
  echo "🔍 Updating apt package index (logs: $APT_LOG)"
  if ! apt-get update >"$APT_LOG" 2>&1; then
    echo "❌ apt-get update failed"
    echo "   ↳ Inspect $APT_LOG for proxy/firewall errors (HTTP 403 is common in sandboxes)"
    echo "   ↳ Ensure outbound access to archive.ubuntu.com and security.ubuntu.com"
    exit 1
  fi
}

install_packages()
{
  local description="$1"
  shift
  echo ""
  echo "📦 Installing ${description}..."
  if ! DEBIAN_FRONTEND=noninteractive apt-get install -y "$@" >>"$APT_LOG" 2>&1; then
    echo "❌ Failed to install ${description}"
    echo "   ↳ Inspect $APT_LOG for details"
    exit 1
  fi
  echo "✅ ${description} installed successfully"
}

configure_ros_repository()
{
  . /etc/os-release
  case "$UBUNTU_CODENAME" in
    jammy)
      ROS_DISTRO=humble
      ;;
    noble)
      ROS_DISTRO=jazzy
      ;;
    *)
      echo "⚠️ Unsupported Ubuntu release: $UBUNTU_CODENAME"
      echo "   ↳ Continuing without configuring a ROS 2 apt repository"
      return 1
      ;;
  esac

  echo ""
  echo "📚 Configuring ROS 2 $ROS_DISTRO repository for Ubuntu $UBUNTU_CODENAME"
  install_packages "base apt prerequisites" \
    software-properties-common \
    ca-certificates \
    curl \
    gnupg

  local KEYRING=/etc/apt/keyrings/ros-archive-keyring.gpg
  mkdir -p "$(dirname "$KEYRING")"
  if ! curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
      | gpg --dearmor -o "$KEYRING"; then
    echo "❌ Failed to fetch ROS 2 GPG key"
    echo "   ↳ Check network access to raw.githubusercontent.com"
    exit 1
  fi

  echo "deb [arch=$(dpkg --print-architecture) signed-by=$KEYRING] \
http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
    >/etc/apt/sources.list.d/ros2.list

  check_apt

  install_packages "ROS 2 ${ROS_DISTRO^} base stack" \
    "ros-$ROS_DISTRO-ros-base" \
    "ros-$ROS_DISTRO-soem" \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

  echo ""
  echo "🔧 Initialising rosdep database"
  if ! rosdep init >>"$APT_LOG" 2>&1; then
    echo "⚠️ rosdep init reported an error (often harmless if already initialised)"
  fi
  if ! rosdep update >>"$APT_LOG" 2>&1; then
    echo "⚠️ rosdep update failed; please re-run once network access is available"
  fi

  export ROS_DISTRO
  return 0
}

check_apt

configure_ros_repository || true

install_packages "OpenSSL development libraries (SHA-256 hashing)" libssl-dev

install_packages "yaml-cpp library (YAML parsing)" libyaml-cpp-dev

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo "✅ All dependencies installed successfully!"
echo "═══════════════════════════════════════════════════════════════"
echo ""
echo "📝 Next Steps:"
echo "   1. Build the package:"
echo "      cd ~/ultrabot_ws"
echo "      colcon build --packages-select somanet"
echo ""
echo "   2. Generate certified parameters hash:"
echo "      python3 scripts/generate_certification_hash.py \\"
echo "              config/certified_safety_params.yaml"
echo ""
echo "   3. Test the system:"
echo "      ros2 run somanet safety_supervisor_node --autostart"
echo ""
echo "   4. Try to modify a safety parameter (should be rejected):"
echo "      ros2 param set /safety_supervisor max_linear_velocity 10.0"
echo ""
