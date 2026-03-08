#!/bin/bash
# AUV Development Environment Setup
# Run once after cloning the repo on a fresh Pi 5 (Ubuntu 24.04 + ROS2 Jazzy).
# Usage: bash scripts/setup_dev_environment.sh

set -e
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "=== AUV Dev Environment Setup ==="
echo "Repo: $REPO_ROOT"

# ── 1. Source ROS2 ────────────────────────────────────────────────────────────
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ERROR: ROS2 Jazzy not found at /opt/ros/jazzy"
    echo "Install ROS2 Jazzy first: https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi
source /opt/ros/jazzy/setup.bash
echo "[✓] ROS2 Jazzy sourced"

# ── 2. Install micro-ROS agent ────────────────────────────────────────────────
echo "Installing micro_ros_agent..."
if ! ros2 pkg list | grep -q micro_ros_agent 2>/dev/null; then
    sudo apt-get install -y ros-jazzy-micro-ros-agent
fi
echo "[✓] micro_ros_agent installed"

# ── 3. Install colcon and Python deps ─────────────────────────────────────────
sudo apt-get install -y python3-colcon-common-extensions python3-numpy
echo "[✓] colcon and numpy installed"

# ── 4. Build the ROS2 workspace ───────────────────────────────────────────────
echo "Building ros2_ws..."
cd "$REPO_ROOT/ros2_ws"
colcon build --symlink-install
echo "[✓] Workspace built"

# ── 5. Add dialout group (for serial ports) ───────────────────────────────────
if ! groups "$USER" | grep -q dialout; then
    sudo usermod -aG dialout "$USER"
    echo "[✓] Added $USER to dialout group (re-login required)"
else
    echo "[✓] $USER already in dialout group"
fi

# ── 6. Deploy udev rules ──────────────────────────────────────────────────────
UDEV_SRC="$REPO_ROOT/config/udev/99-auv-teensy.rules"
UDEV_DST="/etc/udev/rules.d/99-auv-teensy.rules"
echo ""
echo "NOTE: The udev rules file contains placeholder serial numbers."
echo "      Edit $UDEV_SRC first (see instructions inside),"
echo "      then run this script again or manually:"
echo "        sudo cp $UDEV_SRC $UDEV_DST"
echo "        sudo udevadm control --reload-rules && sudo udevadm trigger"
echo ""

if grep -q "<SENSOR_HUB_SERIAL>" "$UDEV_SRC"; then
    echo "[!] Skipping udev deploy — placeholder serials not filled in yet."
else
    sudo cp "$UDEV_SRC" "$UDEV_DST"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "[✓] udev rules deployed"
fi

# ── 7. Shell environment setup ────────────────────────────────────────────────
BASHRC="$HOME/.bashrc"
if ! grep -q "ROS_DOMAIN_ID=42" "$BASHRC"; then
    cat >> "$BASHRC" << 'EOF'

# AUV ROS2 environment
source /opt/ros/jazzy/setup.bash
source /home/pi/repos/claude_AUV/ros2_ws/install/setup.bash 2>/dev/null || true
export ROS_DOMAIN_ID=42
EOF
    echo "[✓] Added ROS2 sourcing and ROS_DOMAIN_ID=42 to ~/.bashrc"
else
    echo "[✓] ~/.bashrc already configured"
fi

echo ""
echo "=== Setup complete! ==="
echo "Next steps:"
echo "  1. Fill in Teensy serial numbers in config/udev/99-auv-teensy.rules"
echo "  2. Re-run this script to deploy udev rules"
echo "  3. Re-login (or newgrp dialout) for serial port access"
echo "  4. Source your shell: source ~/.bashrc"
echo "  5. Flash firmware: scripts/flash_sensor_hub.sh"
