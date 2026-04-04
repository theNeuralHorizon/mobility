#!/bin/bash
# =============================================================================
# UGV Navigation Challenge - Environment Setup Script
# Run this inside WSL2 Ubuntu 22.04
# Usage: chmod +x setup_env.sh && ./setup_env.sh
# =============================================================================

set -e

echo "=========================================="
echo " UGV Challenge Environment Setup"
echo "=========================================="

# ── 1. System prerequisites ──────────────────────────────────────────────────
echo "[1/6] Installing system prerequisites..."
sudo apt update && sudo apt install -y \
    software-properties-common \
    curl \
    gnupg \
    lsb-release \
    git \
    python3-pip \
    wget

# ── 2. ROS 2 Humble ─────────────────────────────────────────────────────────
echo "[2/6] Installing ROS 2 Humble..."
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop

# Build tools
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Initialize rosdep (skip if already initialized)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

# ── 3. Gazebo + ROS-Gazebo bridge ───────────────────────────────────────────
echo "[3/6] Installing Gazebo Sim + ROS bridge..."
sudo apt install -y \
    ros-humble-ros-gz \
    ros-humble-twist-stamper

# ── 4. Nav2 ─────────────────────────────────────────────────────────────────
echo "[4/6] Installing Nav2..."
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# ── 5. Perception dependencies ──────────────────────────────────────────────
echo "[5/6] Installing perception dependencies..."
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    python3-opencv \
    ros-humble-teleop-twist-keyboard

# ── 6. Shell configuration ──────────────────────────────────────────────────
echo "[6/6] Configuring shell..."

BASHRC="$HOME/.bashrc"

# Add ROS 2 sourcing if not already present
if ! grep -q "source /opt/ros/humble/setup.bash" "$BASHRC"; then
    echo "" >> "$BASHRC"
    echo "# ROS 2 Humble" >> "$BASHRC"
    echo "source /opt/ros/humble/setup.bash" >> "$BASHRC"
fi

if ! grep -q "mobility_ws/install/setup.bash" "$BASHRC"; then
    echo "# UGV Challenge workspace (uncomment after first build)" >> "$BASHRC"
    echo "# source ~/mobility_ws/install/setup.bash" >> "$BASHRC"
fi

# Source ROS 2 for the remainder of this script
source /opt/ros/humble/setup.bash

echo ""
echo "=========================================="
echo " Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Close and reopen your terminal (or run: source ~/.bashrc)"
echo "  2. Create a symlink to the workspace:"
echo "     ln -s /mnt/c/codespace/mobility ~/mobility_ws"
echo "  3. Build the workspace:"
echo "     cd ~/mobility_ws && colcon build --symlink-install"
echo "  4. Source the workspace:"
echo "     source ~/mobility_ws/install/setup.bash"
echo "  5. Launch the simulation:"
echo "     ros2 launch mini_r1_v1_gz sim.launch.py"
echo ""
