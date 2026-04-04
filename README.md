# UGV Autonomous Navigation Challenge

ROS 2 Humble + Nav2 + Gazebo simulation for map-less autonomous navigation with ArUco marker detection and directional sign interpretation.

## Prerequisites

- Windows 11 with WSL2 enabled
- Ubuntu 22.04 installed in WSL2

### Install WSL2 + Ubuntu 22.04

Open PowerShell as Administrator:
```powershell
wsl --install -d Ubuntu-22.04
```

After installation, open Ubuntu from the Start menu and set up your user account.

## Quick Start

### 1. Install ROS 2 and dependencies

```bash
# Inside WSL2 Ubuntu terminal
cd /mnt/c/codespace/mobility
chmod +x setup_env.sh
./setup_env.sh
```

### 2. Set up workspace

```bash
# Create symlink for convenience
ln -s /mnt/c/codespace/mobility ~/mobility_ws

# Close and reopen terminal, then:
cd ~/mobility_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to .bashrc for persistence (uncomment the line the setup script added):
# Edit ~/.bashrc and uncomment: source ~/mobility_ws/install/setup.bash
```

### 3. Launch simulation

```bash
# Basic simulation (empty world)
ros2 launch mini_r1_v1_gz sim.launch.py

# With warehouse world
ros2 launch mini_r1_v1_gz sim.launch.py \
  world:=$(ros2 pkg prefix mini_r1_v1_description)/share/mini_r1_v1_description/worlds/warehouse.sdf

# Full challenge launch (sim + Nav2)
ros2 launch ugv_nav_bringup challenge.launch.py
```

### 4. Teleoperate the robot

```bash
# In a new terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 5. Run perception nodes

```bash
# ArUco marker detection
ros2 run ugv_perception aruco_detector

# Directional sign detection
ros2 run ugv_perception sign_detector
```

## Workspace Structure

```
src/
  mini_r1_v1_description/   # Robot URDF, meshes, sensor configs
  mini_r1_v1_gz/             # Gazebo simulation launch + bridge config
  ugv_nav_bringup/           # Challenge launch files + Nav2 config
  ugv_perception/            # ArUco + sign detection nodes
```

## ROS Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/r1_mini/lidar` | `sensor_msgs/LaserScan` | 2D LiDAR (360 samples, 30m range) |
| `/r1_mini/camera/image_raw` | `sensor_msgs/Image` | RGB camera (640x480) |
| `/r1_mini/imu` | `sensor_msgs/Imu` | IMU data |
| `/r1_mini/odom` | `nav_msgs/Odometry` | Wheel odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/ugv/aruco/detections` | `std_msgs/Int32MultiArray` | Detected ArUco IDs |
| `/ugv/sign/direction` | `std_msgs/String` | Detected sign direction |

## Challenge Rules

- **No SLAM** (no map creation)
- **No AMCL** (no global localization)
- **No hardcoded positions** or predefined trajectories
- Must use **local costmap + reactive planning** only
- Must detect all **4 ArUco markers** and reach the **correct goal zone**

## Next Steps

1. Design and build the custom arena world (`src/ugv_nav_bringup/worlds/arena.sdf`)
2. Implement sign detection logic in `sign_detector.py`
3. Build the reactive navigation behavior tree
4. Add recovery behaviors (rotate, backtrack, escape dead-end)
5. Test and tune in simulation
