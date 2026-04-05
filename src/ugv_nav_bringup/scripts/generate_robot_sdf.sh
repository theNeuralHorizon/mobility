#!/bin/bash
# Generate robot SDF with all plugins (sensors included)
# Usage: bash generate_robot_sdf.sh
# Output: /tmp/mini_r1.sdf
# Supports both Gazebo Fortress (ign) and Harmonic+ (gz)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$HOME/mobility_ws"

# Source ROS 2 (detect jazzy or humble)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

source "$WS_DIR/install/setup.bash"

# Generate base SDF from URDF
xacro "$WS_DIR/src/mini_r1_v1_description/urdf/mini_r1.urdf.xacro" > /tmp/mini_r1.urdf

# Detect Gazebo version: Harmonic uses `gz sdf`, Fortress uses `ign sdf`
if command -v gz &>/dev/null && gz sdf --help &>/dev/null; then
    echo "Using Gazebo Harmonic+ (gz sdf)"
    gz sdf -p /tmp/mini_r1.urdf > /tmp/mini_r1_base.sdf
else
    echo "Using Gazebo Fortress (ign sdf)"
    ign sdf -p /tmp/mini_r1.urdf > /tmp/mini_r1_base.sdf
fi

# Fix cmd_vel topic to absolute
sed -i 's|<topic>cmd_vel</topic>|<topic>/cmd_vel</topic>|' /tmp/mini_r1_base.sdf

MESH_DIR="$WS_DIR/install/mini_r1_v1_description/share/mini_r1_v1_description/meshes"

# Insert sensor plugins before the closing </model> tag
# We need: LiDAR sensor, Camera sensor, IMU sensor
python3 << 'PYEOF'
import re

with open("/tmp/mini_r1_base.sdf", "r") as f:
    sdf = f.read()

sensor_plugins = """
    <!-- ========== LiDAR Sensor ========== -->
    <plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'>
      <render_engine>ogre2</render_engine>
    </plugin>

    <plugin name='gz::sim::systems::Imu' filename='gz-sim-imu-system'>
    </plugin>
"""

# Find the LIDAR link and add a sensor to it
# The LIDAR is fixed to base_link, so we add sensor to base_link
lidar_sensor = """
      <!-- LiDAR Sensor -->
      <sensor name='lidar' type='gpu_lidar'>
        <pose>0.095 0 0.12 0 0 0</pose>
        <topic>/r1_mini/lidar</topic>
        <update_rate>10</update_rate>
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1</resolution>
              <min_angle>-3.14</min_angle>
              <max_angle>3.14</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.12</min>
            <max>30.0</max>
            <resolution>0.01</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0</mean>
            <stddev>0.005</stddev>
          </noise>
        </lidar>
        <always_on>true</always_on>
        <visualize>false</visualize>
      </sensor>

      <!-- Camera Sensor -->
      <sensor name='camera' type='camera'>
        <pose>0.1658 0.000503 0.078 0 0 0</pose>
        <topic>/r1_mini/camera</topic>
        <update_rate>15</update_rate>
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
        </camera>
        <always_on>true</always_on>
        <visualize>false</visualize>
      </sensor>

      <!-- IMU Sensor -->
      <sensor name='imu' type='imu'>
        <pose>0.095 0 0.08 0 0 0</pose>
        <topic>/r1_mini/imu</topic>
        <update_rate>50</update_rate>
        <imu>
          <angular_velocity>
            <x><noise><type>gaussian</type><mean>0</mean><stddev>0.0002</stddev></noise></x>
            <y><noise><type>gaussian</type><mean>0</mean><stddev>0.0002</stddev></noise></y>
            <z><noise><type>gaussian</type><mean>0</mean><stddev>0.0002</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise><type>gaussian</type><mean>0</mean><stddev>0.017</stddev></noise></x>
            <y><noise><type>gaussian</type><mean>0</mean><stddev>0.017</stddev></noise></y>
            <z><noise><type>gaussian</type><mean>0</mean><stddev>0.017</stddev></noise></z>
          </linear_acceleration>
        </imu>
        <always_on>true</always_on>
        <visualize>false</visualize>
      </sensor>
"""

# Insert sensors into the base_link (first <link> in the model)
# Find the first </link> and insert sensors before it
sdf = sdf.replace("    </link>\n    <joint", lidar_sensor + "    </link>\n    <joint", 1)

# Insert model-level sensor system plugin before </model>
sdf = sdf.replace("  </model>", sensor_plugins + "  </model>")

with open("/tmp/mini_r1.sdf", "w") as f:
    f.write(sdf)

print("Generated /tmp/mini_r1.sdf with all sensor plugins")
PYEOF

echo "Robot SDF generated at /tmp/mini_r1.sdf"
