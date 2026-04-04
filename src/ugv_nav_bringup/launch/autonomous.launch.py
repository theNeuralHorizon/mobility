"""Full autonomous launch for UGV Navigation Challenge.

Launches:
  1. Robot State Publisher (URDF → TF)
  2. ROS-Gazebo bridge (sensor/cmd topics)
  3. Perception nodes (ArUco detector, sign detector)
  4. Mission controller (state machine brain)

NOTE: Gazebo must be started separately (headless or GUI) and the robot
must be spawned before running this launch file.

Usage:
  # Terminal 1: Start Gazebo
  ign gazebo -r -s ~/mobility_ws/install/ugv_nav_bringup/share/ugv_nav_bringup/worlds/arena.sdf

  # Terminal 2: Spawn robot
  xacro ... > /tmp/mini_r1.urdf && ign sdf -p ... > /tmp/mini_r1.sdf
  ros2 run ros_gz_sim create -file /tmp/mini_r1.sdf -name mini_r1 ...

  # Terminal 3: Launch everything else
  ros2 launch ugv_nav_bringup autonomous.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory("mini_r1_v1_description")
    sim_pkg = get_package_share_directory("mini_r1_v1_gz")

    bridge_config = os.path.join(sim_pkg, "config", "ros_gz_bridge.yaml")

    # Robot State Publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "use_sim_time": True,
            "robot_description": open(
                os.path.join(desc_pkg, "urdf", "mini_r1.urdf.xacro")
            ).read() if False else "",  # placeholder, xacro handled below
        }],
    )

    # Use xacro via the existing RSP launch
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, "launch", "rsp.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # ROS-Gazebo Bridge (supports both Fortress/ignition and Harmonic/gz)
    # Detect which message prefix to use
    import subprocess
    gz_version = subprocess.run(
        ["gz", "sim", "--version"], capture_output=True, text=True
    )
    if "version 8" in gz_version.stdout or "version 9" in gz_version.stdout:
        gz_prefix = "gz.msgs"  # Gazebo Harmonic+
    else:
        gz_prefix = "ignition.msgs"  # Gazebo Fortress

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"/cmd_vel@geometry_msgs/msg/Twist@{gz_prefix}.Twist",
            f"/r1_mini/odom@nav_msgs/msg/Odometry[{gz_prefix}.Odometry",
            f"/r1_mini/lidar@sensor_msgs/msg/LaserScan[{gz_prefix}.LaserScan",
            f"/r1_mini/imu@sensor_msgs/msg/Imu[{gz_prefix}.IMU",
            f"/r1_mini/camera@sensor_msgs/msg/Image[{gz_prefix}.Image",
            f"/clock@rosgraph_msgs/msg/Clock[{gz_prefix}.Clock",
            f"/r1_mini/joint_states@sensor_msgs/msg/JointState[{gz_prefix}.Model",
        ],
        output="screen",
    )

    # Static TF: odom -> base_link (until proper odom TF bridge works)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    )

    # ArUco Detector
    aruco = Node(
        package="ugv_perception",
        executable="aruco_detector",
        output="screen",
    )

    # Sign Detector
    sign = Node(
        package="ugv_perception",
        executable="sign_detector",
        output="screen",
    )

    # Mission Controller (the brain)
    mission = Node(
        package="ugv_perception",
        executable="mission_controller",
        output="screen",
    )

    return LaunchDescription([
        rsp_launch,
        bridge,
        static_tf,
        aruco,
        sign,
        mission,
    ])
