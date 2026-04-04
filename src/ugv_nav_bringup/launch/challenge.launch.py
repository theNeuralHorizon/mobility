import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("ugv_nav_bringup")
    sim_pkg_dir = get_package_share_directory("mini_r1_v1_gz")
    desc_pkg_dir = get_package_share_directory("mini_r1_v1_description")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # --- Launch arguments ---------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    nav2_params = LaunchConfiguration("nav2_params")

    default_world = os.path.join(desc_pkg_dir, "worlds", "empty.sdf")
    default_nav2_params = os.path.join(bringup_dir, "config", "nav2_params.yaml")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("world", default_value=default_world,
                              description="Path to Gazebo world SDF"),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params,
                              description="Path to Nav2 parameter file"),
    ]

    # --- Simulation (reuse existing sim launch) -----------------------------
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg_dir, "launch", "sim.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    # --- Nav2 (local costmap only, no map server / AMCL) --------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
        }.items(),
    )

    return LaunchDescription([
        *declare_args,
        sim_launch,
        nav2_launch,
    ])
