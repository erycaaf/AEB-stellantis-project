"""
CCRs Scenario Launcher
======================
Shortcut to launch CCRs (Car-to-Car Rear Stationary).

Usage:
  ros2 launch aeb_gazebo ccrs.launch.py
  ros2 launch aeb_gazebo ccrs.launch.py ego_speed:=50.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def _resolve_scenario(context):
    ego_speed = int(float(context.launch_configurations['ego_speed']))
    scenario_name = f'ccrs_{ego_speed}'

    pkg_share = get_package_share_directory('aeb_gazebo')
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'aeb_scenario.launch.py')
        ),
        launch_arguments={'scenario': scenario_name}.items()
    )
    return [main_launch]


def generate_launch_description():
    ego_speed_arg = DeclareLaunchArgument('ego_speed', default_value='40.0')

    return LaunchDescription([
        ego_speed_arg,
        OpaqueFunction(function=_resolve_scenario),
    ])
