"""
CCRb Scenario Launcher
======================
Launch CCRb (Car-to-Car Rear Braking).

Usage:
  ros2 launch aeb_gazebo ccrb.launch.py
  ros2 launch aeb_gazebo ccrb.launch.py decel:=-6.0 gap:=12.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def _resolve_scenario(context):
    decel = int(abs(float(context.launch_configurations['decel'])))
    gap = int(float(context.launch_configurations['gap']))
    scenario_name = f'ccrb_d{decel}_g{gap}'

    pkg_share = get_package_share_directory('aeb_gazebo')
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'aeb_scenario.launch.py')
        ),
        launch_arguments={'scenario': scenario_name}.items()
    )
    return [main_launch]


def generate_launch_description():
    decel_arg = DeclareLaunchArgument('decel', default_value='-2.0')
    gap_arg = DeclareLaunchArgument('gap', default_value='40.0')

    return LaunchDescription([
        decel_arg,
        gap_arg,
        OpaqueFunction(function=_resolve_scenario),
    ])
