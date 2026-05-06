"""
CCRm Scenario Launcher
======================
Launch CCRm (Car-to-Car Rear Moving): ego 50 km/h, target 20 km/h.

Usage:
  ros2 launch aeb_gazebo ccrm.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('aeb_gazebo')

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'aeb_scenario.launch.py')
        ),
        launch_arguments={'scenario': 'ccrm'}.items()
    )

    return LaunchDescription([main_launch])
