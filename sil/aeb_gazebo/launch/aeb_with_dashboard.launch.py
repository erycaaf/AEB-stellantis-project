"""
AEB with Dashboard Launch
=========================
Launches the full AEB scenario + the real-time dashboard window.

Usage:
  ros2 launch aeb_gazebo aeb_with_dashboard.launch.py scenario:=ccrs_40
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('aeb_gazebo')

    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='ccrs_40',
        description='Scenario name'
    )

    # Include the main AEB scenario launch
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'aeb_scenario.launch.py')
        ),
        launch_arguments={'scenario': LaunchConfiguration('scenario')}.items()
    )

    # Dashboard node
    dashboard = Node(
        package='aeb_gazebo',
        executable='dashboard_node.py',
        name='dashboard',
        output='screen',
    )

    return LaunchDescription([
        scenario_arg,
        main_launch,
        dashboard,
    ])
