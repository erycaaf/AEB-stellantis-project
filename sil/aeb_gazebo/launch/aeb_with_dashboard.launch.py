"""
AEB with Dashboard Launch
=========================
Launches the full AEB scenario. Kept under the historic name
`aeb_with_dashboard.launch.py` so external callers (`run.sh`,
`launcher.sh`, contributor muscle memory) keep working, but this
launch is now identical to `aeb_scenario.launch.py` — the legacy
matplotlib `dashboard_node` was removed once the web dashboard at
http://localhost:3000 (sil/docker/dashboard/index.html) became the
supported visualisation for the SIL stack.

Usage:
  ros2 launch aeb_gazebo aeb_with_dashboard.launch.py scenario:=ccrs_40
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('aeb_gazebo')

    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='ccrs_40',
        description='Scenario name'
    )

    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'aeb_scenario.launch.py')
        ),
        launch_arguments={'scenario': LaunchConfiguration('scenario')}.items()
    )

    return LaunchDescription([
        scenario_arg,
        main_launch,
    ])
