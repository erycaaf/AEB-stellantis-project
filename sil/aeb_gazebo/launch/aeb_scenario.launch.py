"""
AEB Scenario Launch File
========================
Launches Gazebo with the AEB highway world, spawns ego + target vehicles,
and starts the AEB controller and scenario controller nodes.

Usage:
  ros2 launch aeb_gazebo aeb_scenario.launch.py scenario:=ccrs_40
  ros2 launch aeb_gazebo aeb_scenario.launch.py scenario:=ccrm
  ros2 launch aeb_gazebo aeb_scenario.launch.py scenario:=ccrb_d2_g40
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# Scenario parameter lookup
SCENARIOS = {
    'ccrs_20':   {'ego': 20.0, 'target': 0.0,  'gap': 100.0, 'decel': 0.0,  'brake_t': 0.0},
    'ccrs_30':   {'ego': 30.0, 'target': 0.0,  'gap': 100.0, 'decel': 0.0,  'brake_t': 0.0},
    'ccrs_40':   {'ego': 40.0, 'target': 0.0,  'gap': 100.0, 'decel': 0.0,  'brake_t': 0.0},
    'ccrs_50':        {'ego': 50.0, 'target': 0.0,  'gap': 100.0, 'decel': 0.0,  'brake_t': 0.0},
    'ccrs_50_no_aeb': {'ego': 50.0, 'target': 0.0,  'gap': 100.0, 'decel': 0.0,  'brake_t': 0.0, 'aeb_enabled': False},
    'ccrm':      {'ego': 50.0, 'target': 20.0, 'gap': 100.0, 'decel': 0.0,  'brake_t': 0.0},
    'ccrb_d2_g12': {'ego': 50.0, 'target': 50.0, 'gap': 12.0, 'decel': -2.0, 'brake_t': 2.0},
    'ccrb_d2_g40': {'ego': 50.0, 'target': 50.0, 'gap': 40.0, 'decel': -2.0, 'brake_t': 2.0},
    'ccrb_d6_g12': {'ego': 50.0, 'target': 50.0, 'gap': 12.0, 'decel': -6.0, 'brake_t': 2.0},
    'ccrb_d6_g40': {'ego': 50.0, 'target': 50.0, 'gap': 40.0, 'decel': -6.0, 'brake_t': 2.0},
}


def _launch_nodes(context):
    """Resolve scenario name at runtime and pass correct parameters."""
    scenario_name = context.launch_configurations['scenario']
    params = SCENARIOS.get(scenario_name)
    if params is None:
        raise RuntimeError(
            f"Cenário desconhecido: '{scenario_name}'. "
            f"Opções válidas: {', '.join(sorted(SCENARIOS.keys()))}"
        )

    pkg_share = get_package_share_directory('aeb_gazebo')
    world_file = os.path.join(pkg_share, 'worlds', 'aeb_highway.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items()
    )

    scenario_controller = Node(
        package='aeb_gazebo',
        executable='scenario_controller.py',
        name='scenario_controller',
        output='screen',
        parameters=[{
            'scenario': scenario_name,
            'ego_speed_kmh': params['ego'],
            'target_speed_kmh': params['target'],
            'initial_gap_m': params['gap'],
            'target_decel': params['decel'],
            'target_brake_time': params['brake_t'],
            # Pass world spawn x so scenario_controller skips the redundant teleport.
            # The target already spawns at x=100 in aeb_highway.world; calling
            # set_entity_state to x=100 again gives the vehicle a physics impulse
            # that makes it drift forward for ~3s, delaying the WARNING trigger.
            'skip_teleport': (abs(params['gap'] - 100.0) < 0.1),
            'aeb_enabled': params.get('aeb_enabled', True),
        }]
    )

    perception_node = Node(
        package='aeb_gazebo',
        executable='perception_node.py',
        name='perception',
        output='screen',
    )

    nodes = [gazebo, scenario_controller, perception_node]

    # AEB control logic runs on the Zephyr ECU (native_sim, aeb-ecu container),
    # which receives sensor data via CAN from perception_node and emits
    # brake commands back via CAN to scenario_controller.

    return nodes


def generate_launch_description():
    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='ccrs_40',
        description='Scenario name: ccrs_20/30/40/50, ccrm, ccrb_d2_g12/g40, ccrb_d6_g12/g40'
    )

    return LaunchDescription([
        scenario_arg,
        OpaqueFunction(function=_launch_nodes),
    ])
