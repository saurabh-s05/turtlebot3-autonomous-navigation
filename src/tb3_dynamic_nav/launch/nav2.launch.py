from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.descriptions import ParameterFile

LOCALIZATION_NODES = ['map_server', 'amcl']
NAVIGATION_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'collision_monitor',
    'bt_navigator',
    'waypoint_follower',
]

LIFECYCLE_MANAGER_PARAMS = {
    'autostart': True,
    'bond_timeout': 10.0,
    'attempt_respawn_reconnection': True,
    'bond_respawn_max_duration': 120.0,
}


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = ParameterFile(params_file, allow_substs=True)

    def nav_node(package, executable, remappings=(), extra_params=None):
        return Node(
            package=package, executable=executable, name=executable, output='screen',
            respawn=True, respawn_delay=2.0,
            parameters=[params] + ([extra_params] if extra_params else []),
            remappings=list(remappings))

    cmd_vel_nav = [('cmd_vel', 'cmd_vel_nav')]

    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        GroupAction([
            SetParameter('use_sim_time', use_sim_time),

            nav_node('nav2_map_server', 'map_server', extra_params={'yaml_filename': map_yaml}),
            nav_node('nav2_amcl', 'amcl'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[LIFECYCLE_MANAGER_PARAMS, {'node_names': LOCALIZATION_NODES}]),

            nav_node('nav2_controller', 'controller_server', cmd_vel_nav),
            nav_node('nav2_smoother', 'smoother_server'),
            nav_node('nav2_planner', 'planner_server'),
            nav_node('nav2_behaviors', 'behavior_server', cmd_vel_nav),
            nav_node('nav2_velocity_smoother', 'velocity_smoother', cmd_vel_nav),
            nav_node('nav2_collision_monitor', 'collision_monitor'),
            nav_node('nav2_bt_navigator', 'bt_navigator'),
            nav_node('nav2_waypoint_follower', 'waypoint_follower'),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[LIFECYCLE_MANAGER_PARAMS, {'node_names': NAVIGATION_NODES}]),

            Node(package='tb3_dynamic_nav', executable='nav_watchdog', name='nav_watchdog',
                 output='screen',
                 parameters=[{'localization_nodes': LOCALIZATION_NODES,
                              'navigation_nodes': NAVIGATION_NODES}]),
        ]),
    ])
