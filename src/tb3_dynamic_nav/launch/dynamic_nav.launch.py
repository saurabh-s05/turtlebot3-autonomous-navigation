import os
import signal

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def stop_gz_servers(partition):
    # gz sim keeps running after launch stops its ruby wrapper
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                if not f.read().startswith(b'gz\0sim'):
                    continue
            with open(f'/proc/{pid}/environ', 'rb') as f:
                if f'GZ_PARTITION={partition}'.encode() in f.read().split(b'\0'):
                    os.kill(int(pid), signal.SIGTERM)
        except OSError:
            pass


def generate_launch_description():
    pkg = get_package_share_directory('tb3_dynamic_nav')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch = os.path.join(tb3_gazebo, 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    params = LaunchConfiguration('params_file')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    patrol = LaunchConfiguration('patrol')
    move_obstacles = LaunchConfiguration('move_obstacles')
    obstacle_speed = LaunchConfiguration('obstacle_speed')

    args = [
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'dynamic_world.sdf')),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'clean_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_dynamic.yaml')),
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz'),
        DeclareLaunchArgument('patrol', default_value='false', description='Run the waypoint patrol'),
        DeclareLaunchArgument('move_obstacles', default_value='true'),
        DeclareLaunchArgument('obstacle_speed', default_value='1.0',
                              description='Speed scale for moving obstacles'),
    ]

    # own partition per launch so a stale gz server from an earlier run can't interfere
    partition = os.environ.get('TB3_GZ_PARTITION', f'tb3_dynamic_nav_{os.getpid()}')

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items())

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(gui))

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_launch, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    # spawn pose has to match the origin of the saved map
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_launch, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': '-2.0', 'y_pose': '-0.5'}.items())

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'nav2.launch.py')),
        launch_arguments={'map': map_yaml, 'params_file': params, 'use_sim_time': 'true'}.items())

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(pkg, 'rviz', 'tb3_navigation2.rviz')],
        parameters=[{'use_sim_time': True}], condition=IfCondition(rviz))

    obstacles_node = Node(
        package='tb3_dynamic_nav', executable='dynamic_obstacles', output='screen',
        parameters=[{'use_sim_time': True, 'speed_scale': obstacle_speed}],
        condition=IfCondition(move_obstacles))

    patrol_node = Node(
        package='tb3_dynamic_nav', executable='patrol', output='screen',
        parameters=[{'use_sim_time': True}], condition=IfCondition(patrol))

    return LaunchDescription(args + [
        RegisterEventHandler(OnShutdown(on_shutdown=lambda event, context: stop_gz_servers(partition))),
        LogInfo(msg=f'GZ_PARTITION={partition}'),
        SetEnvironmentVariable('GZ_PARTITION', partition),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3_gazebo, 'models')),
        gz_server,
        gz_client,
        robot_state_publisher,
        spawn_robot,
        TimerAction(period=5.0, actions=[nav2, rviz_node, obstacles_node]),
        TimerAction(period=12.0, actions=[patrol_node]),
    ])
