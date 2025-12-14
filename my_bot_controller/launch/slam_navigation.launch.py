import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_my_bot = get_package_share_directory('my_bot_controller')
    pkg_ydlidar = get_package_share_directory('ydlidar_ros2_driver')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox')

    lidar_config = os.path.join(pkg_ydlidar, 'params', 'X4.yaml')
    nav2_params = os.path.join(pkg_my_bot, 'config', 'nav2_params.yaml')
    slam_config = os.path.join(pkg_my_bot, 'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        
        # 1. HARDWARE (TF, Motores, Lidar)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame']
        ),
        Node(
            package='my_bot_controller',
            executable='driver_motores',
            name='motor_driver',
            output='screen'
        ),
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_driver',
            output='screen',
            emulate_tty=True,
            parameters=[lidar_config]
        ),

        # 2. CÉREBRO SISTEMA (SLAM + Nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
            launch_arguments={'slam_params_file': slam_config, 'use_sim_time': 'false'}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={'params_file': nav2_params, 'use_sim_time': 'false', 'autostart': 'true'}.items()
        ),

        # 3. CÉREBRO TAREFA (Aqui está a mudança!)
        # Usamos um TimerAction para dar 10 segundos para o Nav2 carregar antes de iniciar o script
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='my_bot_controller',
                    executable='coverage_demo',  # Nome definido no setup.py (entry_points)
                    name='coverage_logic',
                    output='screen'
                )
            ]
        )
    ])