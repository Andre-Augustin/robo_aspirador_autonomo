import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_my_bot = get_package_share_directory('my_bot_controller')
    pkg_ydlidar = get_package_share_directory('ydlidar_ros2_driver')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    # Arquivos
    lidar_config = os.path.join(pkg_ydlidar, 'params', 'X4.yaml') # Perfil do Lidar
    nav2_params = os.path.join(pkg_my_bot, 'config', 'nav2_params.yaml')
    
    # ATENÇÃO: Este arquivo ainda não existe! O launch vai falhar se rodar agora.
    # Mas quando você salvar o mapa com esse nome, funcionará.
    map_file = os.path.join(pkg_my_bot, 'config', 'meu_mapa.yaml')

    return LaunchDescription([
        
        # 1. HARDWARE: TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame']
        ),
        
        # 2. HARDWARE: Motores
        Node(
            package='my_bot_controller',
            executable='driver_motores',
            name='motor_driver',
            output='screen'
        ),

        # 3. HARDWARE: Lidar
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_driver',
            output='screen',
            emulate_tty=True,
            parameters=[lidar_config]
        ),

        # 4. CÉREBRO: Nav2 (Jazzy)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'params_file': nav2_params,
                'use_sim_time': 'false', 
                'autostart': 'true'
            }.items()
        )
    ])