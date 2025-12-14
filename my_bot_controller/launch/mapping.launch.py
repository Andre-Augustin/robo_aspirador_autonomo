import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Caminho para o config do SLAM (Fase 3)
    slam_config = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    )

    return LaunchDescription([
        
        # 1. TF DO LIDAR (Fase 2)
        # Ajuste o primeiro número '0.1' (10cm) para a distância real entre
        # o centro das rodas e onde o LiDAR está colado.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # 2. SEU DRIVER DE MOTORES (Fase 1 - O arquivo que você enviou)
        # IMPORTANTE: Verifique no seu setup.py qual nome você deu ao entry_point
        Node(
            package='meu_robo',          # Nome da pasta do seu pacote
            executable='driver_motores', # Nome do executável definido no setup.py
            name='motor_driver',
            output='screen'
        ),

        # 3. DRIVER DO LIDAR (Fase 2)
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_driver',
            parameters=[{'port': '/dev/ttyUSB0', 'frame_id': 'laser_frame'}]
        ),

        # 4. SLAM TOOLBOX (Fase 3 - O cérebro do mapa)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config]
        ),
    ])