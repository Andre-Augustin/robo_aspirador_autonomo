import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Caminhos dos pacotes
    pkg_my_bot = get_package_share_directory('my_bot_controller')
    pkg_ydlidar = get_package_share_directory('ydlidar_ros2_driver')

    # 1. CONFIGURAÇÃO DO LIDAR (Fase 2)
    # Como o X3 não tem arquivo próprio na sua pasta, usamos o X4.yaml
    # Ambos operam geralmente em 128000 baudrate.
    lidar_config = os.path.join(pkg_ydlidar, 'params', 'X4.yaml')

    # 2. CONFIGURAÇÃO DO SLAM (Fase 3)
    # Aponta para o arquivo que está dentro da pasta 'config' do seu pacote
    slam_config = os.path.join(pkg_my_bot, 'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        
        # --- TF ESTÁTICO (Conexão Física) ---
        # Define onde o laser está em relação ao centro do robô (base_link)
        # Ajuste '0.1' (x) e '0.05' (z) conforme a posição real no seu chassi
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # --- SEU DRIVER DE MOTORES (Odometria) ---
        Node(
            package='my_bot_controller',
            executable='driver_motores',
            name='motor_driver',
            output='screen'
        ),

        # --- DRIVER DO LIDAR X3 (Usando perfil X4) ---
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_driver',
            output='screen',
            emulate_tty=True,
            parameters=[lidar_config] # Carrega os parametros completos do X4
        ),

        # --- SLAM TOOLBOX (Mapeamento) ---
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config]
        ),
    ])