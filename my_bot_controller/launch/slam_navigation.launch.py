import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 1. DEFINIÇÃO DE PACOTES
    pkg_my_bot = get_package_share_directory('my_bot_controller')
    pkg_ydlidar = get_package_share_directory('ydlidar_ros2_driver')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_slam = get_package_share_directory('slam_toolbox') # <--- NOVO: Pacote do SLAM

    # 2. ARQUIVOS DE CONFIGURAÇÃO
    lidar_config = os.path.join(pkg_ydlidar, 'params', 'X4.yaml')
    nav2_params = os.path.join(pkg_my_bot, 'config', 'nav2_params.yaml')
    
    # <--- MUDANÇA: Em vez de carregar um mapa, carregamos a config do SLAM
    slam_config = os.path.join(pkg_my_bot, 'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        
        # --- HARDWARE: TF ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
            arguments=['0.1', '0', '0.05', '0', '0', '0', 'base_link', 'laser_frame']
        ),
        
        # --- HARDWARE: Motores ---
        Node(
            package='my_bot_controller',
            executable='driver_motores',
            name='motor_driver',
            output='screen'
        ),

        # --- HARDWARE: Lidar ---
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_driver',
            output='screen',
            emulate_tty=True,
            parameters=[lidar_config]
        ),

        # --- CÉREBRO PARTE 1: SLAM (Substitui o Map Server e AMCL) ---
        # Este nó cria o mapa ao vivo e publica no tópico /map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_slam, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_params_file': slam_config,
                'use_sim_time': 'false'
            }.items()
        ),

        # --- CÉREBRO PARTE 2: Navegação (Planejamento de Rota) ---
        # MUDANÇA CRÍTICA: Trocamos 'bringup_launch.py' por 'navigation_launch.py'
        # O 'bringup' tenta carregar mapa e AMCL. O 'navigation' só carrega o planejador,
        # aceitando o mapa que vem do SLAM acima.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': nav2_params,
                'use_sim_time': 'false',
                'autostart': 'true'
            }.items()
        )
    ])