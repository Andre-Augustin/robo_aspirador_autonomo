import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Localiza o diretório do pacote instalado
    share_dir = get_package_share_directory('ydlidar_ros2_driver')

    # Caminho para o arquivo de parâmetros (ydlidar.yaml)
    # Se você precisar alterar baudrate ou modelo, é neste arquivo que deve mexer depois
    parameter_file = os.path.join(share_dir, 'params', 'ydlidar.yaml')

    # Define o nó do driver
    driver_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        # Se quiser forçar parâmetros aqui direto, descomente abaixo:
        # parameters=[parameter_file, {'port': '/dev/ydlidar', 'baudrate': 115200}],
    )

    # Define o nó que publica a transformação estática (TF) do robô para o laser
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame'],
    )

    return LaunchDescription([
        driver_node,
        tf_node,
    ])

