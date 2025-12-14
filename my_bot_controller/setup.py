import os
from glob import glob
from setuptools import setup

package_name = 'my_bot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- AQUI: Instalação dos arquivos de Launch ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # --- AQUI: Instalação dos arquivos de Config (Mapas e Params) ---
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seu_nome',
    maintainer_email='seu_email@email.com',
    description='Controlador do Robo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # O formato é: 'nome_do_comando = pasta.arquivo:funcao_main'
            
            # Seu driver de motores (Fase 1)
            'driver_motores = my_bot_controller.motor_driver:main',
            
            # Seu script de cobertura (Fase 5) - ADICIONE ESTA LINHA
            'coverage_demo = my_bot_controller.coverage_demo:main',
        ],
    },
)