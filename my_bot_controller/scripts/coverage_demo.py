#! /usr/bin/env python3
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Espera o Nav2 estar 100% ativo
    navigator.waitUntilNav2Active()

    # --- DEFINIR A ÁREA DE COBERTURA (Manual por enquanto) ---
    # Imagine que você quer que o robô vá em 4 pontos formando um quadrado/zigue-zague
    # Você vai ajustar esses números (x, y) olhando no RViz depois.
    pontos_de_visita = [
        [1.0, 0.0],  # Ponto 1: 1 metro pra frente
        [1.0, 0.5],  # Ponto 2: 1m frente, 0.5m esquerda
        [0.0, 0.5],  # Ponto 3: Volta pro inicio (mas deslocado)
        [0.0, 0.0]   # Ponto 4: Base
    ]

    goal_poses = []
    
    for pt in pontos_de_visita:
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pt[0]
        goal_pose.pose.position.y = pt[1]
        goal_pose.pose.orientation.w = 1.0 # Sem rotação específica
        goal_poses.append(goal_pose)

    # MANDAR O ROBÔ SEGUIR A LISTA (WAYPOINTS)
    print("Iniciando varredura...")
    navigator.followWaypoints(goal_poses)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        # Aqui você poderia imprimir o progresso
        
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Varredura Completa!')
    else:
        print('Falha na varredura!')

    exit(0)

if __name__ == '__main__':
    main()