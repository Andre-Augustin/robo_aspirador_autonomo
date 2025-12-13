import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

# --- CONFIGURAÇÃO DOS PINOS (BCM) ---
ENA = 13  # PWM Motor A (Esquerdo?)
ENB = 12  # PWM Motor B (Direito?)
IN1 = 16  # Motor A Direção
IN2 = 25  # Motor A Direção
IN3 = 24  # Motor B Direção
IN4 = 23  # Motor B Direção

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.get_logger().info('Iniciando Driver de Motor L298N...')

        # Configura o ROS para ouvir comandos de velocidade
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        # Configura GPIO do Raspberry
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        pinos = [ENA, ENB, IN1, IN2, IN3, IN4]
        GPIO.setup(pinos, GPIO.OUT)

        # Configura PWM (Frequência 1000Hz)
        self.pwm_a = GPIO.PWM(ENA, 1000)
        self.pwm_b = GPIO.PWM(ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def listener_callback(self, msg):
        # Pega a velocidade linear (frente/traz) e angular (girar)
        # linear.x: positivo = frente
        # angular.z: positivo = esquerda
        linear = msg.linear.x
        angular = msg.angular.z

        # --- CINEMÁTICA DIFERENCIAL SIMPLES ---
        # Mistura frente com giro para saber a força de cada roda
        # Ajuste o fator 1.0 se ele girar muito rápido
        velocidade_esquerda = linear - angular
        velocidade_direita = linear + angular

        # Envia para os motores
        self.controlar_motor(self.pwm_a, IN1, IN2, velocidade_esquerda)
        self.controlar_motor(self.pwm_b, IN3, IN4, velocidade_direita)

    def controlar_motor(self, pwm_obj, pino_frente, pino_tras, velocidade):
        # 1. Normalizar a velocidade (converter m/s para % de PWM)
        # Vamos assumir que 1.0 m/s (ou rad/s) = 100% de força
        # Limitamos entre -100 e 100
        duty_cycle = abs(velocidade) * 100
        if duty_cycle > 100: duty_cycle = 100
        
        # Zona morta: Se for muito fraco, desliga para não ficar zumbindo
        if duty_cycle < 10: duty_cycle = 0

        # 2. Controlar Direção
        if velocidade > 0:
            GPIO.output(pino_frente, GPIO.HIGH)
            GPIO.output(pino_tras, GPIO.LOW)
        elif velocidade < 0:
            GPIO.output(pino_frente, GPIO.LOW)
            GPIO.output(pino_tras, GPIO.HIGH)
        else:
            GPIO.output(pino_frente, GPIO.LOW)
            GPIO.output(pino_tras, GPIO.LOW)

        # 3. Aplicar Força (PWM)
        pwm_obj.ChangeDutyCycle(duty_cycle)

    def stop(self):
        self.pwm_a.stop()
        self.pwm_b.stop()
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    driver = MotorDriver()
    
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    finally:
        driver.stop()
        driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
