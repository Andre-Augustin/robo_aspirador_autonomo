import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import RPi.GPIO as GPIO
import time
import math
import threading

# =========================
#   PARÂMETROS DO ROBÔ
# =========================

DIAMETRO_RODA = 0.065        # [m]  6,5 cm
DISTANCIA_RODAS = 0.10       # [m]  10 cm entre centros das rodas
TICKS_POR_VOLTA = 20         # Nº de furos do disco do encoder

# Velocidade máxima realista da roda (ajustar na calibração)
VEL_MAX_METROS = 0.4         # [m/s] chute inicial, calibra depois

# =========================
#   PINAGEM E AJUSTES
# =========================

PINO_ENCODER_ESQ = 26
PINO_ENCODER_DIR = 6
FILTRO_FORCA = 35            # “debounce” por software

# Ponte H
IN1 = 17
IN2 = 27
IN3 = 22
IN4 = 23
ENA = 13
ENB = 12


class EncoderForte:
    """
    Faz leitura estável do encoder via polling rápido + filtro.
    Conta apenas borda de subida (0 -> 1).
    """
    def __init__(self, pino):
        self.pino = pino
        self.contador = 0
        self.direcao = 1  # +1 frente, -1 ré

        GPIO.setup(pino, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.estado_validado = GPIO.input(pino)
        self.estabilidade = 0

    def atualizar(self):
        leitura = GPIO.input(self.pino)

        if leitura != self.estado_validado:
            self.estabilidade += 1

            if self.estabilidade >= FILTRO_FORCA:   
                self.estado_validado = leitura
                self.estabilidade = 0

                if self.estado_validado == 1:
                    self.contador += self.direcao
        else:
            self.estabilidade = 0


class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        self.setup_motores()

        # Encoders
        self.enc_esq = EncoderForte(PINO_ENCODER_ESQ)
        self.enc_dir = EncoderForte(PINO_ENCODER_DIR)

        # Pose estimada
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Histórico para odometria
        self.last_time = self.get_clock().now()
        self.prev_ticks_esq = 0
        self.prev_ticks_dir = 0

        # ROS2: cmd_vel, odom e TF
        self.sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer de odometria (10 Hz)
        self.timer = self.create_timer(0.1, self.update_odometry)

        # Thread de leitura dos encoders
        self.running = True
        self.encoder_thread = threading.Thread(target=self.loop_leitura_encoders)
        self.encoder_thread.daemon = True
        self.encoder_thread.start()

        self.get_logger().info("Motor Driver iniciado com odometria.")

    # --------------------------
    #   Setup de motores
    # --------------------------
    def setup_motores(self):
        pinos = [IN1, IN2, IN3, IN4, ENA, ENB]
        for p in pinos:
            GPIO.setup(p, GPIO.OUT)

        # PWM nos enables
        self.pwm_a = GPIO.PWM(ENA, 1000)
        self.pwm_b = GPIO.PWM(ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    # --------------------------
    #   Loop de encoders (thread)
    # --------------------------
    def loop_leitura_encoders(self):
        while self.running:
            self.enc_esq.atualizar()
            self.enc_dir.atualizar()
            time.sleep(0.0001)  # 100 µs

    # --------------------------
    #   Odometria (timer)
    # --------------------------
    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.last_time = current_time

        # Leitura atual dos ticks
        curr_ticks_esq = self.enc_esq.contador
        curr_ticks_dir = self.enc_dir.contador

        # Diferença desde a última leitura
        d_ticks_esq = curr_ticks_esq - self.prev_ticks_esq
        d_ticks_dir = curr_ticks_dir - self.prev_ticks_dir

        self.prev_ticks_esq = curr_ticks_esq
        self.prev_ticks_dir = curr_ticks_dir

        # Converte ticks -> metros na roda
        metros_por_tick = (math.pi * DIAMETRO_RODA) / TICKS_POR_VOLTA
        d_esq = d_ticks_esq * metros_por_tick
        d_dir = d_ticks_dir * metros_por_tick

        # Cinemática diferencial
        d_center = (d_esq + d_dir) / 2.0
        d_theta = (d_dir - d_esq) / DISTANCIA_RODAS

        # Integração pelo método do "meio passo" (melhor pra curvas)
        if abs(d_theta) < 1e-6:
            # Movimento praticamente retilíneo
            dx = d_center * math.cos(self.th)
            dy = d_center * math.sin(self.th)
        else:
            # Movimento em arco
            th_meio = self.th + d_theta / 2.0
            dx = d_center * math.cos(th_meio)
            dy = d_center * math.sin(th_meio)

        self.x += dx
        self.y += dy
        self.th += d_theta

        # Normaliza o ângulo entre -pi e pi
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        # Quaternion
        q = self.euler_to_quaternion(0.0, 0.0, self.th)

        # ---------------- TF odom -> base_link ----------------
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # ---------------- Mensagem /odom ----------------
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        # Velocidades instantâneas
        odom.twist.twist.linear.x = d_center / dt
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = d_theta / dt

        self.odom_pub.publish(odom)

    # --------------------------
    #   Callback do /cmd_vel
    # --------------------------
    def cmd_vel_callback(self, msg: Twist):
        """
        Converte /cmd_vel (m/s, rad/s) -> velocidade das rodas -> PWM.
        """
        v = msg.linear.x     # [m/s]
        w = msg.angular.z    # [rad/s]

        # Cinemática diferencial inversa
        v_r = v + (w * DISTANCIA_RODAS / 2.0)  # [m/s]
        v_l = v - (w * DISTANCIA_RODAS / 2.0)  # [m/s]

        # Sinal de direção para os encoders
        self.enc_esq.direcao = 1 if v_l >= 0.0 else -1
        self.enc_dir.direcao = 1 if v_r >= 0.0 else -1

        # Normaliza para faixa -1..1 para os motores
        v_l_norm = max(min(v_l / VEL_MAX_METROS, 1.0), -1.0)
        v_r_norm = max(min(v_r / VEL_MAX_METROS, 1.0), -1.0)

        self.set_motor(self.pwm_a, IN1, IN2, v_l_norm)
        self.set_motor(self.pwm_b, IN3, IN4, v_r_norm)

    # --------------------------
    #   Acionamento dos motores
    # --------------------------
    def set_motor(self, pwm, in_a, in_b, vel_norm):
        """
        vel_norm em [-1, 1]
        """
        duty = min(abs(vel_norm) * 100.0, 100.0)

        if vel_norm > 0:
            GPIO.output(in_a, GPIO.HIGH)
            GPIO.output(in_b, GPIO.LOW)
        elif vel_norm < 0:
            GPIO.output(in_a, GPIO.LOW)
            GPIO.output(in_b, GPIO.HIGH)
        else:
            GPIO.output(in_a, GPIO.LOW)
            GPIO.output(in_b, GPIO.LOW)

        pwm.ChangeDutyCycle(duty)

    # --------------------------
    #   Euler -> Quaternion
    # --------------------------
    def euler_to_quaternion(self, roll, pitch, yaw):
        # Usando parênteses para evitar erros de indentação com barra invertida
        qx = (math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - 
              math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2))
        
        qy = (math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + 
              math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2))
        
        qz = (math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - 
              math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2))
        
        qw = (math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + 
              math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2))
        
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    # --------------------------
    #   Finalização
    # --------------------------
    def destroy_node(self):
        # Para a thread e limpa GPIO
        self.running = False
        try:
            if self.encoder_thread.is_alive():
                self.encoder_thread.join(timeout=0.5)
        except Exception:
            pass

        try:
            self.pwm_a.stop()
            self.pwm_b.stop()
        except Exception:
            pass

        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()