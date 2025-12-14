import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import RPi.GPIO as GPIO
import time
import math
import threading

# --- MEDIDAS Do ROBÔ  ---

DIAMETRO_RODA = 0.065   
DISTANCIA_RODAS = 0.10  # 10cm entre as rodas
TICKS_POR_VOLTA = 20    # Aquele disco cheio de furinhos tem 20 buracos

# --- PINAGEM E AJUSTES DE SINAL ---
PINO_ENCODER_ESQ = 26
PINO_ENCODER_DIR = 6
FILTRO_FORCA = 35  # O nosso "Segura a Onda" pra matar o ruído

# Pinos que ligam na Ponte H (Pra fazer a roda girar)
IN1 = 27
IN2 = 17
IN3 = 23
IN4 = 22
ENA = 13
ENB = 12

class EncoderForte:
    """ 
    Essa classe é o vigia do encoder. Ela fica olhando o pino sem parar
    pra garantir que o sinal é verdadeiro e não loucura do motor.
    """
    def __init__(self, pino):
        self.pino = pino
        self.contador = 0
        self.direcao = 1  # 1 = Indo pra frente, -1 = Dando ré (Quem manda é o motor)
        
        # Configura o pino pra leitura
        GPIO.setup(pino, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Tira uma foto inicial do estado do pino
        self.estado_validado = GPIO.input(pino)
        self.estabilidade = 0 # Nosso "balde de confiança"

    def atualizar(self):
        # Dá uma olhada no pino agora
        leitura = GPIO.input(self.pino)
        
        # Opa, mudou alguma coisa?
        if leitura != self.estado_validado:
            # Hum... será que mudou mesmo ou é ruído? Vamos encher o balde.
            self.estabilidade += 1
            
            # Se encheu o balde até a boca (35 confirmações), então é verdade!
            if self.estabilidade >= FILTRO_FORCA:
                self.estado_validado = leitura # Aceitamos a mudança
                self.estabilidade = 0 # Esvazia o balde pra próxima
                
                # Se o sinal subiu (foi pra 1), conta um pulso!
                if self.estado_validado == 1:
                    # Aqui a mágica: Soma ou Subtrai dependendo da marcha
                    self.contador += self.direcao 
        else:
            # Se a leitura tá igual ao que a gente já sabia, zera o balde.
            # Qualquer tremidinha (ruído) morre aqui.
            self.estabilidade = 0

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        
        # Prepara os pinos do Raspberry (Modo BCM é vida)
        GPIO.setmode(GPIO.BCM)
        self.setup_motores()
        
        # Cria os nossos vigias (Encoders)
        self.enc_esq = EncoderForte(PINO_ENCODER_ESQ)
        self.enc_dir = EncoderForte(PINO_ENCODER_DIR)
        
        # Aqui é onde o robô acha que está no mundo (X, Y e Rotação)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0 
        
        # Guarda o tempo pra calcular velocidade depois
        self.last_time = self.get_clock().now()
        self.prev_ticks_esq = 0
        self.prev_ticks_dir = 0
        
        # O ROS precisa ouvir comandos e falar onde estamos
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Cria um timer que roda 10 vezes por segundo pra fazer a matemática
        self.timer = self.create_timer(0.1, self.update_odometry)
        
        # THREAD PARALELA: O Segredo do Sucesso!
        # Isso aqui roda numa "pista expressa" separada só pra ler encoder voando
        self.running = True
        self.encoder_thread = threading.Thread(target=self.loop_leitura_encoders)
        self.encoder_thread.start()

        self.get_logger().info("Motor Driver Ligado! Agora com Odometria calibrada.")

    def setup_motores(self):
        # Só avisando pro Rasp que esses pinos são de saída (pra mandar energia)
        pines = [IN1, IN2, IN3, IN4, ENA, ENB]
        for p in pines:
            GPIO.setup(p, GPIO.OUT)
        
        # Configura o PWM (o acelerador do carro)
        self.pwm_a = GPIO.PWM(ENA, 1000)
        self.pwm_b = GPIO.PWM(ENB, 1000)
        self.pwm_a.start(0)
        self.pwm_b.start(0)

    def loop_leitura_encoders(self):
        """ 
        Essa função roda numa realidade paralela (Thread).
        Ela não faz nada além de perguntar pros encoders: "Mudou? Mudou? E agora?"
        """
        while self.running:
            self.enc_esq.atualizar()
            self.enc_dir.atualizar()
            # Um cochilo de milionésimos de segundo pra CPU não pegar fogo
            time.sleep(0.0001) 

    def update_odometry(self):
        # 1. Quanto tempo passou desde a última vez?
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # 2. Pergunta pros vigias: "Quantos pulsos temos agora?"
        curr_ticks_esq = self.enc_esq.contador
        curr_ticks_dir = self.enc_dir.contador
        
        # 3. Quantos pulsos novos aconteceram nesse tempinho?
        d_ticks_esq = curr_ticks_esq - self.prev_ticks_esq
        d_ticks_dir = curr_ticks_dir - self.prev_ticks_dir
        
        # Atualiza o histórico
        self.prev_ticks_esq = curr_ticks_esq
        self.prev_ticks_dir = curr_ticks_dir
        
        # 4. Converte "Pulsos" para "Metros" (Matemática pura)
        # Fórmula: (Pi * Diâmetro) / Total de furos
        metros_por_tick = (math.pi * DIAMETRO_RODA) / TICKS_POR_VOLTA
        
        d_esq = d_ticks_esq * metros_por_tick
        d_dir = d_ticks_dir * metros_por_tick
        
        # 5. Calcula onde o robô foi parar
        # Distância média (quanto o centro do robô andou)
        d_center = (d_esq + d_dir) / 2.0
        # O quanto ele girou (diferença entre as rodas dividida pela largura)
        d_theta = (d_dir - d_esq) / DISTANCIA_RODAS
        
        # Soma tudo na posição global (Navegação estimada)
        self.x += d_center * math.cos(self.th)
        self.y += d_center * math.sin(self.th)
        self.th += d_theta
        
        # 6. Avisa o Rviz que o robô se mexeu (TF)
        # Transformando Euler (ângulos normais) pra Quaternion (matemática alienígena do ROS)
        q = self.euler_to_quaternion(0, 0, self.th)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)
        
        # 7. Publica no tópico /odom pra quem quiser ouvir
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        
        # Calcula a velocidade instantânea tbm
        if dt > 0:
            odom.twist.twist.linear.x = d_center / dt
            odom.twist.twist.angular.z = d_theta / dt
        
        self.odom_pub.publish(odom)

    def cmd_vel_callback(self, msg):
        # Recebeu ordem do teclado ou do cérebro autônomo!
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Mistura pra saber o quanto cada roda tem que girar (Cinemática Diferencial)
        vel_esq = linear - angular
        vel_dir = linear + angular
        
        # O PULO DO GATO: Avisa o encoder se estamos indo pra frente ou pra trás
        # Pq o sensor sozinho não sabe, ele é meio burrinho pra direção.
        if vel_esq >= 0: self.enc_esq.direcao = 1
        else: self.enc_esq.direcao = -1
            
        if vel_dir >= 0: self.enc_dir.direcao = 1
        else: self.enc_dir.direcao = -1

        # Manda ver nos motores!
        self.set_motor(self.pwm_a, IN1, IN2, vel_esq)
        self.set_motor(self.pwm_b, IN3, IN4, vel_dir)

    def set_motor(self, pwm, in_a, in_b, vel):
        # Transforma a velocidade (-1 a 1) em força do motor (0 a 100)
        duty = min(abs(vel * 100), 100)
        
        if vel > 0:
            GPIO.output(in_a, GPIO.HIGH)
            GPIO.output(in_b, GPIO.LOW)
        elif vel < 0:
            GPIO.output(in_a, GPIO.LOW)
            GPIO.output(in_b, GPIO.HIGH)
        else:
            # Ponto morto
            GPIO.output(in_a, GPIO.LOW)
            GPIO.output(in_b, GPIO.LOW)
            
        pwm.ChangeDutyCycle(duty)

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Função matemática chata pra converter ângulos. Copia e cola que é sucesso.
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def __del__(self):
        # Se desligar o nó, para a thread e limpa a bagunça nos pinos
        self.running = False
        if self.encoder_thread.is_alive():
            self.encoder_thread.join()
        GPIO.cleanup()

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
