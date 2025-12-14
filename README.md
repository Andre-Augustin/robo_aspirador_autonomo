=> Driver de Motores e Odometria (ROS 2)

Este pacote controla os motores DC do robô, lê os encoders (com filtro anti-ruído) e publica a posição do robô (/odom) para o ROS.


=> Estrutura de Arquivos (para os integrantes da minha equipa se acharem, ou para qualquer um que irá o projeto e queira saber onde codar de fato)

    - my_bot_controller/motor_driver.py 👈 Esse é o arquivo principal

        É aqui que a mágica acontece. Contém a classe EncoderForte (filtro de ruído, nossa placa tem muitos fios... infelizmente, ai tivemos que aplicar um filtro grosseiro no código), 
        o MotorDriver (nó ROS), o cálculo da Odometria e o controle PWM.

        Se precisar mudar a lógica, a pinagem ou as medidas do robô, é só nesse arquivo.

    setup.py (Configuração)

        Arquivo padrão do Python/ROS. Só mexemos aqui para definir o nome do executável (entry_points). Não precisa alterar.

    package.xml (Configuração)

        Lista as dependências (como rclpy e geometry_msgs). Não precisa alterar.


=> Pingagem GPIO

    Encoder Esquerda: GPIO 26

    Encoder Direita: GPIO 6

    Ponte H (PWM/Direção): Pinos 13, 12 (PWM: ENA, ENB) e 17, 27, 22, 23 (Direção: IN1, IN2, IN3, IN4).

    O que está faltando: definir pinos para o relé e para os módulos infra-vermelhos.
    
    ⚠️ IMPORTANTE: quando for configurar os sensores infra-vermelhos, conecte a alimentação no 3.3 v do rasp, não use o de 5V, risco de queimar, já que os GPIOS operam em 3.3 V
    Os infra-vermelhos serão os únicos módulos alimentados pelo rasp.

    ⚠️ IMPORTANTE: muitas vezes estamos utilizando fonte externa para ligar o Raspberry. Quando for essa situação, não esqueçam de ligar um jumper para conectar os GNDs das baterias e do Rasp.
