# --- CLASSE PARA O SENSOR INFRAVERMELHO ---

class SensorIR:
    """
    Classe para encapsular a leitura de um sensor infravermelho digital (Obstáculo).
    Assume que o sensor retorna LOW (0) quando um objeto é detectado.
    """
    def __init__(self, pino_gpio: int):
        """
        Inicializa o sensor.
        :param pino_gpio: O número do pino GPIO (BCM) conectado à saída OUT do sensor.
        """
        self._pino = pino_gpio
        
        # Configura o pino como ENTRADA.
        GPIO.setup(self._pino, GPIO.IN) 

    def detectar_objeto(self) -> bool:
        """
        Lê o estado do sensor.
        :return: True se um obstáculo for detectado (leitura LOW), False caso contrário.
        """
        # Se seu sensor for "active-HIGH" (HIGH = Objeto), mude para:
        # return GPIO.input(self._pino) == GPIO.HIGH
        
        return GPIO.input(self._pino) == GPIO.LOW
