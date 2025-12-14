#!/usr/bin/env python3
import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# ========== CONFIGURAÇÃO GLOBAL ==========
R1 = 40000
R2 = 10000
FATOR_DIVISOR = (R1 + R2) / R2

V_BAT_CHEIA = 16.8
V_BAT_VAZIA = 8.0

# Inicializa o hardware (executado uma vez ao importar)
_i2c = busio.I2C(board.SCL, board.SDA)
_ads = ADS.ADS1115(_i2c)
_ads.gain = 1
_canal_bateria = AnalogIn(_ads, ADS.P0)

print(f"[BatteryMonitor] Módulo carregado - R1={R1/1000}kΩ, R2={R2/1000}kΩ")


# ========== FUNÇÕES PÚBLICAS ==========

def get_battery_level():
    
    """Retorna o nível de carga da bateria em porcentagem."""
    tensao_bat = get_battery_voltage()
    return _calcula_percentual(tensao_bat)


def get_battery_voltage():
    
    """Retorna a tensão real da bateria em volts."""
    tensao_adc = _canal_bateria.voltage
    return tensao_adc * FATOR_DIVISOR


def get_battery_data():
    
    """Retorna todos os dados da bateria."""
    tensao_adc = _canal_bateria.voltage
    tensao_bat = tensao_adc * FATOR_DIVISOR
    percentual = _calcula_percentual(tensao_bat)
    
    return {
        'tensao_adc': tensao_adc,
        'tensao_bateria': tensao_bat,
        'percentual': percentual
    }


# ========== FUNÇÕES INTERNAS ==========

def _calcula_percentual(v_bat):
    """Calcula o percentual de carga."""
    if v_bat >= V_BAT_CHEIA:
        return 100.0
    if v_bat <= V_BAT_VAZIA:
        return 0.0
    return (v_bat - V_BAT_VAZIA) * 100.0 / (V_BAT_CHEIA - V_BAT_VAZIA)


"""
O I²C vem desabilitado por padrão no Raspberry Pi. Você precisa habilitá-lo antes de usar.

RODAR TUDO NO TERMINAR DO RASPBERRY

Passo 1: Habilitar o I²C via raspi-config
sudo raspi-config
Navegue pelo menu:
1 - 3 Interface Options
2 - I5 I2C
3 - Selecione Yes para habilitar
4 - Finish e reinicie:
sudo reboot

Passo 2: Instalar ferramentas I²C
Após reiniciar, instale as ferramentas para testar o I²C:
sudo apt-get update
sudo apt-get install -y python3-pip i2c-tools

Passo 3: Verificar se o I²C está funcionando
Com o ADS1115 já conectado (VDD, GND, SDA, SCL), rode:
sudo i2cdetect -y 1

Passo 4: Instalar a biblioteca Python para o ADS1115
pip3 install adafruit-circuitpython-ads1x15


┌─────────────────┐
│    ADS1115      │
├─────────────────┤
│ VDD   ●         │  ← Alimentação 3.3V
│ GND   ●         │  ← GND (0V)
│ SCL   ●         │  ← Clock I²C (Ligado no Pin 5 do Rasp)
│ SDA   ●         │  ← Dados I²C (Ligado no Pin 3 do Rasp)
│ ADDR  ●         │  ← Endereço I²C (ligar ao GND)
│ ALRT  ●         │  
│ A0    ●         │  ← Entrada analógica 0 (vai receber o divisor)
│ A1    ●         │  
│ A2    ●         │  
│ A3    ●         │  
└─────────────────┘
"""