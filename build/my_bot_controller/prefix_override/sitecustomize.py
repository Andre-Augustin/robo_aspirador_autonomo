import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/samuel/eraseme/robo_aspirador_autonomo/install/my_bot_controller'
