#!/usr/bin/env python3
import time
import vida_bateria

def main():
    print("Sistema principal iniciado")

    try:
        while True:
            # Lê o nível da bateria (função do módulo vida_bateria)
            nivel = vida_bateria.get_battery_level()
            dados = vida_bateria.get_battery_data()

            print(
                f"[MAIN] Bateria: {nivel:.1f}% | "
                f"Tensão ADC: {dados['tensao_adc']:.3f} V | "
                f"Tensão real: {dados['tensao_bateria']:.2f} V"
            )

            if nivel < 20:
                print("[MAIN] ⚠️  Bateria baixa!")

            time.sleep(2)

    except KeyboardInterrupt:
        print("\nSistema encerrado.")

if __name__ == "__main__":
    main()