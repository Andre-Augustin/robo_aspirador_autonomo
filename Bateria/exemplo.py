#!/usr/bin/env python3
import time
import vida_bateria

def main():
    print("Sistema principal iniciado")
    
    try:
        while True:
            # Lê o nível da bateria
            nivel = battery_monitor.get_battery_level()
            
            print(f"[MAIN] Bateria: {nivel:.1f}%")
            
            if nivel < 20:
                print("[MAIN] ⚠️  Bateria baixa!")
            
            time.sleep(2)
    
    except KeyboardInterrupt:
        print("\nSistema encerrado.")

if __name__ == "__main__":
    main()