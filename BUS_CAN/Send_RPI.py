import os
import can
import time

# --- Configuration de l'interface CAN ---
os.system("sudo ip link set can0 type can bitrate 500000")  # même débit que l’ESP32
os.system("sudo ifconfig can0 up")

# --- Ouverture du bus ---
bus = can.interface.Bus(channel='can0', bustype='socketcan_ctypes')

# --- Envoi de trames toutes les secondes ---
try:
    print("Démarrage de l'envoi CAN...")
    while True:
        # Exemple de trame standard (11 bits)
        msg = can.Message(arbitration_id=0x12, data=[ord(c) for c in "HELLO"], is_extended_id=False)
        bus.send(msg)
        print(f"Trame envoyée : {msg}")

        time.sleep(1)

        # Exemple de trame étendue (29 bits)
        msg_ext = can.Message(arbitration_id=0xABCDEF, data=[ord(c) for c in "WORLD"], is_extended_id=True)
        bus.send(msg_ext)
        print(f"Trame étendue envoyée : {msg_ext}")

        time.sleep(1)

except KeyboardInterrupt:
    print("Arrêt demandé par l'utilisateur.")

finally:
    os.system("sudo ifconfig can0 down")
    print("Interface CAN désactivée.")
