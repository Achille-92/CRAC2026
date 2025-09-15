import can
import time

def main():
    # Ouvre l’interface CAN (assume que can0 est déjà configurée)
    bus = can.interface.Bus(channel='can0', bustype='socketcan')

    message = can.Message(arbitration_id=0x123,
                          data=[0x48, 0x65, 0x6C, 0x6C, 0x6F],  # "Hello" en ASCII
                          is_extended_id=False)

    try:
        while True:
            bus.send(message)
            print("Message envoyé sur le bus CAN:", message)
            time.sleep(1)

    except KeyboardInterrupt:
        print("Arrêt du programme.")

if __name__ == "__main__":
    main()
