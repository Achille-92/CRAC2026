import can
import time

def main():
    # Ouvre le bus CAN
    bus = can.interface.Bus(channel='can0', bustype='socketcan')

    message = can.Message(
        arbitration_id=0x123,   # Identifiant CAN
        data=[72, 101, 108, 108, 111],  # "Hello" en ASCII
        is_extended_id=False
    )

    try:
        while True:
            bus.send(message)
            print("Message envoyé :", message.data)
            time.sleep(1)
    except KeyboardInterrupt:
        print("Arrêt de l’émission")

if __name__ == "__main__":
    main()

