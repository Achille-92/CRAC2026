import can
import time

def main():
    bus = can.interface.Bus(channel='can0', bustype='socketcan')
    try:
        # ton code d'envoi ici
        message = can.Message(arbitration_id=0x123, data=[0x11, 0x22, 0x33, 0x44], is_extended_id=False)
        bus.send(message)
        time.sleep(1)
    finally:
        # Cette ligne est cruciale pour libérer le socket
        bus.shutdown()

if __name__ == "__main__":
    main()
