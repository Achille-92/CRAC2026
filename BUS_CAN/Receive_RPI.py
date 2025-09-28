import can
import time
import threading
import signal
import sys

# Flag pour arrêter proprement les threads
running = True

def signal_handler(sig, frame):
    global running
    print("\nArrêt demandé, fermeture du bus CAN...")
    running = False

signal.signal(signal.SIGINT, signal_handler)

def read_messages(bus):
    """Thread qui lit les messages entrants pour ne pas saturer le buffer"""
    while running:
        msg = bus.recv(timeout=1)
        if msg:
            print(f"Message reçu (ignoré): {msg}")

def main():
    global running

    try:
        # Initialisation du bus CAN
        bus = can.interface.Bus(channel='can0', interface='socketcan')
        print("Bus CAN ouvert avec succès")

        # Démarrage du thread de lecture
        reader_thread = threading.Thread(target=read_messages, args=(bus,), daemon=True)
        reader_thread.start()

        # Message à envoyer
        message = can.Message(arbitration_id=0x123,
                              data=[0x11, 0x22, 0x33],
                              is_extended_id=False)

        print("Début de l'envoi des messages toutes les 1 seconde...")
        while running:
            try:
                bus.send(message)
                print(f"Message envoyé : {message}")
            except can.CanError as e:
                print(f"Erreur lors de l'envoi : {e}")
            time.sleep(1)

    finally:
        # Fermeture propre du bus
        bus.shutdown()
        print("Bus CAN fermé correctement.")

if __name__ == "__main__":
    main()
