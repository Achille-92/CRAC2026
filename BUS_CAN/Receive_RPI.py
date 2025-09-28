import can
import time

def main():
    try:
        # Création du bus CAN
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
        print("Bus CAN ouvert avec succès")

        # Exemple de message à envoyer
        message = can.Message(arbitration_id=0x123,
                              data=[0x11, 0x22, 0x33],
                              is_extended_id=False)

        print("Début de l'envoi des messages toutes les 1 seconde...")
        while True:
            try:
                bus.send(message)
                print(f"Message envoyé : {message}")
            except can.CanError as e:
                print(f"Erreur lors de l'envoi : {e}")
            time.sleep(1)  # délai 1 seconde entre chaque envoi

    except Exception as e:
        print(f"Erreur lors de l'ouverture du bus : {e}")
    
    finally:
        # Toujours fermer le bus pour libérer le buffer
        try:
            bus.shutdown()
            print("Bus CAN fermé proprement")
        except:
            pass

if __name__ == "__main__":
    main()
