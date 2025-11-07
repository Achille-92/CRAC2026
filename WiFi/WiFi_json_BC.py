import socket
import json
import time
import threading

# Configuration pour l'envoi
HOST_PC = "192.168.0.99"  # IP de l'ordinateur
PORT_ENVOI = 5000

# Configuration pour la réception
HOST_RPI = '0.0.0.0'  # Écoute sur toutes les interfaces
PORT_RECEPTION = 5001

# Données à envoyer
Liste_actions = [["Consigne", 1500, 1000], ["Rotation", 90]]
Liste_trajectoire = [[500, 400], [2800, 1000]]

donnees_pour_robot = {
    "Liste_actions": Liste_actions,
    "Liste_trajectoire": Liste_trajectoire
}

message = json.dumps(donnees_pour_robot)

# Fonction pour envoyer des données
def envoyer_donnees():
    print("[Émetteur] Démarrage de l'envoi en boucle...")
    try:
        while True:
            try:
                client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client_socket.connect((HOST_PC, PORT_ENVOI))
                client_socket.sendall(message.encode())
                client_socket.close()
                
                print("[Émetteur] Données JSON envoyées avec succès.")
                time.sleep(1)
                
            except ConnectionRefusedError:
                print("[Émetteur] Connexion refusée. Nouvelle tentative dans 2s...")
                time.sleep(2)
            except Exception as e:
                print(f"[Émetteur] Erreur : {e}. Nouvelle tentative dans 2s...")
                time.sleep(2)
                
    except KeyboardInterrupt:
        print("[Émetteur] Arrêt.")

# Fonction pour recevoir des données
def recevoir_donnees():
    print(f"[Récepteur] Serveur en attente sur le port {PORT_RECEPTION}...")
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST_RPI, PORT_RECEPTION))
    server_socket.listen(1)
    
    try:
        while True:
            conn, addr = server_socket.accept()
            print(f"\n[Récepteur] --- Connexion depuis {addr} ---")
            
            # Réception des données
            data = b""
            while True:
                packet = conn.recv(1024)
                if not packet:
                    break
                data += packet
            
            conn.close()
            
            # Décodage et affichage
            try:
                donnees_recues = json.loads(data.decode())
                
                print("[Récepteur] Données reçues depuis l'ordinateur :")
                print(json.dumps(donnees_recues, indent=4))
                
                # Traitement des données reçues
                if "commande" in donnees_recues:
                    print(f"[Récepteur] Commande reçue : {donnees_recues['commande']}")
                
            except json.JSONDecodeError:
                print("[Récepteur] Erreur : données JSON invalides")
            
    except KeyboardInterrupt:
        print("[Récepteur] Arrêt.")
    finally:
        server_socket.close()

# Lancement des deux threads
if __name__ == "__main__":
    print("=== RPI4 - Communication bidirectionnelle ===")
    print("Appuyez sur Ctrl+C pour arrêter\n")
    
    # Créer les threads
    thread_envoi = threading.Thread(target=envoyer_donnees, daemon=True)
    thread_reception = threading.Thread(target=recevoir_donnees, daemon=True)
    
    # Démarrer les threads
    thread_envoi.start()
    thread_reception.start()
    
    # Garder le programme actif
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n\n=== Arrêt du programme RPI4 ===")
