import socket
import json
import time
import threading

# Configuration pour la réception
HOST_PC = '0.0.0.0'
PORT_RECEPTION = 5000

# Configuration pour l'envoi vers la RPI
HOST_RPI = "192.168.0.100"  # IP de la RPI4
PORT_ENVOI = 5001

# Données à envoyer vers la RPI
x_robot_actuel = 500
y_robot_actuel = 400
angle_robot_actuel = 90
x_ennemi = 2500
y_ennemi = 1000
Batteries = [[12,14,14,100],[12,14,14,100],[12,14,14,100]]
donnees_vers_rpi = {
    "x_robot_actuel": x_robot_actuel,
    "y_robot_actuel": y_robot_actuel,
    "angle_robot_actuel": angle_robot_actuel,
    "x_ennemi": x_ennemi,
    "y_ennemi": y_ennemi,
    "Batteries": Batteries
}

message = json.dumps(donnees_vers_rpi)

# Fonction pour recevoir des données de la RPI
def recevoir_donnees():
    print(f"[Récepteur] Serveur en attente sur le port {PORT_RECEPTION}...")
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST_PC, PORT_RECEPTION))
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
                donnees_robot = json.loads(data.decode())
                
                print("[Récepteur] Données reçues depuis la RPI :")
                print(json.dumps(donnees_robot, indent=4))
                
                Liste_actions = donnees_robot["Liste_actions"]
                Liste_trajectoire = donnees_robot["Liste_trajectoire"]
                Ordre_receive = donnees_robot["Ordre_receive"]
                
                print(f"[Récepteur] Liste_actions : {Liste_actions}")
                print(f"[Récepteur] Liste_trajectoire : {Liste_trajectoire}")
                print(f"[Récepteur] Ordre_receive : {Ordre_receive}")
                
            except json.JSONDecodeError:
                print("[Récepteur] Erreur : données JSON invalides")
            
    except KeyboardInterrupt:
        print("[Récepteur] Arrêt.")
    finally:
        server_socket.close()


# Lancement des deux threads
if __name__ == "__main__":
    print("=== Ordinateur - Communication bidirectionnelle ===")
    print("Appuyez sur Ctrl+C pour arrêter\n")
    
    # Créer les threads
    thread_reception = threading.Thread(target=recevoir_donnees, daemon=True)
    
    # Démarrer les threads
    thread_reception.start()
    
    # Garder le programme actif
    try:
        while True:
            try:
                client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client_socket.connect((HOST_RPI, PORT_ENVOI))
                client_socket.sendall(message.encode())
                client_socket.close()
                
                print("[Émetteur] Données JSON envoyées vers la RPI avec succès.")
                time.sleep(1)
                
            except ConnectionRefusedError:
                print("[Émetteur] Connexion refusée. Nouvelle tentative dans 2s...")
                time.sleep(2)
            except Exception as e:
                print(f"[Émetteur] Erreur : {e}. Nouvelle tentative dans 2s...")
                time.sleep(2)
    except KeyboardInterrupt:
        print("\n\n=== Arrêt du programme Ordinateur ===")
