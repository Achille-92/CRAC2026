import socket
import json

HOST = '0.0.0.0'
PORT = 5000

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"Serveur en attente sur le port {PORT}...")
print("Appuyez sur Ctrl+C pour arrêter")

try:
    while True:
        conn, addr = server_socket.accept()
        print(f"\n--- Connexion depuis {addr} ---")
        
        # Réception des données (chaîne JSON)
        data = b""
        while True:
            packet = conn.recv(1024)
            if not packet:
                break
            data += packet
        
        conn.close()
        
        # Décodage et affichage
        try:
            donnees_pour_robot = json.loads(data.decode())
            
            print("Données reçues :")
            print(json.dumps(donnees_pour_robot, indent=4))
            
            Liste_actions = donnees_pour_robot["Liste_actions"]
            Liste_trajectoire = donnees_pour_robot["Liste_trajectoire"]
            
            print("Liste_actions :", Liste_actions)
            print("Liste_trajectoire :", Liste_trajectoire)
        except json.JSONDecodeError:
            print("Erreur : données JSON invalides")
        
except KeyboardInterrupt:
    print("\nArrêt du serveur.")
finally:
    server_socket.close()
