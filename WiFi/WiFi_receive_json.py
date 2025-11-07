import socket
import json
import time

HOST = "192.168.0.99"  # IP de ton PC
PORT = 5000

# Exemple de données
Liste_actions = [["Consigne", 1500, 1000], ["Rotation", 90]]
Liste_trajectoire = [[500, 400], [2800, 1000]]

donnees_pour_robot = {
    "Liste_actions": Liste_actions,
    "Liste_trajectoire": Liste_trajectoire
}

# Convertir en chaîne JSON
message = json.dumps(donnees_pour_robot)

print("Démarrage de l'envoi en boucle...")
print("Appuyez sur Ctrl+C pour arrêter")

try:
    while True:
        try:
            # Création et envoi via socket TCP
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((HOST, PORT))
            client_socket.sendall(message.encode())
            client_socket.close()
            
            print("Données JSON envoyées avec succès.")
            
            # Attendre un peu avant le prochain envoi (optionnel)
            time.sleep(1)  # Pause de 1 seconde entre chaque envoi
            
        except ConnectionRefusedError:
            print("Connexion refusée. Le serveur n'est pas disponible. Nouvelle tentative dans 2s...")
            time.sleep(2)
        except Exception as e:
            print(f"Erreur lors de l'envoi : {e}. Nouvelle tentative dans 2s...")
            time.sleep(2)
            
except KeyboardInterrupt:
    print("\nArrêt de l'émetteur.")
