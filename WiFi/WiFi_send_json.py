import socket
import json

HOST = "192.168.0.99"  # IP de ton PC (ou de la RPi5)
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

# Création et envoi via socket TCP
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))
client_socket.sendall(message.encode())  # on envoie la chaîne encodée en UTF-8
client_socket.close()

print("Données JSON envoyées avec succès.")
