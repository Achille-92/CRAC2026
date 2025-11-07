import socket
import json

# Adresse IP de la Raspberry Pi 5
HOST = "192.168.0.99"  # <-- remplace par l'IP réelle de la RPi5
PORT = 5000

# Exemple de listes
Liste_actions = [["Consigne",1500,1000],["Rotation",90]]
Liste_trajectoire = [[500,400],[2800,1000]]

# Rassembler dans un dictionnaire (clé-valeur)
donnees_pour_robot = {
    "Liste_actions": Liste_actions,
    "Liste_trajectoire": Liste_trajectoire
}

# Écriture dans un fichier JSON
with open("donnees_pour_robot.json", "w") as f:
    json.dump(donnees_pour_robot, f, indent=4)

# Lecture du fichier JSON
with open("donnees_pour_robot.json", "rb") as f:
    data = f.read()

# Envoi via socket TCP
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))
client_socket.sendall(data)
client_socket.close()

print("Fichier donnees_pour_robot.json envoyé avec succès.")
