import socket

# Adresse IP de la Raspberry Pi 5
HOST = "192.168.1.25"  # <-- remplace par l'IP réelle de la RPi5
PORT = 5000

# Lecture du fichier JSON
with open("donnees_envoi.json", "rb") as f:
    data = f.read()

# Envoi via socket TCP
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))
client_socket.sendall(data)
client_socket.close()

print("Fichier donnees_envoi.json envoyé avec succès.")
