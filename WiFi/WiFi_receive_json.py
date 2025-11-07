import socket

HOST = ''       # écoute sur toutes les interfaces
PORT = 5000     # port d'écoute

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)

print(f"Serveur en attente sur le port {PORT}...")

conn, addr = server_socket.accept()
print(f"Connexion depuis {addr}")

with open("donnees_recue.json", "wb") as f:
    while True:
        data = conn.recv(1024)
        if not data:
            break
        f.write(data)

conn.close()
server_socket.close()
print("Fichier donnees_recue.json sauvegardé avec succès.")
