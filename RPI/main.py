################## Librairies ##########################################
from rplidar import RPLidar
import math
import numpy as np
import matplotlib.pyplot as plt
import threading, queue
from collections import deque
import time
import os
import can
import struct
import matplotlib.image as mpimg
import matplotlib.patches as patches
from affichage import init_affichage
from lidar import init_lidar
from lidar import calcul_points
########################################################################

# config CAN
"""os.system('sudo ip link set can0 type can bitrate 500000')
os.system('sudo ifconfig can0 up')
bus = can.interface.Bus(
    channel='can0',
    bustype='socketcan',
    bitrate=500000,
    can_filters=[{"can_id": 0x01, "can_mask": 0x7FF, "extended": False},
                 {"can_id": 0x10, "can_mask": 0x7FF, "extended": False},
                 {"can_id": 0x11, "can_mask": 0x7FF, "extended": False},
                 {"can_id": 0x12, "can_mask": 0x7FF, "extended": False},
                 {"can_id": 0x20, "can_mask": 0x7FF, "extended": False},
                 {"can_id": 0x21, "can_mask": 0x7FF, "extended": False},
                 {"can_id": 0x22, "can_mask": 0x7FF, "extended": False}]
)"""

Liste_ID = [0x01,0x10, 0x11, 0x12,0x20,0x21,0x22]

buffer_Lidar = 4096
buffer_Pile_calcul = 500
buffer_affichage = 50
# Port série et Baudrate du lidar
PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 256000
lidar = init_lidar(PORT_NAME, BAUDRATE)

# Création de l'objet Lidar, et de la Pile pile_points

pile_calcul = queue.Queue(maxsize=buffer_Pile_calcul)
marge_bordurepiste = 15 #mm

# Coordonnées et angle de notre robot
x_robot_depart = 120
y_robot_depart = 145
angle_robot_depart = 90

x_robot_actuel = 0
y_robot_actuel = 0
angle_robot_actuel = 0

x_robot_voulu = 0
y_robot_voulu = 0
angle_robot_voulu = 0

x_ennemi = 1500
y_ennemi = 1000

Batteries = [[12,14,14,100],[12,14,14,100],[12,14,14,100]]
largeur_rect = 50       # largeur en mm
hauteur_rect = 150      # hauteur en mm
espacement = 0         # espace entre rectangles
espacement_salves = 200 # espace entre chaque salve
y_base = 2050           # position verticale (en haut de la piste)
marge_texte = 20  # espace horizontal entre texte et rectangle
texte_offset_y = 80  # décalage vertical du texte par rapport aux rectangles

# Objets et variables pour la fenêtre graphique
fig = None 
ax = None
robot_plot = None
scat = None

################## Fonction  ###########################################

def CAN_Odometrie(stop_event):
    
    #Argument : flag "stop_event"
    #Modification : variables globales x_robot, y_robot, angle_robot

    #Utilisation :
    #Lis le Bus CAN
    #Si l'ID du message n'est pas dans la Liste_ID, saute
    #Sinon, met à jour les coordonées et angle du robot
     
    """global x_robot_actuel, y_robot_actuel, angle_robot_actuel, Liste_ID

    while not stop_event.is_set():
        msg = bus.recv(0.01)  # attend 10 ms max
        if msg is None:
            continue  # pas de message, on repart

        # Vérifie qu'on a bien reçu 4 octets avant de décoder
        if msg.arbitration_id not in Liste_ID:
            continue  # on saute les autres trames

        if msg.arbitration_id == 0x10:
            x_robot_actuel = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x11:
            y_robot_actuel = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x12:
            angle_robot_actuel = struct.unpack('f', bytes(msg.data))[0]
"""
def calcul_ennemi(stop_event):
    """
    Arguments : flag stop_event
    Modification : buffer_points, x_ennemi, y_ennemi

    Utilisation :
    Récupére le haut de la pile_calcul, puis le remet dans buffer_points
    Moyenne les coordonées des points de la pile, donne x_ennemi et y_ennemi
    """
    global x_ennemi, y_ennemi,buffer_affichage

    buffer_points = deque(maxlen=buffer_affichage)

    while not stop_event.is_set():
        try:
            # Récupère un point du Lidar
            p = pile_calcul.get(timeout=0.1)
            buffer_points.append(p)
        except queue.Empty:
            pass

        # Calcul barycentre, angle et vitesse
        if buffer_points:
            xs, ys = zip(*buffer_points)
            x_ennemi = np.mean(xs)
            y_ennemi = np.mean(ys)

########################################################################

################## Lancement du programme principal ####################

if __name__ == '__main__':

    stop_event = threading.Event()

    fig, ax, robot_plot, ennemi_plot, scat = init_affichage()
    
    tache_lidar = threading.Thread(target=calcul_points, args=(lidar, pile_calcul, stop_event, x_robot_actuel, y_robot_actuel, angle_robot_actuel,buffer_Lidar, marge_bordurepiste), daemon=False)
    """tache_calcul = threading.Thread(target=calcul_ennemi, args=(stop_event,), daemon=False)
    tache_odometrie = threading.Thread(target=CAN_Odometrie, args=(stop_event,), daemon=True)

    tache_odometrie.start()
    tache_calcul.start()"""
    
    tache_lidar.start()
    buffer_points = deque(maxlen=buffer_affichage)

    print("INFO:", lidar.get_info())                                    # Affichage d'informations propres au Lidar
    print("HEALTH:", lidar.get_health())

    try:
       
        """data_x = struct.pack('<f',x_robot_depart)
        data_y = struct.pack('<f',y_robot_depart)
        data_angle = struct.pack('<f',angle_robot_depart)

        msg = can.Message(arbitration_id=0x20, data=data_x, is_extended_id=False)
        bus.send(msg)
        print(f"Trame envoyée : {msg}")

        msg = can.Message(arbitration_id=0x21, data=data_y, is_extended_id=False)
        bus.send(msg)
        print(f"Trame envoyée : {msg}")

        msg = can.Message(arbitration_id=0x22, data=data_angle, is_extended_id=False)
        bus.send(msg)
        print(f"Trame envoyée : {msg}")
        time.sleep(1)"""

        while True:
            etat = 1
            """data_etat = struct.pack('<I',etat)
            bus.send(can.Message(arbitration_id=0x01, data=data_etat, is_extended_id=False))"""
            # Mettre à jour robot et ennemi sur affichage
            robot_plot.set_offsets([[x_robot_actuel, y_robot_actuel]])
            ennemi_plot.set_offsets([[x_ennemi, y_ennemi]])

            # Gestion batteries :
            for i in range(len(Batteries)):
                Batteries[i][3]=100*(Batteries[i][2]-Batteries[i][0])/(Batteries[i][1]-Batteries[i][0])
                Batteries[i][3] = round(Batteries[i][3],2)
                
            
                # Définir couleurs dans l'ordre
                couleurs = ['red', 'orange', 'yellow', 'lime', 'green']
                seuils = [10, 25, 50, 75, 90]  # seuil minimal pour chaque rectangle

                # Calcul du point de départ de la salve
                x_depart = 200 + i * (5 * (largeur_rect + espacement) + espacement_salves)

                # Création des rectangles selon le pourcentage
                for j in range(5):
                    if Batteries[i][3] >= seuils[j]:
                        x = x_depart + j * (largeur_rect + espacement)
                        rect = patches.Rectangle(
                            (x, y_base), largeur_rect, hauteur_rect,
                            linewidth=0, edgecolor='none', facecolor=couleurs[j], alpha=0.9
                        )
                        ax.add_patch(rect)

                # Texte pour la salve
                largeur_totale = 5 * largeur_rect + 4 * espacement
                x_centre_salve = x_depart + largeur_totale / 2
                ax.text(
                    x_centre_salve, y_base + hauteur_rect + texte_offset_y,
                    f"{Batteries[i][3]}%",
                    color='black', fontsize=8, ha='center', va='bottom'
                )
            print("")
                

            plt.draw()
            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.01)

    except KeyboardInterrupt:
        """etat = 2
        data_etat = struct.pack('<I',etat)
        bus.send(can.Message(arbitration_id=0x01, data=data_etat, is_extended_id=False))"""

        print("Arrêt demandé par l'utilisateur.")
        stop_event.set()  # signal aux threads de s'arrêter
        # Attente que chaque thread termine proprement
        tache_lidar.join()
        #tache_odometrie.join()
        #tache_calcul.join()
        #os.system("sudo ifconfig can0 down")
        print("Programme terminé proprement.")


########################################################################
