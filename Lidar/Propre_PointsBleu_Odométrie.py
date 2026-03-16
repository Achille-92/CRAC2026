################## Librairies ##########################################
from rplidar import RPLidar
from pyrplidar import PyRPlidar
import math
import numpy as np
import matplotlib.pyplot as plt
import threading, queue
from collections import deque
import time
import os
import can
import struct
########################################################################
Can = False
# config CAN
if Can:
    os.system('sudo ip link set can0 type can bitrate 500000')  # adapte le bitrate
    os.system('sudo ifconfig can0 up')
    bus = can.interface.Bus(
        channel='can0',
        bustype='socketcan',
        bitrate=500000,
        can_filters=[{"can_id": 0x10, "can_mask": 0x7FF, "extended": False},
                    {"can_id": 0x11, "can_mask": 0x7FF, "extended": False},
                    {"can_id": 0x12, "can_mask": 0x7FF, "extended": False}]
    )


# Port série et Baudrate du lidar
PORT_NAME = '/dev/ttyUSB0'
PORT_NAME = 'COM14'
BAUDRATE = 1000000

# Création de l'objet Lidar, et de la Pile pile_points
lidar = None
pile_points = queue.Queue(maxsize=50)

LARGEUR_ROBOT = 250
LONGUEUR_ROBOT = 130
# Coordonnées et angle de notre robot
x_robot = int(2400+LARGEUR_ROBOT/2)
y_robot = int(1550+LONGUEUR_ROBOT/2+100)
angle_robot = 90

# Coordonnées, angle et vitesse du robot ennemi
x_ennemi = 0
y_ennemi = 0
angle_ennemi = 0
v_ennemi = 0

# Objets et variables pour la fenêtre graphique
fig = None 
ax = None
robot_plot = None
scat = None

################## Fonction  ###########################################
def calcul_points(stop_event):
    """
    Argument : flag "stop_event"
    Modification : variables globales "pile_points"

    Utilisation :
    Création de l'objet "lidar"
    Calcul de l'angle total et des coordonnées des points
    Saturation des valeurs pour les limites de l'aire de jeu, puis pour oublier les bords
    """

    lidar = PyRPlidar()

    global  x_robot, y_robot, angle_robot
    x_point = 0
    y_point = 0

    while not stop_event.is_set():
        try:
            lidar.connect(port=PORT_NAME, baudrate=BAUDRATE, timeout=3)
            lidar.set_motor_pwm(500)
            time.sleep(1)
            
            scan_generator = lidar.start_scan()
            print("Lidar démarré, lecture des points...")
            for scan in scan_generator():
                angle_point = scan.angle
                distance = scan.distance
                quality = scan.quality
                if stop_event.is_set():
                    break
                x_r = x_robot
                y_r = y_robot
                angle_r = angle_robot
                
                phi = math.radians(angle_point)                                 # On converti l'angle de la mesure en radian
                angle_total = phi - math.radians(angle_r) - math.radians(1)    # On calcule l'angle total à partir de l'orientation du Lidar et du robot

                x_point = x_r + distance * math.cos(angle_total)                # On calcule les coordonnées x et y du point à partir de la position et de l'orientation du robot
                y_point = y_r - distance * math.sin(angle_total)

                # Saturation dans le repère (0 ≤ x ≤ 3000, 0 ≤ y ≤ 2000)
                x_point = max(0, min(3000, int(x_point)))
                y_point = max(0, min(2000, int(y_point)))
                distance_robot_point = math.sqrt((x_r - x_point)**2 + (y_r - y_point)**2)
                if 120 <= x_point <= 3000-120 and 80 <= y_point <= 2000-80 and distance_robot_point > 40:                 # Si ce ne sont pas les murs, on ajoute le point dans la pile sous forme de tuple (x,y)
                    pile_points.put((x_point, y_point))

        except Exception as e:                                                      # En cas d'exception on affiche l'erreur
            print("Erreur dans le thread Lidar:", e)
        finally:                                                                    # Et on arrête le Lidar
            # Nettoyage du Lidar
            print("Arrêt du Lidar...")
            lidar.set_motor_pwm(0)
            time.sleep(1)
            lidar.stop()
            lidar.disconnect()


def affichage(stop_event):
    """
    Arguments : x_robot, y_robot, flag stop_event
    Modifications : robot_plot, ennemi_plot, ennemi_vecteur, fig, ax

    Utilisation :
    Créée la fenêtre graphique avec les légendes, et affiche notre robot
    Affiche le robot, et le robot ennemi
    Calcule puis affiche le vecteur direction
    Refresh toutes les 10ms
    """
    global x_ennemi, y_ennemi, angle_ennemi
    global robot_plot, ennemi_plot, ennemi_vecteur

    fig, ax = plt.subplots()
    plt.ion()
    plt.show()

    # Scatter pour les objets
    robot_plot = ax.scatter([x_robot], [y_robot], s=50, c='red', marker='x')
    ennemi_plot = ax.scatter([], [], s=50, c='green', marker='o')
    ennemi_vecteur, = ax.plot([], [], c='orange', linewidth=2)
    scat = ax.scatter([], [], s=5, c='blue', alpha=0.5)

    ax.set_xlim(0, 3000)
    ax.set_ylim(0, 2000)
    ax.set_aspect('equal')

    buffer_points = deque(maxlen=500)

    while not stop_event.is_set():
        try:
            p = pile_points.get(timeout=0.1)
            buffer_points.append(p)
        except queue.Empty:
            pass

        while not pile_points.empty():
            buffer_points.append(pile_points.get_nowait())

        if buffer_points:
            xs, ys = zip(*buffer_points)
            scat.set_offsets(np.c_[xs, ys])

            # met à jour la position du robot (fixe dans ton cas)
            robot_plot.set_offsets([[x_robot, y_robot]])

            fig.canvas.draw()
            fig.canvas.flush_events()

def CAN_Odometrie(stop_event):
    """
    Réception CAN pour mettre à jour x_robot, y_robot et angle_robot
    """
    global x_robot, y_robot, angle_robot

    while not stop_event.is_set():
        msg = bus.recv(0.01)  # attend 10 ms max
        if msg is None:
            continue  # pas de message, on repart

        # Vérifie qu'on a bien reçu 4 octets avant de décoder
        if msg.arbitration_id not in (0x100, 0x101, 0x102):
            continue  # on saute les autres trames

        if msg.arbitration_id == 0x100:
            x_robot = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x101:
            y_robot = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x102:
            angle_robot = struct.unpack('f', bytes(msg.data))[0]

########################################################################

################## Lancement du programme principal ####################

if __name__ == '__main__':

    stop_event = threading.Event()

    # Thread Lidar
    tache_lidar = threading.Thread(target=calcul_points, args=(stop_event,), daemon=False)
    # Thread affichage
    tache_affichage = threading.Thread(target=affichage, args=(stop_event,), daemon=False)
    if Can:
        tache_odometrie = threading.Thread(target=CAN_Odometrie, args=(stop_event,), daemon=True)

    tache_lidar.start()
    tache_affichage.start()
    if Can:
        tache_odometrie.start()


    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur.")
        stop_event.set()  # signal aux threads de s'arrêter
        # Attente que chaque thread termine proprement
        tache_lidar.join()
        tache_affichage.join()
        if Can:
            tache_odometrie.join()
        print("Programme terminé proprement.")


########################################################################
