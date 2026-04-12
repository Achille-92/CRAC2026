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

# Config CAN (désactivée par défaut)
Can = False
if Can:
    os.system('sudo ip link set can0 type can bitrate 500000')
    os.system('sudo ifconfig can0 up')
    bus = can.interface.Bus(
        channel='can0',
        bustype='socketcan',
        bitrate=500000,
        can_filters=[{"can_id": 0x10, "can_mask": 0x7FF, "extended": False},
                    {"can_id": 0x11, "can_mask": 0x7FF, "extended": False},
                    {"can_id": 0x12, "can_mask": 0x7FF, "extended": False}]
    )

# Port série et Baudrate du LiDAR
import platform
current_os = platform.system()
if current_os == "Linux":
    PORT_NAME = '/dev/ttyUSB0'
else:
    PORT_NAME = 'COM14'
BAUDRATE = 1000000

# Pile pour les points LiDAR
pile_points = queue.Queue(maxsize=2000)

# Coordonnées et angle du robot
LARGEUR_ROBOT = 250
LONGUEUR_ROBOT = 130
x_robot = int(2400+LARGEUR_ROBOT/2+50)
y_robot = int(1550+LONGUEUR_ROBOT/2+100)

x_robot = 1500
y_robot = 1700
angle_robot = 90

# Coordonnées et angle du robot ennemi
x_ennemi = 0
y_ennemi = 0
angle_ennemi = 0
v_ennemi = 0

# Variables pour l'affichage
fig = None
ax = None
robot_plot = None
scat = None

# Fonction pour récupérer les données LiDAR

def calcul_points(stop_event):
    global x_robot, y_robot, angle_robot, pile_points

    dict_points = {i: 0 for i in range(360)}
    try:
        lidar = PyRPlidar()
        lidar.connect(port=PORT_NAME, baudrate=BAUDRATE, timeout=3)
        print("INFO:", lidar.get_health())

        lidar.set_motor_pwm(660)
        time.sleep(2)
    
        #scan_generator = lidar.start_scan_express(0)
        scan_generator = lidar.start_scan()
        time.sleep(0.5)

        for count, scan in enumerate(scan_generator()):  
            if stop_event.is_set():
                break
            flag = scan.start_flag
            quality = scan.quality
            angle_point = scan.angle
            distance = scan.distance
            if flag == True:
                for angle_point, distance in dict_points.items():
                    phi = math.radians(angle_point)
                    angle_total = phi - math.radians(angle_robot) - math.radians(0)

                    x_point = x_robot + distance * math.cos(angle_total)
                    y_point = y_robot - distance * math.sin(angle_total)

                    # Saturation dans le repère (0 ≤ x ≤ 3000, 0 ≤ y ≤ 2000)
                    x_point = max(0, min(3000, int(x_point)))
                    y_point = max(0, min(2000, int(y_point)))

                    # Filtrage des points (on ignore les bords)
                    if (120 <= x_point <= 2880 and 80 <= y_point <= 1920) and distance > 10:
                        pile_points.put((int(x_point), int(y_point)))

                dict_points = {i: 0 for i in range(360)}
                print("Début de tour")
            if distance > 4000:  
                continue
            if quality < 1:
                continue
            
            if dict_points[int(angle_point)] == 0:
                dict_points[int(angle_point)] = distance
                print(f"[{int(angle_point)}]: ",dict_points[int(angle_point)])
            

            
    except Exception as e:
        print("Erreur dans le thread LiDAR:", e)
    finally:
        lidar.stop()
        lidar.disconnect()
        print("LiDAR arrêté.")

# Fonction pour la réception CAN (si activée)
def CAN_Odometrie(stop_event):
    global x_robot, y_robot, angle_robot

    while not stop_event.is_set():
        msg = bus.recv(0.01)
        if msg is None:
            continue

        if msg.arbitration_id == 0x10:
            x_robot = struct.unpack('f', bytes(msg.data))[0]
        elif msg.arbitration_id == 0x11:
            y_robot = struct.unpack('f', bytes(msg.data))[0]
        elif msg.arbitration_id == 0x12:
            angle_robot = struct.unpack('f', bytes(msg.data))[0]

# Fonction pour l'affichage en temps réel (à exécuter dans le thread principal)
def affichage(stop_event):
    global x_robot, y_robot, x_ennemi, y_ennemi, angle_ennemi, fig, ax, robot_plot, scat

    fig, ax = plt.subplots()
    plt.ion()
    plt.show(block=False)  # Utilisation de block=False pour éviter le blocage

    robot_plot = ax.scatter([x_robot], [y_robot], s=50, c='red', marker='x')
    ennemi_plot = ax.scatter([], [], s=50, c='green', marker='o')
    ennemi_vecteur, = ax.plot([], [], c='orange', linewidth=2)
    scat = ax.scatter([], [], s=5, c='blue', alpha=0.5)

    ax.set_xlim(0, 3000)
    ax.set_ylim(0, 2000)
    ax.set_aspect('equal')

    buffer_points = deque(maxlen=100)

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
            robot_plot.set_offsets([[x_robot, y_robot]])
            fig.canvas.draw_idle()  # Utilisation de draw_idle pour éviter les conflits de thread
            plt.pause(0.01)  # Pause pour permettre la mise à jour de l'interface

# Programme principal
if __name__ == '__main__':
    stop_event = threading.Event()

    tache_lidar = threading.Thread(target=calcul_points, args=(stop_event,), daemon=True)

    if Can:
        tache_odometrie = threading.Thread(target=CAN_Odometrie, args=(stop_event,), daemon=True)

    tache_lidar.start()

    if Can:
        tache_odometrie.start()

    # L'affichage est exécuté dans le thread principal
    affichage(stop_event)

    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur.")
        stop_event.set()
        tache_lidar.join()
        if Can:
            tache_odometrie.join()
        print("Programme terminé proprement.")

#abcd