from pyrplidar import PyRPlidar
import math
import numpy as np
import matplotlib.pyplot as plt
from multiprocessing import Process, Queue, Event, Value
import ctypes
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

# Constantes (pas besoin de les partager car elles ne changent pas)
X_PISTE = 3000
Y_PISTE = 2000
MARGE_BORDUREPISTE_X = 120
MARGE_BORDUREPISTE_Y = 80
LARGEUR_ROBOT = 250
LONGUEUR_ROBOT = 130

# Fonction pour récupérer les données LiDAR
def calcul_points(stop_event, pile_points, x_robot, y_robot, angle_robot, x_ennemi, y_ennemi):
    """
    Fonction exécutée dans un processus séparé.
    Tous les objets partagés sont passés en paramètres.
    """
    buffer_points = deque(maxlen=50)
    dict_points = {i: 0 for i in range(360)}
    
    try:
        lidar = PyRPlidar()
        lidar.connect(port=PORT_NAME, baudrate=BAUDRATE, timeout=3)
        print("INFO:", lidar.get_health())
        lidar.stop()
        time.sleep(0.5)
        lidar.disconnect()
        time.sleep(0.5)
        lidar.connect(port=PORT_NAME, baudrate=BAUDRATE, timeout=3)

        lidar.set_motor_pwm(660)
        time.sleep(5)
    
        scan_generator = lidar.start_scan_express(0)

        for count, scan in enumerate(scan_generator()):  
            if stop_event.is_set():
                break
            flag = scan.start_flag
            quality = scan.quality
            angle_point = scan.angle
            distance = scan.distance
            
            if flag == True:
                # Lecture des valeurs partagées avec .value
                x_rob = x_robot.value
                y_rob = y_robot.value
                angle_rob = angle_robot.value
                
                for angle_point, distance in dict_points.items():
                    phi = math.radians(angle_point)
                    angle_total = phi - math.radians(angle_rob) - math.radians(-3)

                    x_point = x_rob + distance * math.cos(angle_total)
                    y_point = y_rob - distance * math.sin(angle_total)

                    # Saturation dans le repère (0 ≤ x ≤ 3000, 0 ≤ y ≤ 2000)
                    x_point = max(0, min(3000, int(x_point)))
                    y_point = max(0, min(2000, int(y_point)))

                    # Filtrage des points (on ignore les bords)
                    distance_robot_point = math.sqrt((x_rob - x_point)**2 + (y_rob - y_point)**2)
                    if MARGE_BORDUREPISTE_X <= x_point <= X_PISTE-MARGE_BORDUREPISTE_X and \
                       MARGE_BORDUREPISTE_Y <= y_point <= Y_PISTE-MARGE_BORDUREPISTE_Y and distance_robot_point > 50:
                        if not(600 < x_point < 2400 and 1550 < y_point < 2000):
                            buffer_points.append((int(x_point), int(y_point)))
                            pile_points.put((int(x_point), int(y_point)))

                dict_points = {i: 0 for i in range(360)}
                
                if buffer_points:
                    xs, ys = zip(*buffer_points)
                    # Écriture des valeurs partagées avec .value
                    x_ennemi.value = np.mean(xs)
                    y_ennemi.value = np.mean(ys)
                    
            if distance > 4000:  
                continue
            if quality < 1:
                continue
            
            if dict_points[int(angle_point)] == 0:
                dict_points[int(angle_point)] = distance
            
    except Exception as e:
        print("Erreur dans le processus LiDAR:", e)
    finally:
        lidar.stop()
        lidar.disconnect()
        print("LiDAR arrêté.")

# Fonction pour la réception CAN (si activée)
def CAN_Odometrie(stop_event, x_robot, y_robot, angle_robot):
    """
    Fonction exécutée dans un processus séparé.
    Met à jour les valeurs partagées depuis le bus CAN.
    """
    while not stop_event.is_set():
        msg = bus.recv(0.01)
        if msg is None:
            continue

        # Écriture des valeurs partagées avec .value
        if msg.arbitration_id == 0x10:
            x_robot.value = struct.unpack('f', bytes(msg.data))[0]
        elif msg.arbitration_id == 0x11:
            y_robot.value = struct.unpack('f', bytes(msg.data))[0]
        elif msg.arbitration_id == 0x12:
            angle_robot.value = struct.unpack('f', bytes(msg.data))[0]

# Fonction pour l'affichage en temps réel (dans le processus principal)
def affichage(stop_event, pile_points, x_robot, y_robot, x_ennemi, y_ennemi):
    """
    Fonction d'affichage exécutée dans le processus principal.
    Lit les valeurs partagées pour mettre à jour l'affichage.
    """
    fig, ax = plt.subplots()
    plt.ion()
    plt.show(block=False)

    # Lecture des valeurs initiales avec .value
    robot_plot = ax.scatter([x_robot.value], [y_robot.value], s=50, c='red', marker='x')
    ennemi_plot = ax.scatter([x_ennemi.value], [y_ennemi.value], s=50, c='green', marker='o')
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
        except:
            pass

        while not pile_points.empty():
            try:
                buffer_points.append(pile_points.get_nowait())
            except:
                break

        if buffer_points:
            xs, ys = zip(*buffer_points)
            scat.set_offsets(np.c_[xs, ys])
            # Lecture des valeurs partagées avec .value
            robot_plot.set_offsets([[x_robot.value, y_robot.value]])
            ennemi_plot.set_offsets([[x_ennemi.value, y_ennemi.value]])
            fig.canvas.draw_idle()
            plt.pause(0.2)

# Programme principal
if __name__ == '__main__':
    import multiprocessing as mp
    mp.set_start_method('spawn')
    
    # Création des objets partagés entre processus
    stop_event = Event()
    pile_points = Queue(maxsize=500)
    
    # Création des Value pour les coordonnées (utilisent ctypes pour le partage)
    x_robot = Value(ctypes.c_float, float(2400 + LARGEUR_ROBOT/2))
    #x_robot = Value(ctypes.c_float, float(1500))
    y_robot = Value(ctypes.c_float, float(1550 + LONGUEUR_ROBOT/2 + 100))
    #y_robot = Value(ctypes.c_float, float(1700))
    angle_robot = Value(ctypes.c_float, -90.0)
    
    x_ennemi = Value(ctypes.c_float, 0.0)
    y_ennemi = Value(ctypes.c_float, 0.0)

    # Création des processus (au lieu de threads)
    tache_lidar = Process(
        target=calcul_points, 
        args=(stop_event, pile_points, x_robot, y_robot, angle_robot, x_ennemi, y_ennemi),
        daemon=True
    )

    if Can:
        tache_odometrie = Process(
            target=CAN_Odometrie, 
            args=(stop_event, x_robot, y_robot, angle_robot),
            daemon=True
        )

    # Démarrage des processus
    tache_lidar.start()

    if Can:
        tache_odometrie.start()

    # L'affichage est exécuté dans le processus principal
    affichage(stop_event, pile_points, x_robot, y_robot, x_ennemi, y_ennemi)

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur.")
        stop_event.set()
        tache_lidar.join(timeout=2)
        if Can:
            tache_odometrie.join(timeout=2)
        print("Programme terminé proprement.")