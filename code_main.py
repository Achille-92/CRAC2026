################## Librairies ##########################################
from rplidar import RPLidar
import math
import threading, queue
from collections import deque

import os
import sys

if './CRAC2026' not in sys.path:
    sys.path.insert(0, './CRAC2026')
import AffichagePiste
########################################################################

################## Fonction run() ######################################
def prise_de_points(PORT_NAME,BAUDRATE):
    global liste_points, x_robot, y_robot, angle_robot
    x_point = 0
    y_point = 0
    angle_total = 0

    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE) 

    print("INFO:", lidar.get_info())
    print("HEALTH:", lidar.get_health())

    try:                                                                # On essaie
        lidar.start_motor()
        for scan in lidar.iter_scans(scan_type='express', max_buf_meas=4096): # scan = [(quality,distance,angle),(quality,distance,angle), ... ,(quality,distance,angle)] pour un tour entier
            for (quality, angle_point, distance) in scan:
                phi = math.radians(angle_point)   # angle_point en degrés (Lidar)
                angle_total = phi - math.radians(angle_robot)        # tout en radians    
                x_point = x_robot + distance * math.cos(angle_total)
                y_point = y_robot - distance * math.sin(angle_total)
                
                # Saturation dans le repère (0 ≤ x ≤ 3000, 0 ≤ y ≤ 2000)
                x_point = max(0, min(3000, int(x_point)))
                y_point = max(0, min(2000, int(y_point)))
                
                pile_points.put((x_point, y_point))  # envoie le point dans la queue


    except KeyboardInterrupt:                                           # Sauf en cas d'erreur d'interruption
        print("Arrêt demandé par l'utilisateur")                        
    finally:                                                            # À la fin 
        lidar.stop()                                                    #   On arrête le Lidar
        lidar.stop_motor()
        lidar.disconnect()                                              #   Et on le déconnecte

########################################################################




################## Lancement du programme principal ####################

if __name__ == '__main__':
    # Port série du lidar (vérifie avec dmesg | grep tty)
    PORT_NAME = '/dev/ttyUSB0'
    BAUDRATE = 256000

    pile_points = queue.Queue()
    liste_points = [(0,0) for n in range(360)]
    
    x_robot = 300
    y_robot = 300
    angle_robot = 0

    AffichagePiste.initialisation_Affichage(x_robot,y_robot)
    detection_robot = threading.Thread(target=prise_de_points(PORT_NAME,BAUDRATE), daemon=True)
    affichage_robot = threading.Thread(target=AffichagePiste.affichage(pile_points,x_robot,y_robot), daemon=True)
    detection_robot.start()
    affichage_robot.start()

########################################################################
