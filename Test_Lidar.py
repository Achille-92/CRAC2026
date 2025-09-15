
################## Librairies ##########################################
import matplotlib.pyplot as plt
from rplidar import RPLidar
import numpy as np
import math
########################################################################

# Port série du lidar (vérifie avec dmesg | grep tty)
PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 256000

lidar = None

angles = []                                                             #   Création d'une liste pour les angles
distances = []                                                          #   Création d'une liste pour les distances
liste_points = [(0,0) for n in range(360)]

x_robot = 100
y_robot = 100
angle_robot = 45

################## Fonction run() ######################################
def run():
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE)                       # Instanciation de l'objet "lidar" par la classe "RPLidar" en effectuant la connexion au port série
    
    print("INFO:", lidar.get_info())
    print("HEALTH:", lidar.get_health())

    lidar.start_motor()
    global angles, distances, liste_points, x_robot, y_robot, angle_robot
    x_point = 0
    y_point = 0
    cpt = 0
    angle_total = 0

    try:                                                                # On essaie
        for scan in lidar.iter_scans(scan_type='express', max_buf_meas=1000): # scan = [(quality,distance,angle),(quality,distance,angle), ... ,(quality,distance,angle)] pour un tour entier
            for (quality, angle, distance) in scan:                           #   Pour chaque quality, angle et distance dans le scan
                angles.append(np.radians(angle))                        #       Ajoute l'angle EN RADIANS dans la liste "angles"
                distances.append(distance)                              #       Ajoute la distance dans la liste "distances"
                if distance < 400:
                    angle_total = angle + angle_robot
                    x_point = distance * math.cos(np.radians(angle_total))
                    y_point = distance * math.sin(np.radians(angle_total))
                    liste_points[cpt] = (x_point+x_robot,y_point+y_robot)
                    print(liste_points[cpt])
                    cpt = (cpt+1)%360

    except KeyboardInterrupt:                                           # Sauf en cas d'erreur d'interruption
        print("Arrêt demandé par l'utilisateur")                        
    finally:                                                            # À la fin 
        lidar.stop()                                                    #   On arrête le Lidar
        lidar.stop_motor()
        lidar.disconnect()                                              #   Et on le déconnecte

########################################################################


################## Lancement du programme principal ####################

if __name__ == '__main__':
    run()
else :
    lidar.stop()                                                    #   On arrête le Lidar
    lidar.stop_motor()
    lidar.disconnect()                                              #   Et on le déconnecte
########################################################################
