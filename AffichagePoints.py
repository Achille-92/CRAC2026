################## Librairies ##########################################
from rplidar import RPLidar
import numpy as np
import math
import matplotlib.pyplot as plt
########################################################################

# Port série du lidar (vérifie avec dmesg | grep tty)
PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 256000

lidar = None

liste_points = [(0,0) for n in range(360)]

x_robot = 300
y_robot = 300
angle_robot = 0

fig, ax = plt.subplots()
scat = ax.scatter([], [], s=5, c='blue')
ax.set_xlim(0, 3000)
ax.set_ylim(0, 2000)
ax.set_aspect('equal')

################## Fonction run() ######################################
def run():
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE)                       # Instanciation de l'objet "lidar" par la classe "RPLidar" en effectuant la connexion au port série
    
    print("INFO:", lidar.get_info())
    print("HEALTH:", lidar.get_health())

    lidar.start_motor()
    global liste_points, x_robot, y_robot, angle_robot
    x_point = 0
    y_point = 0
    
    angle_total = 0

    try:                                                                # On essaie
        tour = 0
        for scan in lidar.iter_scans(scan_type='express', max_buf_meas=1000): # scan = [(quality,distance,angle),(quality,distance,angle), ... ,(quality,distance,angle)] pour un tour entier
            tour += 1
            liste_points = []
            for (quality, angle, distance) in scan:
                theta = angle              # angle mesuré par le Lidar
                alpha = angle_robot        # orientation du robot
                angle_total = (theta + alpha) % 360
                
                # Coordonnées globales
                x_point = x_robot + distance * math.sin(math.radians(angle_total))
                y_point = y_robot + distance * math.cos(math.radians(angle_total))
                
                # Saturation dans le repère (0 ≤ x ≤ 3000, 0 ≤ y ≤ 2000)
                x_point = max(0, min(3000, int(x_point)))
                y_point = max(0, min(2000, int(y_point)))
                
                liste_points[int(angle_total)] = (x_point, y_point)
                print("Point N°", int(angle_total), liste_points[int(angle_total)])
                
            if tour % 5 == 0:   # affiche 1 tour sur 5
                afficher_points(liste_points)

    except KeyboardInterrupt:                                           # Sauf en cas d'erreur d'interruption
        print("Arrêt demandé par l'utilisateur")                        
    finally:                                                            # À la fin 
        lidar.stop()                                                    #   On arrête le Lidar
        lidar.stop_motor()
        lidar.disconnect()                                              #   Et on le déconnecte

########################################################################


def afficher_points(liste_points):
    xs = [p[0] for p in liste_points]
    ys = [p[1] for p in liste_points]
    scat.set_offsets(np.c_[xs, ys])   # met à jour les données
    plt.pause(0.01)

################## Lancement du programme principal ####################

if __name__ == '__main__':
    run()
else :
    lidar.stop()                                                    #   On arrête le Lidar
    lidar.stop_motor()
    lidar.disconnect()                                              #   Et on le déconnecte
########################################################################
