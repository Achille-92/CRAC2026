
################## Librairies ##########################################
import matplotlib.pyplot as plt
from rplidar import RPLidar
import numpy as np
import math
import time
########################################################################

# Port série du lidar (vérifie avec dmesg | grep tty)
PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 256000

lidar = None

angles = []                                                             #   Création d'une liste pour les angles
distances = []                                                          #   Création d'une liste pour les distances
liste_points = [(0,0) for _ in range(360)]

x_robot = 100
y_robot = 100
angle_robot = 45

plt.ion()  # mode interactif
fig, ax = plt.subplots(figsize=(1000,1000), facecolor="white")

################## Fonction run() ######################################
def run():
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE)
    
    print("INFO:", lidar.get_info())
    print("HEALTH:", lidar.get_health())

    lidar.start_motor()
    global angles, distances, liste_points, x_robot, y_robot, angle_robot
    x_point = 0
    y_point = 0
    cpt = 0
    angle_total = 0

    # préparation figure
    ax.set_facecolor("white")
    ax.grid(True, linestyle="--", alpha=0.5)

    # scatter initial (points + robot)
    sc_points = ax.scatter([], [], color="blue", s=5, label="Points")
    sc_robot = ax.scatter([x_robot], [y_robot], color="red", s=50, label="Robot")
    ax.legend()

    try:
        for scan in lidar.iter_scans(scan_type='express', max_buf_meas=1000):
            for (quality, angle, distance) in scan:
                angles.append(np.radians(angle))
                distances.append(distance)
                if distance < 400:
                    angle_total = angle + angle_robot
                    x_point = distance * math.cos(np.radians(angle_total))
                    y_point = distance * math.sin(np.radians(angle_total))
                    liste_points[cpt] = (x_point+x_robot, y_point+y_robot)
                    cpt = (cpt+1) % 360

            # mise à jour des scatter sans effacer la figure
            if liste_points:
                sc_points.set_offsets(liste_points)
            sc_robot.set_offsets([[x_robot, y_robot]])

            plt.draw()
            plt.pause(0.01)  # petit délai pour garder la boucle fluide

    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()


########################################################################


################## Lancement du programme principal ####################

if __name__ == '__main__':
    run()
else :
    lidar.stop()                                                    #   On arrête le Lidar
    lidar.stop_motor()
    lidar.disconnect()                                              #   Et on le déconnecte
########################################################################
