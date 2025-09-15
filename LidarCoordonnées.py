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

liste_points = [(0,0) for n in range(360)]

x_robot = 300
y_robot = 300
angle_robot = 0

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
        for scan in lidar.iter_scans(scan_type='express', max_buf_meas=1000): # scan = [(quality,distance,angle),(quality,distance,angle), ... ,(quality,distance,angle)] pour un tour entier
            for (quality, angle, distance) in scan:                           #   Pour chaque quality, angle et distance dans le scan
                angle_total = (angle + angle_robot)%360
                x_point = distance * math.sin(np.radians(angle_total))
                y_point = distance * math.cos(np.radians(angle_total))
                if x_point >= 0 and x_point <= 3000 and y_point >= 0 and y_point <= 2000:
                    liste_points[int(angle_total)] = (int(x_point+x_robot),int(y_point+y_robot))
                    print("Point N° ",int(angle_total), " ",liste_points[int(angle_total)])
                else :
                    liste_points[int(angle_total)] = ("XXX","XXX")

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
