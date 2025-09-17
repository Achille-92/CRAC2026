################## Librairies ##########################################
from rplidar import RPLidar
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
            for (quality, angle_point, distance) in scan:

                # Conversion angle lidar → trigonométrique
                phi = math.radians(90 - angle_point)   
                angle_total = angle_robot + phi        
            
                # Coordonnées globales du point
                x_point = x_robot + distance * math.cos(angle_total)
                y_point = y_robot + distance * math.sin(angle_total)
            
                # Conversion angle en degrés modulo 360 pour indexer la liste
                index_angle = int(math.degrees(angle_total)) % 360
            
                liste_points[index_angle] = (x_point, y_point)
                print(f"Point angle {index_angle}° : {liste_points[index_angle]}")


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

