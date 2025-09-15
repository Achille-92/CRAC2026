
################## Librairies ##########################################
import matplotlib.pyplot as plt
from rplidar import RPLidar
import numpy as np
########################################################################

# Port série du lidar (vérifie avec dmesg | grep tty)
PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 256000

lidar = None

angles = []                                                             #   Création d'une liste pour les angles
distances = []                                                          #   Création d'une liste pour les distances

################## Fonction run() ######################################
def run():
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE)                       # Instanciation de l'objet "lidar" par la classe "RPLidar" en effectuant la connexion au port série
    
    print("INFO:", lidar.get_info())
    print("HEALTH:", lidar.get_health())

    lidar.start_motor()
    global angles, distances

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})          # Création d'une figure (fig) et de zone de dessin avec axes (ax), en précisant que l'on souhaite une projection polaire
    ax.set_ylim(0, 6000)  # portée max du lidar en mm                   # On définit la valeur max d'affichage du rayon "r", comme à 6000mm soit 6m
    ax.set_title("RPLIDAR A2 - Scan en temps réel")                     # Ajout d'un titre au graphique
    points, = ax.plot([], [], 'bo', markersize=2)                       
    # On traçe des points sur la zone de dessin "ax". Rien au début ([] et []). "bo" et "markersize=2" pour indiquer des cercles bleus de taille 2
    # "points, " récupère une liste de points qui pourra être mise à jour et exploitée.
    
    try:                                                                # On essaie
        for scan in lidar.iter_scans(scan_type='express', max_buf_meas=1000): # scan = [(quality,distance,angle),(quality,distance,angle), ... ,(quality,distance,angle)] pour un tour entier
            for (quality, angle, distance) in scan:                           #   Pour chaque quality, angle et distance dans le scan
                angles.append(np.radians(angle))                        #       Ajoute l'angle EN RADIANS dans la liste "angles"
                distances.append(distance)                              #       Ajoute la distance dans la liste "distances"
                if distance < 400:
                    print("Distance : ",distance, "   Angle : ",int(angle))

            #points.set_data(angles, distances)                          #   Ajoute dans la liste de points la distance et l'angle qui sont associés
            #plt.pause(0.01)                                             #   Met à jour la fenêtre graphique, ~100 fps (limité par 10 Hz du lidar)
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
