from rplidar import RPLidar
import math
import queue

def init_lidar(port, baudrate):
    """
    Initialise et renvoie un objet RPLidar.
    """
    lidar = RPLidar(port, baudrate=baudrate)
    return lidar

def calcul_points(lidar, pile_calcul, stop_event, x_robot, y_robot, angle_robot,buffer_Lidar, marge_bordurepiste):
    """
    Argument : flag "stop_event"
    Modification : variables globales "pile_points"

    Utilisation :
    Création de l'objet "lidar"
    Calcul de l'angle total et des coordonnées des points
    Saturation des valeurs pour les limites de l'aire de jeu, puis pour oublier les bords
    """

    print("INFO:", lidar.get_info())                                    # Affichage d'informations propres au Lidar
    print("HEALTH:", lidar.get_health())

    lidar.start_motor()                                                 # Démarrage du moteur du Lidar

    x_point = 0
    y_point = 0

    try:
        # On lit les scans tant que le stop_event n’est pas activé
        for scan in lidar.iter_scans(scan_type='express', max_buf_meas=buffer_Lidar):
            if stop_event.is_set():   # si on demande l’arrêt → on sort
                break

            for (quality, angle_point, distance) in scan:                       # Pour chaque points dans le scan
                phi = math.radians(angle_point)                                 # On converti l'angle de la mesure en radian
                angle_total = phi - math.radians(angle_robot) - math.radians(11)    # On calcule l'angle total à partir de l'orientation du Lidar et du robot

                x_point = x_robot + distance * math.cos(angle_total)                # On calcule les coordonnées x et y du point à partir de la position et de l'orientation du robot
                y_point = y_robot - distance * math.sin(angle_total)

                # Saturation dans le repère (0 ≤ x ≤ 3000, 0 ≤ y ≤ 2000)
                x_point = max(0, min(3000, int(x_point)))
                y_point = max(0, min(2000, int(y_point)))

                if marge_bordurepiste <= x_point <= 3000-marge_bordurepiste and marge_bordurepiste <= y_point <= 2000-marge_bordurepiste:                 # Si ce ne sont pas les murs, on ajoute le point dans la pile sous forme de tuple (x,y)
                    pile_calcul.put((x_point, y_point))

    except Exception as e:                                                      # En cas d'exception on affiche l'erreur
        print("Erreur dans le thread Lidar:", e)
    finally:                                                                    # Et on arrête le Lidar
        # Nettoyage du Lidar
        print("Arrêt du Lidar...")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
