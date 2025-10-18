# lidar.py
from rplidar import RPLidar
import math
import queue
import threading

def init_lidar(port, baudrate):
    """
    Initialise et renvoie un objet RPLidar sans démarrer le moteur.
    """
    return RPLidar(port, baudrate=baudrate)

def start_lidar_thread(lidar, pile_calcul, stop_event,
                       get_robot_position,
                       buffer_Lidar=4096, marge_bordurepiste=15):
    """
    Démarre un thread qui lit en continu les scans du Lidar et remplit la pile.

    Args:
        lidar: objet RPLidar
        pile_calcul: queue.Queue() pour stocker les points
        stop_event: threading.Event() pour arrêter le thread
        get_robot_position: fonction retournant (x_robot, y_robot, angle_robot)
        buffer_Lidar: nombre max de mesures par scan
        marge_bordurepiste: marge pour ignorer les bords de la piste
    """
    def lidar_thread():
        try:
            lidar.start_motor()
            while not stop_event.is_set():
                for scan in lidar.iter_scans(scan_type='express', max_buf_meas=buffer_Lidar):
                    if stop_event.is_set():
                        break
                    x_robot, y_robot, angle_robot = get_robot_position()
                    for quality, angle_point, distance in scan:
                        phi = math.radians(angle_point)
                        angle_total = phi - math.radians(angle_robot) - math.radians(11)

                        x_point = x_robot + distance * math.cos(angle_total)
                        y_point = y_robot - distance * math.sin(angle_total)

                        x_point = max(0, min(3000, int(x_point)))
                        y_point = max(0, min(2000, int(y_point)))

                        if marge_bordurepiste <= x_point <= 3000 - marge_bordurepiste and \
                           marge_bordurepiste <= y_point <= 2000 - marge_bordurepiste:
                            pile_calcul.put((x_point, y_point))

        except Exception as e:
            print("Erreur dans le thread Lidar:", e)
        finally:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
            print("Thread Lidar terminé")

    thread = threading.Thread(target=lidar_thread, daemon=True)
    thread.start()
    return thread
