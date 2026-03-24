################## Librairies ##########################################
from rplidar import RPLidar
from pyrplidar import PyRPlidar
import math
import numpy as np
import matplotlib.pyplot as plt
import threading, queue
from collections import deque
import time
import os
import can
import struct
########################################################################
Can = False
# config CAN
if Can:
    os.system('sudo ip link set can0 type can bitrate 500000')
    os.system('sudo ifconfig can0 up')
    bus = can.interface.Bus(
        channel='can0',
        bustype='socketcan',
        bitrate=500000,
        can_filters=[{"can_id": 0x100, "can_mask": 0x7FF, "extended": False},
                    {"can_id": 0x101, "can_mask": 0x7FF, "extended": False},
                    {"can_id": 0x102, "can_mask": 0x7FF, "extended": False}]
    )


# Port série et Baudrate du lidar
PORT_NAME = '/dev/ttyUSB0'
PORT_NAME = 'COM14'
BAUDRATE = 1000000

# Création de l'objet Lidar, et de la Pile pile_points
lidar = None
pile_points = queue.Queue(maxsize=10000)  # ← RÉDUIT de 1000 à 100

LARGEUR_ROBOT = 250
LONGUEUR_ROBOT = 130
# Coordonnées et angle de notre robot
x_robot = int(2400+LARGEUR_ROBOT/2)
y_robot = int(1550+LONGUEUR_ROBOT/2+100)
"""x_robot = 280-15
y_robot = 220"""
angle_robot = -90

# Coordonnées, angle et vitesse du robot ennemi
x_ennemi = 1400
y_ennemi = 500
angle_ennemi = 0
v_ennemi = 0

# Objets et variables pour la fenêtre graphique
fig = None 
ax = None
robot_plot = None
ennemi_plot = None
scat = None

################## Fonction  ###########################################
def calcul_points(stop_event):
    """
    Argument : flag "stop_event"
    Modification : variables globales "pile_points"

    Utilisation :
    Création de l'objet "lidar" avec PyRPlidar
    Calcul de l'angle total et des coordonnées des points
    Saturation des valeurs pour les limites de l'aire de jeu
    
    Version OPTIMISÉE avec PyRPlidar (mode Express DenseBoost) :
    - Scan Express haute performance
    - Sous-échantillonnage pour réduire le retard
    - Vidage automatique de la queue si pleine
    """
    
    from pyrplidar import PyRPlidar
    import math
    import time
    import queue
    
    lidar = None
    
    global x_robot, y_robot, angle_robot, x_ennemi, y_ennemi
    x_point = 0
    y_point = 0
    
    # Statistiques de monitoring
    total_measurements = 0
    valid_points = 0
    sent_points = 0  # Points effectivement envoyés à l'affichage
    dropped_points = 0  # Points abandonnés (queue pleine)
    error_count = 0
    tour_count = 0
    
    # OPTIMISATION 1 : Sous-échantillonnage
    decimation_counter = 0
    DECIMATION_FACTOR = 3  # Garder 1 point sur 3 pour réduire la charge
    
    try:
        # Initialisation du Lidar
        lidar = PyRPlidar()
        lidar.connect(port="COM14", baudrate=1000000, timeout=3)
        print("✓ Lidar S2 connecté sur COM14 @ 1 Mbaud")
        
        # Configuration du moteur
        lidar.set_motor_pwm(660)
        time.sleep(3)  # Attendre stabilisation du moteur
        print("✓ Moteur démarré (PWM=660)")
        
        # Démarrage du scan Express en mode DenseBoost (mode 1)
        print("✓ Démarrage du scan Express (mode DenseBoost)...")
        scan_gen = lidar.start_scan_express(mode=1)
        generator = scan_gen()
        print("✓ Acquisition démarrée\n")
        
        # Boucle principale d'acquisition
        while not stop_event.is_set():
            
            try:
                # Récupérer la prochaine mesure depuis le générateur
                measurement = next(generator)
                
                # Comptabiliser les mesures
                total_measurements += 1
                
                # Détection de nouveau tour
                if hasattr(measurement, 'start_flag') and measurement.start_flag:
                    tour_count += 1
                
                # Extraction des données
                angle_point = measurement.angle  # Angle en degrés (0-360°)
                distance = measurement.distance / 1000.0  # Conversion mm → m
                quality = measurement.quality
                
                # Filtrer les mesures de mauvaise qualité ou distance nulle
                if distance < 0.01 or quality < 10:  # Distance < 10mm ou qualité trop faible
                    continue
                
                # Récupération de la position et orientation du robot
                x_r = x_robot
                y_r = y_robot
                angle_r = angle_robot
                
                # Calcul de l'angle total (correction d'orientation)
                phi = math.radians(angle_point)
                angle_total = phi - math.radians(angle_r) - math.radians(-2)
                
                # Calcul des coordonnées du point dans le repère global
                x_point = x_r + distance * math.cos(angle_total)
                y_point = y_r - distance * math.sin(angle_total)
                
                # Saturation dans le repère (0 ≤ x ≤ 3000, 0 ≤ y ≤ 2000)
                x_point = max(0, min(3000, int(x_point)))
                y_point = max(0, min(2000, int(y_point)))
                
                # Distance entre le robot et le point détecté
                distance_robot_point = math.sqrt((x_r - x_point)**2 + (y_r - y_point)**2)
                
                # Filtrage des points valides
                if (10 <= x_point <= 3000-10 and 
                    10 <= y_point <= 2000-10 and 
                    distance_robot_point > 50):
                    
                    valid_points += 1
                    
                    # OPTIMISATION 2 : Sous-échantillonnage (garder 1 point sur N)
                    decimation_counter += 1
                    if decimation_counter >= DECIMATION_FACTOR:
                        decimation_counter = 0
                        
                        # OPTIMISATION 3 : Vidage si queue pleine (éviter le blocage)
                        if pile_points.full():
                            # Vider la moitié de la queue pour faire de la place
                            dropped_in_batch = 0
                            while not pile_points.empty() and dropped_in_batch < 50:
                                try:
                                    pile_points.get_nowait()
                                    dropped_points += 1
                                    dropped_in_batch += 1
                                except queue.Empty:
                                    break
                        
                        # Tenter d'ajouter le point (non-bloquant)
                        try:
                            pile_points.put_nowait((x_point, y_point))
                            sent_points += 1
                        except queue.Full:
                            dropped_points += 1
                    
                    # Affichage périodique des statistiques
                    if valid_points % 5000 == 0:
                        queue_size = pile_points.qsize()
                        print(f"📊 Tours: {tour_count} | Valides: {valid_points} | "
                              f"Envoyés: {sent_points} | Perdus: {dropped_points} | "
                              f"Queue: {queue_size}/100")
            
            except StopIteration:
                print("⚠ Fin du générateur de scan (inattendu)")
                break
            
            except Exception as e:
                error_count += 1
                if error_count % 100 == 0:
                    print(f"⚠ Erreurs ponctuelles: {error_count}")
                if error_count > 1000:
                    print(f"✗ Trop d'erreurs consécutives, arrêt du thread")
                    break
                continue
                  
    except KeyboardInterrupt:
        print("\n⚠ Arrêt du Lidar demandé par l'utilisateur")
    
    except Exception as e:
        print(f"✗ Erreur dans le thread Lidar: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Nettoyage propre du Lidar
        print("\n🔄 Arrêt du Lidar...")
        if lidar is not None:
            try:
                lidar.stop()
                time.sleep(0.2)
                lidar.disconnect()
                print("✓ Lidar arrêté proprement")
            except Exception as e:
                print(f"⚠ Erreur lors de l'arrêt du Lidar: {e}")
        
        # Affichage des statistiques finales
        print(f"\n📊 Statistiques finales:")
        print(f"   Tours complets     : {tour_count}")
        print(f"   Mesures totales    : {total_measurements}")
        print(f"   Points valides     : {valid_points}")
        print(f"   Points envoyés     : {sent_points}")
        print(f"   Points perdus      : {dropped_points}")
        if valid_points > 0:
            print(f"   Taux envoi/valide  : {100*sent_points//valid_points}%")
        if total_measurements > 0:
            print(f"   Taux de validité   : {100*valid_points//total_measurements}%")
        print(f"   Erreurs ponctuelles: {error_count}")


# ============================================================================
# FONCTIONS UTILITAIRES
# ============================================================================

def find_sync_pattern(serial_port, timeout=2.0):
    """Recherche le pattern de synchronisation dans le flux de données"""
    start_time = time.time()
    buffer = bytearray()
    
    while time.time() - start_time < timeout:
        byte = serial_port.read(1)
        if len(byte) == 0:
            continue
        
        buffer.append(byte[0])
        
        if len(buffer) > 10:
            buffer.pop(0)
        
        if len(buffer) >= 5:
            potential_measurement = bytes(buffer[-5:])
            
            check_bit = potential_measurement[1] & 0b1
            if check_bit != 1:
                continue
            
            new_scan = bool(potential_measurement[0] & 0b1)
            inversed = bool((potential_measurement[0] >> 1) & 0b1)
            
            if new_scan != inversed:
                angle = ((potential_measurement[1] >> 1) + (potential_measurement[2] << 7)) / 64.0
                if 0 <= angle < 360:
                    return True
    
    return False


def decode_measurement_safe(raw_data):
    """Décode un paquet avec validation stricte"""
    if len(raw_data) != 5:
        return None
    
    check_bit = raw_data[1] & 0b1
    if check_bit != 1:
        return None
    
    new_scan = bool(raw_data[0] & 0b1)
    inversed_new_scan = bool((raw_data[0] >> 1) & 0b1)
    
    if new_scan == inversed_new_scan:
        return None
    
    quality = raw_data[0] >> 2
    angle = ((raw_data[1] >> 1) + (raw_data[2] << 7)) / 64.0
    distance = (raw_data[3] + (raw_data[4] << 8)) / 4.0
    
    if angle < 0 or angle >= 360:
        return None
    
    if distance < 0 or distance > 20000:
        return None
    
    return {
        'new_scan': new_scan,
        'quality': quality,
        'angle': angle,
        'distance': distance
    }


def affichage(stop_event):
    """Affichage graphique OPTIMISÉ avec batch processing"""
    global fig, ax, scat, robot_plot,ennemi_plot, x_robot, y_robot, x_ennemi, y_ennemi
    buffer_points = deque(maxlen=10)  # ← AUGMENTÉ de 300 à 1000 pour plus d'historique

    # Initialisation de la figure
    fig, ax = plt.subplots()
    plt.ion()
    plt.show()
    robot_plot = ax.scatter([x_robot], [y_robot], s=50, c='green', marker='o')
    #ennemi_plot = ax.scatter([x_ennemi], [y_ennemi], s=50, c='red', marker='x')
    scat = ax.scatter([], [], s=5, c='blue', alpha=0.5)
    ax.set_xlim(0, 3000)
    ax.set_ylim(0, 2000)
    ax.set_aspect('equal')
    ax.set_title("Lidar - Affichage temps réel")
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")

    frame_count = 0
    last_fps_time = time.time()

    while not stop_event.is_set():
        # OPTIMISATION : Vider toute la queue d'un coup (batch processing)
        points_frais = []
        max_batch = 200  # Limiter le nombre de points par frame
        count = 0
        
        while not pile_points.empty() and count < max_batch:
            try:
                points_frais.append(pile_points.get_nowait())
                count += 1
            except queue.Empty:
                break
        
        if points_frais:
            buffer_points.extend(points_frais)
            
            # Mise à jour de l'affichage
            if buffer_points:
                xs, ys = zip(*buffer_points)
                scat.set_offsets(np.c_[xs, ys])
            
            robot_plot.set_offsets([[x_robot, y_robot]])
            #ennemi_plot.set_offsets([[x_ennemi, y_ennemi]])
            
            # Calcul et affichage du FPS
            frame_count += 1
            if frame_count % 30 == 0:
                current_time = time.time()
                fps = 30 / (current_time - last_fps_time)
                ax.set_title(f"Lidar - Affichage temps réel ({fps:.1f} FPS)")
                last_fps_time = current_time
            
            fig.canvas.draw()
            fig.canvas.flush_events()
        
        # OPTIMISATION : Sleep plus court pour réduire la latence
        time.sleep(0.005)  # ← RÉDUIT de 0.01s à 0.005s (200 FPS max au lieu de 100)


def CAN_Odometrie(stop_event):
    """Réception CAN pour mettre à jour x_robot, y_robot et angle_robot"""
    global x_robot, y_robot, angle_robot

    while not stop_event.is_set():
        msg = bus.recv(0.01)
        if msg is None:
            continue

        if msg.arbitration_id not in (0x100, 0x101, 0x102):
            continue

        if msg.arbitration_id == 0x100:
            x_robot = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x101:
            y_robot = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x102:
            angle_robot = struct.unpack('f', bytes(msg.data))[0]

########################################################################

################## Lancement du programme principal ####################

if __name__ == '__main__':

    stop_event = threading.Event()

    # Thread Lidar
    tache_lidar = threading.Thread(target=calcul_points, args=(stop_event,), daemon=False)
    # Thread affichage
    tache_affichage = threading.Thread(target=affichage, args=(stop_event,), daemon=False)
    if Can:
        tache_odometrie = threading.Thread(target=CAN_Odometrie, args=(stop_event,), daemon=True)

    tache_lidar.start()
    tache_affichage.start()
    if Can:
        tache_odometrie.start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur.")
        stop_event.set()
        tache_lidar.join()
        tache_affichage.join()
        if Can:
            tache_odometrie.join()
        print("Programme terminé proprement.")

########################################################################