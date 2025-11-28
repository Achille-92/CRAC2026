Reel = False
x_robot_depart = 275 
y_robot_depart = 1650
angle_robot_depart = -90

################## Librairies ##########################################
import matplotlib
matplotlib.use('Qt5Agg')
from rplidar import RPLidar
import math
import numpy as np
import threading, queue
from collections import deque
import time
import os
import can
import struct
import socket
import json
import matplotlib.pyplot as plt
from functools import partial
from affichage import init_affichage, bring_to_front,afficher_batteries
from fonction import calculer_pourcentage_batteries,gerer_basculement_batteries
########################################################################

# A envoyer : X_robot_actuel, Y_robot_actuel, Angle_robot_actuel, x_ennemi, y_ennemi, Batteries
# A recevoir : Liste_actions, Liste_trajectoire, Ordre_receive

# Configuration pour la réception
HOST_PC = '0.0.0.0'
PORT_RECEPTION = 5000

# Configuration pour l'envoi vers la RPI
HOST_RPI = "192.168.0.100"  # IP de la RPI4
PORT_ENVOI = 5001

# Config CAN 
Liste_ID = [0x01,0x100, 0x101, 0x102,0x103,0x104,0x105,0x106,0x107,0x108,0x109,0x10A,0x10B,0x200,0x201,0x202]
Filtre = [{"can_id": Id, "can_mask": 0x7FF, "extended": False} for Id in Liste_ID]
if Reel: 
    os.system('sudo ip link set can0 type can bitrate 500000')
    os.system('sudo ifconfig can0 up')
    bus = can.interface.Bus(
        channel='can0',
        bustype='socketcan',
        bitrate=500000,
        can_filters=Filtre)
dico_envoi = {}
for ID in Liste_ID:
    dico_envoi[ID]= 0
#################################################

Liste_actions = []
Liste_trajectoire = []

Liste_GM_libres = [1,2,3,4,5,6,7,8,9,10]
Liste_GM_occuper = []
Liste_noisettes_libres = [1,2,3,4,5,6,7,8]
Liste_noisettes_prises = []

Liste_GM_libres_precedente = Liste_GM_libres.copy()
Liste_noisettes_libres_precedente = Liste_noisettes_libres.copy()
Liste_actions_precedente = Liste_actions.copy()

# Lidar
PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 256000
lidar = None
pile_calcul = queue.Queue(maxsize=500)
####################################

# Piste
X_PISTE = 3000
Y_PISTE = 2000
MARGE_BORDUREPISTE_X = 120 # Détection Lidar
MARGE_BORDUREPISTE_Y = 80 # Détection Lidar

# Coordonnées et angle de notre robot (coordonnées initiales en haut)
x_robot_actuel = x_robot_depart
y_robot_actuel = y_robot_depart
angle_robot_actuel = angle_robot_depart

x_robot_voulu = -1
y_robot_voulu = -1
angle_robot_voulu = -181
############

# Coordonnées Ennemi
x_ennemi = 2500
y_ennemi = 1000
x_ennemi_old = x_ennemi
y_ennemi_old = y_ennemi
########

# Variables fonctionnelles des Batteries
Batteries = [[12,14,14,100],[12,14,14,100],[12,14,14,100]] # Vref-, Vref+, Vactuel, % de charge
U_last = [0,0,0]
Ordre_Batteries = [1,0,0]
############################

# Variables pour l'affichage des rectangles de batteries et des textes
battery_patches = []
battery_texts = [] 
couleurs = ['red', 'orange', 'yellow', 'lime', 'green']
seuils = [1, 20, 50, 75, 90]
largeur_rect = 50       # largeur en mm
hauteur_rect = 150      # hauteur en mm
espacement = 0         # espace entre rectangles
espacement_salves = 200 # espace entre chaque salve
y_base = 2050           # position verticale (en haut de la piste)
marge_texte = 20  # espace horizontal entre texte et rectangle
texte_offset_y = 80  # décalage vertical du texte par rapport aux rectangles
longueur_trait = 100  # longueur trait de direction
compteur_affichage = 0
FREQUENCE_AFFICHAGE = 4
##########################

# Objets et variables pour la fenêtre graphique
fig = None 
ax = None
robot_plot = None
scat = None
#############

action_en_cours = None
trajectoire_bloquee = False
ordre_receive = 0
step = 0
robot_a_objets = False
etat_BC = 0
donnees_vers_rpi = {
    "x_robot_actuel": x_robot_actuel,
    "y_robot_actuel": y_robot_actuel,
    "angle_robot_actuel": angle_robot_actuel,
    "x_ennemi": x_ennemi,
    "y_ennemi": y_ennemi,
    "Batteries": Batteries
}

################## Fonction  ###########################################
def calcul_points(stop_event):
    """
    Argument : flag "stop_event"
    Modification : variables globales "pile_points"

    Utilisation :
    Création de l'objet "lidar"
    Calcul de l'angle total et des coordonnées des points
    Saturation des valeurs pour les limites de l'aire de jeu, puis pour oublier les bords
    """

    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE)                       # connexion au Lidar
    
    print("INFO:", lidar.get_info())                                    # Affichage d'informations propres au Lidar
    print("HEALTH:", lidar.get_health())

    lidar.start_motor()                                                 # Démarrage du moteur du Lidar
    global  x_robot_actuel, y_robot_actuel, angle_robot_actuel
    x_point = 0
    y_point = 0

    try:
        # On lit les scans tant que le stop_event n’est pas activé
        for scan in lidar.iter_scans(scan_type='express', max_buf_meas=4096):
            if stop_event.is_set():   # si on demande l’arrêt → on sort
                break

            for (quality, angle_point, distance) in scan:                       # Pour chaque points dans le scan
                phi = math.radians(angle_point)                                 # On converti l'angle de la mesure en radian
                angle_total = phi - math.radians(angle_robot_actuel) - math.radians(11)    # On calcule l'angle total à partir de l'orientation du Lidar et du robot

                x_point = x_robot_actuel + distance * math.cos(angle_total)                # On calcule les coordonnées x et y du point à partir de la position et de l'orientation du robot
                y_point = y_robot_actuel - distance * math.sin(angle_total)

                # Saturation dans le repère (0 ≤ x ≤ 3000, 0 ≤ y ≤ 2000)
                x_point = max(0, min(X_PISTE, int(x_point)))
                y_point = max(0, min(Y_PISTE, int(y_point)))

                if MARGE_BORDUREPISTE_X <= x_point <= X_PISTE-MARGE_BORDUREPISTE_X and MARGE_BORDUREPISTE_Y <= y_point <= Y_PISTE-MARGE_BORDUREPISTE_Y:                 # Si ce ne sont pas les murs, on ajoute le point dans la pile sous forme de tuple (x,y)
                    pile_calcul.put((x_point, y_point))

    except Exception as e:                                                      # En cas d'exception on affiche l'erreur
        print("Erreur dans le thread Lidar:", e)
    finally:                                                                    # Et on arrête le Lidar
        # Nettoyage du Lidar
        print("Arrêt du Lidar...")
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()

def LectureCAN(stop_event):
    """
    Argument : flag "stop_event"
    Modification : variables globales x_robot, y_robot, angle_robot, Batteries

    Utilisation :
    Lis le Bus CAN
    Si l'ID du message n'est pas dans la Liste_ID, saute
    Sinon, met à jour les coordonées et angle du robot, valeurs des batteries
    """
    global x_robot_actuel, y_robot_actuel, angle_robot_actuel, Liste_ID, Batteries, dico_envoi, bus
    while not stop_event.is_set():
        msg = bus.recv(0.01)  # attend 10 ms max
        if msg is None:
            continue  # pas de message, on repart

        # Vérifie qu'on a bien reçu 4 octets avant de décoder
        if msg.arbitration_id not in Liste_ID:
            continue  # on saute les autres trames

        if msg.arbitration_id == 0x100:
            x_robot_actuel = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x101:
            y_robot_actuel = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x102:
            angle_robot_actuel = struct.unpack('f', bytes(msg.data))[0]

        # Batteries
        elif msg.arbitration_id == 0x103:
            Batteries[0][0] = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x104:
            Batteries[0][1] = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x105:
            Batteries[0][2] = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x106:
            Batteries[1][0] = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x107:
            Batteries[1][1] = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x108:
            Batteries[1][2] = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x109:
            Batteries[2][0] = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x10A:
            Batteries[2][1] = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x10B:
            Batteries[2][2] = struct.unpack('f', bytes(msg.data))[0]
        

def calcul_ennemi(stop_event):
    """
    Arguments : flag stop_event
    Modification : buffer_points, x_ennemi, y_ennemi

    Utilisation :
    Récupére le haut de la pile_calcul, puis le remet dans buffer_points
    Moyenne les coordonées des points de la pile, donne x_ennemi et y_ennemi
    """
    global x_ennemi, y_ennemi

    buffer_points = deque(maxlen=50)

    while not stop_event.is_set():
        try:
            # Récupère un point du Lidar
            p = pile_calcul.get(timeout=0.1)
            buffer_points.append(p)
        except queue.Empty:
            pass

        # Calcul barycentre, angle et vitesse
        if buffer_points:
            xs, ys = zip(*buffer_points)
            x_ennemi = np.mean(xs)
            y_ennemi = np.mean(ys)
            x_ennemi = 1500
            y_ennemi = 1000

################## Fonction ##################################################
def arret_programme(event, stop_event=None):
    print("Bouton STOP pressé — arrêt demandé.")
    if stop_event is not None:
        stop_event.set()
        bring_to_front(fig)
 
# Fonction pour recevoir des données de la RPI
def recevoir_donnees(stop_event):
    print(f"[Récepteur] Serveur en attente sur le port {PORT_RECEPTION}...")
    global Liste_actions, Liste_trajectoire, Ordre_receive
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST_PC, PORT_RECEPTION))
    server_socket.listen(1)
    
    try:
        while not stop_event.is_set():
            conn, addr = server_socket.accept()
            print(f"\n[Récepteur] --- Connexion depuis {addr} ---")
            
            # Réception des données
            data = b""
            while True:
                packet = conn.recv(1024)
                if not packet:
                    break
                data += packet
            
            conn.close()
            
            # Décodage et affichage
            try:
                donnees_robot = json.loads(data.decode())
                
                print("[Récepteur] Données reçues depuis la RPI :")
                print(json.dumps(donnees_robot, indent=4))
                
                Liste_actions = donnees_robot["Liste_actions"]
                Liste_trajectoire = donnees_robot["Liste_trajectoire"]
                Ordre_receive = donnees_robot["Ordre_receive"]
                Liste_GM_libres = donnees_robot["Liste_GM_libres"]
                Liste_GM_occuper = donnees_robot["Liste_GM_occuper"]
                Liste_noisettes_libres = donnees_robot["Liste_noisettes_libres"]
                Liste_noisettes_prises = donnees_robot["Liste_noisettes_prises"]
                
                print(f"[Récepteur] Liste_actions : {Liste_actions}")
                print(f"[Récepteur] Liste_trajectoire : {Liste_trajectoire}")
                print(f"[Récepteur] Ordre_receive : {Ordre_receive}")
                print(f"[Récepteur] Liste_GM_libres : {Liste_GM_libres}")
                print(f"[Récepteur] Liste_GM_occuper : {Liste_GM_occuper}")
                print(f"[Récepteur] Liste_noisettes_libres : {Liste_noisettes_libres}")
                print(f"[Récepteur] Liste_noisettes_prises : {Liste_noisettes_prises}")
                
            except json.JSONDecodeError:
                print("[Récepteur] Erreur : données JSON invalides")
            
    except KeyboardInterrupt:
        print("[Récepteur] Arrêt.")
    finally:
        server_socket.close()


def update_display(background):
    """Mise à jour optimisée avec blitting"""
    # Restaurer le fond
    fig.canvas.restore_region(background)
    
    # Redessiner uniquement les éléments qui changent
    ax.draw_artist(robot_plot)
    ax.draw_artist(ennemi_plot)
    ax.draw_artist(scat)
    ax.draw_artist(robot_info_text)
    ax.draw_artist(x_voulu_text)
    ax.draw_artist(y_voulu_text)
    ax.draw_artist(A_voulu_text)
    
    # Mise à jour finale (seulement la zone modifiée)
    fig.canvas.blit(ax.bbox)
    fig.canvas.flush_events()

########################################################################

################## Lancement du programme principal ####################

if __name__ == '__main__':

    stop_event = threading.Event()

    buffer_points = deque(maxlen=50)

    fig, ax, robot_plot, ennemi_plot, consigne_plot, scat, robot_info_text,ax_button,bouton_stop,point_voulu_plot,x_voulu_text,y_voulu_text,A_voulu_text, robot_angle_line,robot_angle_voulu_line,background = init_affichage()
    bouton_stop.on_clicked(partial(arret_programme, stop_event=stop_event))
    
    # création du trait, initialement à la position du robot
    ax.add_line(robot_angle_line)
    ax.add_line(robot_angle_voulu_line)
    
    thread_reception = threading.Thread(target=recevoir_donnees, args=(stop_event,),daemon=True)

    if Reel: 
        tache_lidar = threading.Thread(target=calcul_points, args=(stop_event,), daemon=False)
        tache_calcul = threading.Thread(target=calcul_ennemi, args=(stop_event,), daemon=False)
        tache_LectureCAN = threading.Thread(target=LectureCAN, args=(stop_event,), daemon=True)
        
        tache_lidar.start()
        tache_calcul.start()
        tache_LectureCAN.start()
    
    thread_reception.start()
    
    try:
        dico_envoi[0x200]=x_robot_depart
        dico_envoi[0x201]=y_robot_depart
        dico_envoi[0x202]=angle_robot_depart
        if Reel: 
            bus.send(can.Message(arbitration_id=0x200, data=struct.pack('<f',dico_envoi[0x200]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x201, data=struct.pack('<f',dico_envoi[0x201]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x202, data=struct.pack('<f',dico_envoi[0x202]), is_extended_id=False))
        
        while(len(Liste_actions) == 0):
            print("Attente de modification de Liste_actions")
        while (not stop_event.is_set() and Batteries[2][3] > 5): # Tant que le Flag de Thread n'est pas levé et que les batteries sont suffisamment chargées
            dico_envoi[0x01]=1
            temps = 0
            step +=1

            # Lire l'action courante
            if type(Liste_actions[0]) == list and len(Liste_actions[0])==3:
                action_voulu = Liste_actions[0][0]
                x_robot_voulu = Liste_actions[0][1]
                y_robot_voulu = Liste_actions[0][2]
                mouvement = True
            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==2:
                action_voulu = Liste_actions[0][0]
                angle_robot_voulu = round(Liste_actions[0][1],0)
                mouvement = True
            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==1:
                print("Appeler Carte Actionneur pour : " + Liste_actions[0][0])
                action_voulu = Liste_actions[0][0]
                mouvement = False
            
            if not Reel: 
                x_ennemi += 3
                y_ennemi -= 2

            print(step)
            print("Action en cours : "+action_voulu)
            print(f"X_actuel = {x_robot_actuel} Y_actuel = {y_robot_actuel} Angle_actuel = {angle_robot_actuel}°")
            print(f"X_voulu = {x_robot_voulu} Y_voulu = {y_robot_voulu} Angle_voulu = {angle_robot_voulu}°")
            print(Liste_actions)

            
            ##### Simulation Mouvement Robot 
            if not Reel: 
                if(action_voulu in ["Rotation"]):
                    if(angle_robot_actuel > angle_robot_voulu):
                        angle_robot_actuel -= 5
                    elif(angle_robot_actuel < angle_robot_voulu):
                        angle_robot_actuel += 5
                if(action_voulu in ["Consigne","Avancer"]):
                    angle_robot_consigne = math.atan2(y_robot_voulu-y_robot_actuel,x_robot_voulu-x_robot_actuel)
                    x_robot_actuel += round(5*np.cos(angle_robot_consigne),0)
                    y_robot_actuel += round(5*np.sin(angle_robot_consigne),0)

            # Envoi des Ordres de Consigne à la Carte Moteur
            dico_envoi[0x205]=angle_robot_voulu

            # Mettre à jour robot, ennemi et consigne sur affichage
            robot_plot.set_offsets([[x_robot_actuel, y_robot_actuel]])
            ennemi_plot.set_offsets([[x_ennemi, y_ennemi]])

            if (action_voulu in ["Consigne","Avancer"]):
                consigne_plot.set_offsets([[x_robot_voulu, y_robot_voulu]])
            else :
                consigne_plot.set_offsets([[-20, -20]])

            x0, y0 = x_robot_actuel, y_robot_actuel
            x1 = x0 + longueur_trait * math.cos(math.radians(angle_robot_actuel))
            y1 = y0 + longueur_trait * math.sin(math.radians(angle_robot_actuel))
            robot_angle_line.set_data([x0, x1], [y0, y1])
            
            if (action_voulu in ["Rotation"]):
                x0, y0 = x_robot_actuel, y_robot_actuel
                x1 = x0 + longueur_trait * math.cos(math.radians(angle_robot_voulu))
                y1 = y0 + longueur_trait * math.sin(math.radians(angle_robot_voulu))
                robot_angle_voulu_line.set_data([x0, x1], [y0, y1])
            else :
                robot_angle_voulu_line.set_data([-20, -20], [-40, -40])
            ################################################
            

            # === GESTION BATTERIES avec les fonctions ===
            # SIMULATION Perte Batterie
            if not Reel: 
                if Ordre_Batteries == [1,0,0]:
                    Batteries[0][2] -=0.001
                elif Ordre_Batteries == [0,1,0]:
                    Batteries[1][2] -=0.001
                elif Ordre_Batteries == [0,0,1]:
                    Batteries[2][2] -=0.001
                
            Batteries = calculer_pourcentage_batteries(Batteries, U_last)
            Ordre_Batteries = gerer_basculement_batteries(Batteries, U_last, Ordre_Batteries)
            battery_patches, battery_texts = afficher_batteries(ax, Batteries, Ordre_Batteries,battery_patches, battery_texts,couleurs, seuils,largeur_rect, hauteur_rect, espacement, espacement_salves,y_base, texte_offset_y)
            
            ### Envoi des Ordres à la Carte Alim
            if(Ordre_Batteries[0]==1):
                dico_envoi[0x300]=1
            else :
                dico_envoi[0x300]=0
            if(Ordre_Batteries[1]==1):
                dico_envoi[0x301]=1
            else :
                dico_envoi[0x301]=0
            if(Ordre_Batteries[2]==1):
                dico_envoi[0x302]=1
            else :
                dico_envoi[0x302]=0
            #######################################
            
            # Affichage texte Coordonées
            robot_info_text.set_text(f"X = {x_robot_actuel:.1f} Y = {y_robot_actuel:.1f} A = {angle_robot_actuel:.1f}°")
            x_voulu_text.set_text(f"X = {x_robot_voulu:.1f}")
            y_voulu_text.set_text(f"Y = {y_robot_voulu:.1f}")
            A_voulu_text.set_text(f"A = {angle_robot_voulu:.1f}°")

            if(abs(x_robot_actuel-x_robot_voulu)>10):
                x_voulu_text.set_color('black')
            else:
                x_voulu_text.set_color('green')
            if(abs(y_robot_actuel-y_robot_voulu)>10):
                y_voulu_text.set_color('black')
            else:
                y_voulu_text.set_color('green')
            if(abs(angle_robot_actuel-angle_robot_voulu)>1):
                A_voulu_text.set_color('black')
            else:
                A_voulu_text.set_color('green')
            ##################################################  
            #              
            # Si robot est à la position de consigne  
            if Reel: 
                if(abs(x_robot_actuel-x_robot_voulu)<10 and abs(y_robot_actuel-y_robot_voulu)<10 and action_voulu in ["Consigne","Avancer","Recul","Contournement"]):
                    print("Bonne position")
                    ordre_receive = 11
                if(abs(angle_robot_actuel-angle_robot_voulu)<1) and action_voulu in ["Rotation","Tourner"]:
                    print("Bon Angle")
                    ordre_receive = 12
            else :
                if(abs(x_robot_actuel-x_robot_voulu)<16 and abs(y_robot_actuel-y_robot_voulu)<16 and action_voulu in ["Consigne","Avancer","Recul","Contournement"]):
                    print("Bonne position")
                    ordre_receive = 11
                if(abs(angle_robot_actuel-angle_robot_voulu)<5) and action_voulu in ["Rotation","Tourner"]:
                    print("Bon Angle")
                    ordre_receive = 12

            if Reel :
                for key, value in dico_envoi.items() :
                    if value != 0:
                        if key in [0x01,0x207,0x300,0x301,0x302]:
                            format_value = struct.pack('<I',dico_envoi[key])
                        else :
                            format_value = struct.pack('<f',dico_envoi[key])
                        msg = can.Message(arbitration_id=key, data=format_value, is_extended_id=False)
                        bus.send(msg)
                        dico_envoi[key]=0
                        time.sleep(0.0005)

            # MAJ de l'affichage et des Variables de Bouncing
            compteur_affichage += 1
            if compteur_affichage >= FREQUENCE_AFFICHAGE:
                update_display(background)
                compteur_affichage = 0
            fig.canvas.flush_events()
            U_last = [Batteries[0][2],Batteries[1][2],Batteries[2][2]]
            x_robot_voulu_last = x_robot_voulu
            y_robot_voulu_last = y_robot_voulu
            x_ennemi_old = x_ennemi
            y_ennemi_old = y_ennemi
            Liste_GM_libres_precedente = Liste_GM_libres.copy()
            Liste_noisettes_libres_precedente = Liste_noisettes_libres.copy()
            Liste_actions_precedente = Liste_actions.copy()
            
            time.sleep(0.001)

            donnees_vers_rpi = {
                "x_robot_actuel": x_robot_actuel,
                "y_robot_actuel": y_robot_actuel,
                "angle_robot_actuel": angle_robot_actuel,
                "x_ennemi": x_ennemi,
                "y_ennemi": y_ennemi,
                "Batteries": Batteries
            }
            message = json.dumps(donnees_vers_rpi)
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((HOST_RPI, PORT_ENVOI))
            client_socket.sendall(message.encode())
            client_socket.close()
            time.sleep(0.01)
            
        time.sleep(2)
        stop_event.set()

    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur.")
        stop_event.set()  # signal aux threads de s'arrêter
        # Attente que chaque thread termine proprement
        if Reel :
            tache_lidar.join()
            tache_calcul.join()
            tache_LectureCAN.join()
            os.system("sudo ifconfig can0 down")
        
    except Exception as e:
        print(e)
        thread_reception.join()
        
    except Exception as e:
        print(e)
    finally:
        print("Programme terminé proprement.")
        if Reel :
            tache_lidar.join()
            tache_calcul.join()
            tache_LectureCAN.join()
            etat = 2
            data_etat = struct.pack('<I',etat)
            bus.send(can.Message(arbitration_id=0x01, data=data_etat, is_extended_id=False))
        
        plt.close(fig)

########################################################################
