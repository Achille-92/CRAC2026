################## Librairies ##########################################
import matplotlib
matplotlib.use('Qt5Agg')
from rplidar import RPLidar
import math
import numpy as np
import random
import threading, queue
from collections import deque
import time
import os
import can
import struct
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from functools import partial
from scipy.ndimage import binary_dilation
from mouvement import verif_et_ajoute_contournement
from affichage import init_affichage, bring_to_front,afficher_obstacles,afficher_zone_securite_ennemi, mettre_a_jour_zone_ennemi,afficher_batteries
from calcul_mouv import euclidienne,astar_safe,simplify_path_safe,smooth_path_safe,creer_grille_avec_ennemi,calculer_angle_vers_point,initialiser_grille
from fonction import creer_obstacles,calculer_pourcentage_batteries,gerer_basculement_batteries

########################################################################
# Somme pondérées : robot proche/mouvement --> Malus
# config CAN

Liste_ID = [0x01,0x100, 0x101, 0x102,0x103,0x104,0x105,0x106,0x107,0x108,0x109,0x110,0x111,0x200,0x201,0x202]
Filtre = [{"can_id": Id, "can_mask": 0x7FF, "extended": False} for Id in Liste_ID]
"""os.system('sudo ip link set can0 type can bitrate 500000')
os.system('sudo ifconfig can0 up')
bus = can.interface.Bus(
    channel='can0',
    bustype='socketcan',
    bitrate=500000,
    can_filters=Filtre)"""

dico_envoi = {}
for ID in Liste_ID:
    dico_envoi[ID]= 0
print(dico_envoi)

Liste_actions = []
Liste_actions.append(["Consigne",2750,1800])
Liste_actions.append(["Tourner",90])
Liste_actions.append(["Consigne",1500,1000])
#Liste_actions.append(["Attraper"])

ordre_receive = 0
step = 0

# Port série et Baudrate du lidar
PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 256000
# Création de l'objet Lidar, et de la Pile pile_points
lidar = None
pile_calcul = queue.Queue(maxsize=500)

# Piste
x_piste = 3000
y_piste = 2000
marge_bordurepiste_x = 120 #mm
marge_bordurepiste_y = 80 #mm
case_mm = 10
width = x_piste//case_mm
height = y_piste//case_mm

# Coordonnées et angle de notre robot
x_robot_depart = 145
y_robot_depart = 120
angle_robot_depart = 90

x_robot_actuel = 145
y_robot_actuel = 120
angle_robot_actuel = 0

x_robot_voulu = 2850
y_robot_voulu = 1850
angle_robot_voulu = 0
############

# Ennemi
x_ennemi = 1900
y_ennemi = 1000
x_ennemi_old = 1900
y_ennemi_old = 1000
########

# Perimètre de sécurité
r_robot = 150
r_ennemi = 150
marge_min = 100

# R_securite : Pour l'évitement dynamique de l'ennemi (robot + ennemi + marge)
R_securite = r_robot + r_ennemi + marge_min  # = 400mm

# 🔧 RÉGLAGE : Marge pour la zone de sécurité VISUELLE de l'ennemi (30-100mm)
marge_securite_ennemi = 50  # Marge pour l'affichage de la zone ennemi


# rayon_total_case : Pour les zones interdites STATIQUES (robot + marge réduite)
# 🔧 RÉGLAGE : Ajustez marge_obstacles_mm selon vos besoins (30-100mm)
marge_obstacles_mm = 50  # Marge réduite pour les obstacles fixes
rayon_total_case = (r_robot + marge_obstacles_mm) // case_mm  # = 20 cases = 200mm

# === CRÉATION DES OBSTACLES avec la fonction ===
grid, grid_expanded, obstacle_array, expanded_array = creer_obstacles(
    x_piste,
    y_piste,
    r_robot,
    marge_obstacles_mm,
    case_mm
)


# Gestion des Batteries
Batteries = [[12,14,14,100],[12,14,14,100],[12,14,14,100]] # Vref-, Vref+, Vactuel, % de charge
U_last = [0,0,0]
Ordre_Batteries = [1,0,0]


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

# Objets et variables pour la fenêtre graphique
fig = None 
ax = None
robot_plot = None
scat = None

# Objets pour la zone de sécurité dynamique de l'ennemi
zone_ennemi_scatter = None
cercle_ennemi_patch = None

# Expansion des obstacles
from scipy.ndimage import binary_dilation
structure = np.zeros((2*rayon_total_case+1, 2*rayon_total_case+1))
y_grid, x_grid = np.ogrid[-rayon_total_case:rayon_total_case+1, -rayon_total_case:rayon_total_case+1]
mask = x_grid*x_grid + y_grid*y_grid <= rayon_total_case*rayon_total_case
structure[mask] = 1
grid_expanded = binary_dilation(grid, structure=structure)

# NOUVEAU : Carte de distance aux obstacles (pour A* pondéré)
from scipy.ndimage import distance_transform_edt
distance_map = distance_transform_edt(~grid_expanded)

action_en_cours = None
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
                x_point = max(0, min(x_piste, int(x_point)))
                y_point = max(0, min(y_piste, int(y_point)))

                if marge_bordurepiste_x <= x_point <= x_piste-marge_bordurepiste_x and marge_bordurepiste_y <= y_point <= y_piste-marge_bordurepiste_y:                 # Si ce ne sont pas les murs, on ajoute le point dans la pile sous forme de tuple (x,y)
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

        elif msg.arbitration_id == 0x110:
            Batteries[2][1] = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x111:
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

def arret_programme(event, stop_event=None):
    print("Bouton STOP pressé — arrêt demandé.")
    if stop_event is not None:
        stop_event.set()
        bring_to_front(fig)

def on_click(event):
    global x_robot_voulu, y_robot_voulu
    if event.inaxes == ax:  # clic dans la zone du graphique
        x_robot_voulu = event.xdata
        y_robot_voulu = event.ydata
        print(f"Clic souris détecté : X_voulu = {x_robot_voulu:.1f}, Y_voulu = {y_robot_voulu:.1f}")
        point_voulu_plot.set_data([round(x_robot_voulu,0)], [round(y_robot_voulu,0)])
        plt.draw()
        bring_to_front(fig)

########################################################################

################## Lancement du programme principal ####################

if __name__ == '__main__':

    stop_event = threading.Event()

    buffer_points = deque(maxlen=50)

    fig, ax, robot_plot, ennemi_plot, consigne_plot, scat, robot_info_text,ax_button,bouton_stop,point_voulu_plot,x_voulu_text,y_voulu_text,A_voulu_text, robot_angle_line,robot_angle_voulu_line = init_affichage()
    cid = fig.canvas.mpl_connect('button_press_event', on_click) # Choix des coordonnées voulues avec la souris
    bouton_stop.on_clicked(partial(arret_programme, stop_event=stop_event))
    
    obstacle_scatter, expanded_scatter = afficher_obstacles(ax,obstacle_array,expanded_array,case_mm=case_mm,show_expanded=True,show_obstacles=False)
    zone_ennemi_scatter, cercle_ennemi_patch = afficher_zone_securite_ennemi(ax,x_ennemi, y_ennemi,r_robot, r_ennemi,marge_securite_ennemi,case_mm,x_piste,y_piste,show_zone=True,show_cercle=False)
    initialiser_grille(grid_expanded, distance_map, width, height, case_mm)
    # création du trait, initialement à la position du robot
    ax.add_line(robot_angle_line)
    ax.add_line(robot_angle_voulu_line)

    """tache_lidar = threading.Thread(target=calcul_points, args=(stop_event,), daemon=False)
    tache_calcul = threading.Thread(target=calcul_ennemi, args=(stop_event,), daemon=False)
    tache_LectureCAN = threading.Thread(target=LectureCAN, args=(stop_event,), daemon=True)
    
    tache_lidar.start()
    tache_calcul.start()
    tache_LectureCAN.start()
    """
    try:
        dico_envoi[0x200]=x_robot_depart
        dico_envoi[0x201]=y_robot_depart
        dico_envoi[0x202]=angle_robot_depart

        """bus.send(can.Message(arbitration_id=0x200, data=struct.pack('<f',dico_envoi[0x200]), is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x201, data=struct.pack('<f',dico_envoi[0x201]), is_extended_id=False))
        bus.send(can.Message(arbitration_id=0x202, data=struct.pack('<f',dico_envoi[0x202]), is_extended_id=False))"""
        # === CALCUL A* DÉPLACÉ DANS LA BOUCLE ===
        # Initialisation des variables de trajectoire
        path_plot = None
        path_simplified_plot = None
        path_smooth_plot = None

        while (not stop_event.is_set() and Batteries[2][3] > 5 and len(Liste_actions)!=0): # Tant que le Flag de Thread n'est pas levé et que les batteries sont suffisamment chargées
        #while (not stop_event.is_set() and Batteries[2][3] > 5): # Tant que le Flag de Thread n'est pas levé et que les batteries sont suffisamment chargées
            dico_envoi[0x01]=1
            ordre_receive = 0
            temps = 0
            verif = False
            x_ennemi +=20
            y_ennemi -= 10
            angle_ennemi = np.degrees(math.atan2(y_ennemi-y_ennemi_old,x_ennemi-x_ennemi_old))
            if type(Liste_actions[0]) == list and len(Liste_actions[0])==3: # Si la consigne est une coordonnée
                action_voulu = Liste_actions[0][0]
                x_robot_voulu = Liste_actions[0][1]
                y_robot_voulu = Liste_actions[0][2]
                verif = True
                mouvement = True
            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==2: # Si la consigne est une coordonnée
                action_voulu = Liste_actions[0][0]
                angle_robot_voulu = Liste_actions[0][1]
                mouvement = True

            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==1:
                print("Appeler Carte Moteur pour : " + Liste_actions[0][0])
                action_voulu = Liste_actions[0][0]
                mouvement = False
                 
                 
            print("\nAvant verif :")
            print("Action en cours : "+action_voulu)
            print(f"X_actuel = {x_robot_actuel} Y_actuel = {y_robot_actuel} Angle_actuel = {angle_robot_actuel}°")
            print(f"X_voulu = {x_robot_voulu} Y_voulu = {y_robot_voulu} Angle_voulu = {angle_robot_voulu}°")
            print(Liste_actions)
            
            # --- Vérifie et modifie la trajectoire si un ennemi bloque ---

            # === CALCUL DE LA TRAJECTOIRE A* ===
            # ✅ Calculer SEULEMENT pour vraies consignes (pas waypoints "Avancer")
            if action_voulu in ["Consigne", "Recul"] or (x_ennemi != x_ennemi_old) or (y_ennemi != y_ennemi_old):
                # Convertir les positions en cases
                
                start_case = (int(x_robot_actuel // case_mm), int(y_robot_actuel // case_mm))
                goal_case = (int(x_robot_voulu // case_mm), int(y_robot_voulu // case_mm))
                
                print(f"🗺️  Calcul trajectoire A* : {start_case} → {goal_case}")
                
                # Calculer le chemin avec A*
                print(f"🎯 Position ennemi : ({x_ennemi}, {y_ennemi})")
                grid_avec_ennemi = creer_grille_avec_ennemi(
                    grid_expanded, x_ennemi, y_ennemi, 
                    r_robot, r_ennemi, marge_min, case_mm
                )

                # ✅ Calculer avec la grille dynamique
                path = astar_safe(start_case, goal_case,2.0,grid_dynamique=grid_avec_ennemi)
                
                if path:
                    print(f"✅ Chemin trouvé : {len(path)} points")
                    
                    # Simplifier le chemin
                    path_simplified = simplify_path_safe(path, 3)
                    print(f"📉 Chemin simplifié : {len(path)} → {len(path_simplified)} points")
                    
                    # Lisser le chemin
                    x_smooth, y_smooth = smooth_path_safe(path_simplified, 2.0)
                    
                    # Calculer la longueur
                    path_length = sum(euclidienne(path[i], path[i+1]) for i in range(len(path)-1))
                    path_length_mm = path_length * case_mm
                    print(f"📏 Longueur trajectoire : {path_length_mm:.0f} mm")
                    
                    # Afficher le chemin brut (optionnel, décommenter si souhaité)
                    px, py = zip(*path)
                    path_plot, = ax.plot(px, py, 'b-', linewidth=1, alpha=0.3, label='A* brut')
                    
                    # Afficher le chemin simplifié
                    px, py = zip(*path_simplified)
                    # Convertir en millimètres pour l'affichage
                    px_mm = [x * case_mm for x in px]
                    py_mm = [y * case_mm for y in py]
                    # Insérer actions avec rotation vers chaque point
                    print(f"📍 Création de {len(px_mm)} waypoints avec rotations")
                    # ✅ Trouver l'index de la consigne actuelle
                    index_insertion = 1  # Par défaut après la 1ère action
                    
                    # Chercher la consigne actuelle dans Liste_actions
                    for idx, action in enumerate(Liste_actions):
                        if action[0] == "Consigne" and len(action) == 3:
                            if action[1] == x_robot_voulu and action[2] == y_robot_voulu:
                                index_insertion = idx + 1
                                break
                    
                    print(f"📍 Insertion waypoints à l'index {index_insertion}")
                    
                    # Effacer anciens plots juste avant création nouveaux
                    if path_plot is not None:
                        path_plot.remove()
                        path_plot = None
                    if path_simplified_plot is not None:
                        path_simplified_plot.remove()
                        path_simplified_plot = None
                    if path_smooth_plot is not None:
                        path_smooth_plot.remove()
                        path_smooth_plot = None
                    
                    for i in range(len(py_mm)-1, 0, -1):  # Parcourir à l'envers
                        x_cible = px_mm[i]
                        y_cible = py_mm[i]
                        
                        # Calculer position précédente
                        if i > 0:
                            x_precedent = px_mm[i-1]
                            y_precedent = py_mm[i-1]
                        else:
                            x_precedent = x_robot_actuel
                            y_precedent = y_robot_actuel
                        
                        # Calculer angle nécessaire pour atteindre ce point
                        angle_requis = calculer_angle_vers_point(
                            x_precedent, y_precedent, x_cible, y_cible
                        )
                        
                        # Insérer Avancer puis Tourner (ordre inversé car on insère en début)
                        # ✅ Insérer à l'index correct (pas en 0)
                        Liste_actions.insert(index_insertion, ["Avancer", x_cible, y_cible])
                        Liste_actions.insert(index_insertion, ["Tourner", round(angle_requis,3)])
                        
                        print(f"  → Waypoint {len(py_mm)-i}: Tourner {angle_requis:.1f}° puis Avancer vers ({x_cible:.0f}, {y_cible:.0f})")
                    path_simplified_plot, = ax.plot(px_mm, py_mm, 'g-', linewidth=2, 
                                                     label='Chemin A*', marker='o', markersize=4, zorder=10)
                    
                    # Afficher le chemin lissé
                    if x_smooth is not None and y_smooth is not None:
                        # Convertir en millimètres pour l'affichage
                        x_smooth_mm = [x * case_mm for x in x_smooth]
                        y_smooth_mm = [y * case_mm for y in y_smooth]
                        #path_smooth_plot, = ax.plot(x_smooth_mm, y_smooth_mm, 'orange', 
                                                     #linewidth=3, label='Trajectoire lissée', zorder=11)
                    
                else:
                    print("❌ Aucun chemin trouvé par A*")
            
            """if verif == True:
                Liste_actions = verif_et_ajoute_contournement(
                    Liste_actions,action_voulu,
                    x_robot_actuel, y_robot_actuel,angle_robot_actuel,
                    x_robot_voulu, y_robot_voulu,
                    x_ennemi, y_ennemi, 
                    r_robot, r_ennemi, marge_min,
                    x_piste, y_piste
                )
            # --- Si la liste a changé, mettre à jour le point voulu ---
            """
            if type(Liste_actions[0]) == list and len(Liste_actions[0])==3: # Si la consigne est une coordonnée
                action_voulu = Liste_actions[0][0]
                x_robot_voulu = Liste_actions[0][1]
                y_robot_voulu = Liste_actions[0][2]
                mouvement = True
            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==2: # Si la consigne est une coordonnée
                action_voulu = Liste_actions[0][0]
                angle_robot_voulu = Liste_actions[0][1]
                mouvement = True
                    

            verif = False
            
            # === DÉTECTION ENNEMI SUR TRAJECTOIRE ===
            if action_voulu in ["Avancer"] and len(Liste_actions) > 1:
                # Calculer distance à l'ennemi
                dx_ennemi = x_ennemi - x_robot_actuel
                dy_ennemi = y_ennemi - y_robot_actuel
                distance_ennemi = math.sqrt(dx_ennemi**2 + dy_ennemi**2)
                
                # Si ennemi dans zone critique ( rayon sécurité)
                if distance_ennemi < R_securite :
                    print(f"⚠️  ENNEMI DÉTECTÉ à {distance_ennemi:.0f}mm ! Recalcul trajectoire...")
                    
                    # Trouver la consigne finale
                    consigne_finale = None
                    for action in Liste_actions:
                        if action[0] == "Consigne" and len(action) == 3:
                            consigne_finale = action
                            break
                    
                    if consigne_finale:
                        # Nettoyer waypoints intermédiaires
                        Liste_actions = [act for act in Liste_actions 
                                        if act[0] not in ["Avancer", "Tourner"] or act == consigne_finale]
                        
                        # Préparer recalcul
                        x_robot_voulu = consigne_finale[1]
                        y_robot_voulu = consigne_finale[2]
                        
                        start_case = (int(x_robot_actuel // case_mm), int(y_robot_actuel // case_mm))
                        goal_case = (int(x_robot_voulu // case_mm), int(y_robot_voulu // case_mm))
                        
                        print(f"🗺️  RECALCUL A* : {start_case} → {goal_case}")
                        print(f"🎯 Position ennemi : ({x_ennemi}, {y_ennemi}) mm")
                        
                        # Créer grille avec ennemi
                        grid_avec_ennemi = creer_grille_avec_ennemi(
                            grid_expanded, x_ennemi, y_ennemi,
                            r_robot, r_ennemi, marge_min, case_mm
                        )
                        
                        # Calculer nouveau chemin
                        path = astar_safe(start_case, goal_case, safety_weight=3.0,
                                          grid_dynamique=grid_avec_ennemi)
                        
                        if path and len(path) > 1:
                            path_simplified = simplify_path_safe(path, min_clearance=3)
                            print(f"✅ NOUVELLE trajectoire : {len(path)} → {len(path_simplified)} points")
                            
                            
                            # Convertir en mm
                            px, py = zip(*path_simplified)
                            px_mm = [x * case_mm for x in px]
                            py_mm = [y * case_mm for y in py]
                            
                            # Insérer nouveaux waypoints avec rotations
                            # ✅ Effacer anciens plots
                            if path_plot is not None:
                                path_plot.remove()
                                path_plot = None
                            if path_simplified_plot is not None:
                                path_simplified_plot.remove()
                                path_simplified_plot = None
                            
                            # ✅ Trouver index consigne
                            index_insertion = 1
                            for idx, action in enumerate(Liste_actions):
                                if action[0] == "Consigne" and len(action) == 3:
                                    if action[1] == x_robot_voulu and action[2] == y_robot_voulu:
                                        index_insertion = idx + 1
                                        break
                            
                            for i in range(len(py_mm)-1, 0, -1):
                                x_cible = px_mm[i]
                                y_cible = py_mm[i]
                                
                                if i > 0:
                                    x_precedent = px_mm[i-1]
                                    y_precedent = py_mm[i-1]
                                else:
                                    x_precedent = x_robot_actuel
                                    y_precedent = y_robot_actuel
                                
                                angle_requis = calculer_angle_vers_point(
                                    x_precedent, y_precedent, x_cible, y_cible
                                )
                                
                                Liste_actions.insert(index_insertion, ["Avancer", x_cible, y_cible])
                                Liste_actions.insert(index_insertion, ["Tourner", round(angle_requis,3)])
                                print(f"  → Nouveau waypoint: Tourner {angle_requis:.1f}° puis Avancer vers ({x_cible:.0f}, {y_cible:.0f})")
                            
                            # Afficher nouveau chemin
                            path_simplified_plot, = ax.plot(px_mm, py_mm, 'g-', linewidth=2,
                                                             label='Nouveau chemin', marker='o', markersize=4, zorder=10)
                            
                            # Mettre à jour l'action courante
                            if len(Liste_actions) > 0:
                                action_voulu = Liste_actions[0][0]
                                if len(Liste_actions[0]) == 2:
                                    angle_robot_voulu = Liste_actions[0][1]
                                elif len(Liste_actions[0]) == 3:
                                    x_robot_voulu = Liste_actions[0][1]
                                    y_robot_voulu = Liste_actions[0][2]
            
            print("\nAprès verif :")
            print("Action en cours : "+action_voulu)
            print(f"X_actuel = {x_robot_actuel} Y_actuel = {y_robot_actuel} Angle_actuel = {angle_robot_actuel}°")
            print(f"X_voulu = {x_robot_voulu} Y_voulu = {y_robot_voulu} Angle_voulu = {angle_robot_voulu}°")
            print(Liste_actions)

            # Envoi des Ordres de Consigne à la Carte Moteur
            dico_envoi[0x203]=x_robot_voulu
            dico_envoi[0x204]=y_robot_voulu
            ################################################

            # Mettre à jour robot, ennemi et consigne sur affichage
            robot_plot.set_offsets([[x_robot_actuel, y_robot_actuel]])
            
            # 🔄 Mettre à jour la zone de sécurité dynamique de l'ennemi
            zone_ennemi_scatter, cercle_ennemi_patch = mettre_a_jour_zone_ennemi(
                zone_ennemi_scatter, cercle_ennemi_patch,
                x_ennemi, y_ennemi, r_robot, r_ennemi,
                marge_securite_ennemi, case_mm,
                x_piste, y_piste
            )
            ennemi_plot.set_offsets([[x_ennemi, y_ennemi]])
            if (action_voulu in ["Consigne","Avancer","Recul","Contournement"]):
                consigne_plot.set_offsets([[x_robot_voulu, y_robot_voulu]])
            else :
                consigne_plot.set_offsets([[-20, -20]])

            x0, y0 = x_robot_actuel, y_robot_actuel
            x1 = x0 + longueur_trait * math.cos(math.radians(angle_robot_actuel))
            y1 = y0 + longueur_trait * math.sin(math.radians(angle_robot_actuel))
            robot_angle_line.set_data([x0, x1], [y0, y1])
            
            if (action_voulu in ["Tourner"]):
                x0, y0 = x_robot_actuel, y_robot_actuel
                x1 = x0 + longueur_trait * math.cos(math.radians(angle_robot_voulu))
                y1 = y0 + longueur_trait * math.sin(math.radians(angle_robot_voulu))
                robot_angle_voulu_line.set_data([x0, x1], [y0, y1])
            else :
                robot_angle_voulu_line.set_data([-20, -20], [-40, -40])
            ################################################
            

            # === GESTION BATTERIES avec les fonctions ===
            # SIMULATION Perte Batterie
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
            robot_info_text.set_text(
                f"X = {x_robot_actuel:.1f} Y = {y_robot_actuel:.1f} A = {angle_robot_actuel:.1f}°"
            )
            x_voulu_text.set_text(
                f"X = {x_robot_voulu:.1f}"
            )
            y_voulu_text.set_text(
                f"Y = {y_robot_voulu:.1f}"
            )
            A_voulu_text.set_text(
                f"A = {angle_robot_voulu:.1f}°"
            )

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

            # Si robot est à la position de consigne  
            if(abs(x_robot_actuel-x_robot_voulu)<10 and abs(y_robot_actuel-y_robot_voulu)<10 and action_voulu in ["Consigne","Avancer","Recul","Contournement"]):
                print("Bonne position")
                ordre_receive = 11
            if(abs(angle_robot_actuel-angle_robot_voulu)<1) and action_voulu == "Tourner":
                print("Bon Angle")
                ordre_receive = 12

            
            """time.sleep(0.3)
            ordre_receive = 1
            """
            ordre_receive= int(input("Ordre : "))
            if (action_voulu in ["Consigne","Avancer","Recul","Contournement"] and ordre_receive == 11) or (action_voulu == "Tourner" and ordre_receive == 12) or (action_voulu == "Attraper" and ordre_receive == 21) or (action_voulu == "Relacher" and ordre_receive == 22):
            #if(ordre_receive==1):   
                print(step)
                step+=1
                Liste_actions.pop(0)
                if len(Liste_actions)!=0:
                    if type(Liste_actions[0]) == list and len(Liste_actions[0])==3:
                        x_robot_actuel = Liste_actions[0][1]
                        y_robot_actuel = Liste_actions[0][2]
                        angle_robot_actuel = angle_robot_voulu
                    elif type(Liste_actions[0]) == list and len(Liste_actions[0])==2:
                        angle_robot_actuel = Liste_actions[0][1]
                    # Retire de la liste l'action en cours
                ordre_receive = 0
                if(step == 2):
                    x_ennemi = 2600
                    y_ennemi = 800

            # MAJ de l'affichage et des Variables de Bouncing
            plt.draw()
            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.01)
            U_last = [Batteries[0][2],Batteries[1][2],Batteries[2][2]]
            x_robot_voulu_last = x_robot_voulu
            y_robot_voulu_last = y_robot_voulu
            x_ennemi_old = x_ennemi
            y_ennemi_old = y_ennemi
            print("")
            
            """for key, value in dico_envoi.items() :
                if value != 0:
                    if key in [0x01,0x300,0x301,0x302]:
                        format_value = struct.pack('<I',dico_envoi[key])
                    else :
                        format_value = struct.pack('<f',dico_envoi[key])
                    msg = can.Message(arbitration_id=key, data=format_value, is_extended_id=False)
                    bus.send(msg)
                    dico_envoi[key]=0"""

        time.sleep(2)
        stop_event.set()

    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur.")
        stop_event.set()  # signal aux threads de s'arrêter
        # Attente que chaque thread termine proprement
        """tache_lidar.join()
        tache_calcul.join()
        tache_LectureCAN.join()
        os.system("sudo ifconfig can0 down")
        """
    finally:
        print("Programme terminé proprement.")
        """tache_lidar.join()
        tache_calcul.join()
        tache_LectureCAN.join()
        etat = 2
        data_etat = struct.pack('<I',etat)
        bus.send(can.Message(arbitration_id=0x01, data=data_etat, is_extended_id=False))
        """
        plt.close(fig)


########################################################################
