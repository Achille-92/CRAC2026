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
from matplotlib.widgets import Button
from scipy.ndimage import binary_dilation
from affichage import init_affichage, bring_to_front,afficher_obstacles,afficher_zone_securite_ennemi, mettre_a_jour_zone_ennemi,afficher_batteries
from calcul_mouv import euclidienne,astar_safe,simplify_path_safe,smooth_path_safe,creer_grille_avec_ennemi,calculer_angle_vers_point,initialiser_grille
from fonction import Obstacles,calculer_pourcentage_batteries,gerer_basculement_batteries
from gestion_zones_dynamiques import mettre_a_jour_zones_dynamiques, regenerer_grilles_combinees
########################################################################
# Somme pondérées : robot proche/mouvement --> Malus
# config CAN
Reel = False
Liste_ID = [0x01,0x100, 0x101, 0x102,0x103,0x104,0x105,0x106,0x107,0x108,0x109,0x110,0x111,0x200,0x201,0x202]
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
print(dico_envoi)

Liste_actions = []
"""Liste_actions.append(["Consigne",2750,1800])
Liste_actions.append(["Rotation",90])
Liste_actions.append(["Consigne",200,1700])
Liste_actions.append(["Attraper"])
Liste_actions.append(["Consigne",1200,500])
Liste_actions.append(["Relacher"])"""
Liste_trajectoire = []

Liste_GM_libres = [1,2,3,4,5,6,7,8,9,10]
Liste_GM_occuper = []
Liste_noisettes_libres = [1,2,3,4,5,6,7,8]
Liste_noisettes_prises = []

Liste_zones_recup_noisettes_xy = [(400,1200),(400,400),(2600,1200),(2600,400),(1150,1025),(1850,1025),(1100,400),(1900,400)]
Liste_zones_recup_noisettes_angle = [180,180,0,0,-90,-90,-90,-90]
Liste_zones_gm_xy = [(1250,1200),(1750,1200),(350,800),(800,1050),(1500,1050),(2200,1050),(2650,800),(700,350),(1500,350),(2300,350)]
Liste_zones_gm_angle = [90,90,180,-90,-90,-90,0,-90,-90,-90]

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
x_robot_depart = 350
y_robot_depart = 1800
angle_robot_depart = -90

x_robot_actuel = x_robot_depart
y_robot_actuel = y_robot_depart
angle_robot_actuel = angle_robot_depart

x_robot_voulu = -1
y_robot_voulu = -1
angle_robot_voulu = -181
############

# Ennemi
x_ennemi = 1900
y_ennemi = 1000
x_ennemi_old = x_ennemi
y_ennemi_old = y_ennemi
########

# Perimètre de sécurité
r_robot = 130
r_ennemi = 150
marge_min = 100

# R_securite : Pour l'évitement dynamique de l'ennemi (robot + ennemi + marge)
R_securite = r_robot + r_ennemi + marge_min  # = 400mm
EPS_EQ = 5
# 🔧 RÉGLAGE : Marge pour la zone de sécurité VISUELLE de l'ennemi (30-100mm)
marge_securite_ennemi = 5  # Marge pour l'affichage de la zone ennemi

# rayon_total_case : Pour les zones interdites STATIQUES (robot + marge réduite)
# 🔧 RÉGLAGE : Ajustez marge_obstacles_mm selon vos besoins (30-100mm)
marge_obstacles_mm = 0  # Marge standard pour les obstacles fixes
marge_noisettes_mm = 0  # Marge réduite pour les Noisettes (plus précis)
rayon_total_case = (r_robot + marge_obstacles_mm) // case_mm  # = 20 cases = 200mm

# === CRÉATION DES OBSTACLES avec la classe Obstacles ===
# Créer le gestionnaire d'obstacles
obs_manager = Obstacles(
    x_piste,
    y_piste,
    r_robot,
    marge_obstacles_mm,
    case_mm
)

# Créer un gestionnaire séparé pour les Noisettes avec marge réduite
obs_manager_noisettes = Obstacles(
    x_piste,
    y_piste,
    r_robot,
    marge_noisettes_mm,  # Marge réduite pour les Noisettes
    case_mm
)

# Ajouter les 10 zones carrées interdites (Coupe de France 2024/2025) - INACTIVES au départ
zones_centres = [
    (1250, 1450), (1750, 1450),           # 2 zones centrales
    (100, 800), (800, 800), (1500, 800), (2200, 800), (2900, 800),  # 5 zones ligne médiane
    (700, 100), (1500, 100), (2300, 100),  # 3 zones ligne basse
]


for i, (x, y) in enumerate(zones_centres, 1):
    obs_manager.ajouter_carre(f"zone{i}", (x, y), 200, actif=False)  # INACTIVES au départ

# Ajouter les 8 zones rectangulaires "Noisettes" - ACTIVES au départ
# Format : (x_min, y_min), (x_max, y_max) où min < max
zones_noisettes = [
    [(100, 1100), (250, 1300)],    # Noisette 1 : coin gauche haut
    [(100, 300), (250, 500)],      # Noisette 2 : coin gauche milieu
    [(2750, 1100), (2900, 1300)],  # Noisette 3 : coin droit haut
    [(2750, 300), (2900, 500)],    # Noisette 4 : coin droit milieu
    [(1050, 725), (1250, 875)],    # Noisette 5 : centre gauche
    [(1750, 725), (1950, 875)],    # Noisette 6 : centre droit
    [(1000, 100), (1200, 250)],    # Noisette 7 : bas gauche
    [(1800, 100), (2000, 250)]     # Noisette 8 : bas droit
]

print("\n--- AJOUT DES NOISETTES (avec marge réduite de {}mm) ---".format(marge_noisettes_mm))
for i, coords in enumerate(zones_noisettes, 1):
    coin_bg = coords[0]  # Premier coin (bas-gauche)
    coin_hd = coords[1]  # Deuxième coin (haut-droit)
    obs_manager_noisettes.ajouter_rectangle(f"Noisette{i}", coin_bg, coin_hd, actif=True)  # ACTIVES au départ

# Ajouter la zone rectangulaire (grenier) - TOUJOURS ACTIVE
obs_manager.ajouter_rectangle("grenier", (600, 1550), (2400, 2000), actif=True)

# Afficher toutes les zones créées
obs_manager.lister()

obs_manager_noisettes.lister()

# Générer les grilles séparément
grid_zones, grid_zones_expanded, obstacle_array_zones, expanded_array_zones = obs_manager.generer_grille()
grid_noisettes, grid_noisettes_expanded, obstacle_array_noisettes, expanded_array_noisettes = obs_manager_noisettes.generer_grille()

# Combiner les deux grilles (union logique OR)
grid = np.logical_or(grid_zones, grid_noisettes).astype(int)
grid_expanded = np.logical_or(grid_zones_expanded, grid_noisettes_expanded).astype(int)

# Combiner les arrays d'obstacles pour l'affichage
obstacle_array = np.vstack([obstacle_array_zones, obstacle_array_noisettes]) if len(obstacle_array_zones) > 0 and len(obstacle_array_noisettes) > 0 else (obstacle_array_zones if len(obstacle_array_zones) > 0 else obstacle_array_noisettes)
expanded_array = np.vstack([expanded_array_zones, expanded_array_noisettes]) if len(expanded_array_zones) > 0 and len(expanded_array_noisettes) > 0 else (expanded_array_zones if len(expanded_array_zones) > 0 else expanded_array_noisettes)


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

# Variables globales pour les boutons et l'affichage des obstacles
obstacle_scatter = None
expanded_scatter = None
boutons_zones = []  # Liste pour stocker les 10 boutons des zones carrées
boutons_noisettes = []  # Liste pour stocker les 8 boutons des Noisettes

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
trajectoire_bloquee = False
ordre_receive = 0
step = 0

# ==================== PARAMÈTRES DE L'ALGORITHME ====================
# Poids de la moyenne pondérée (somme = 1.0)
W_DISTANCE = 0.4     # Importance de la proximité de l'objectif
W_SECURITE = 0.3     # Importance de l'évitement du robot ennemi
W_PRIORITE = 0.2     # Importance du type de tâche (récolte vs dépôt)
W_EFFICACITE = 0.1   # Importance de la cohérence (robot a objets → déposer)

# Distance max sur le terrain (pour normalisation)
DISTANCE_MAX_TERRAIN = 3500  # Diagonale du terrain ≈ 3605mm
robot_a_objets = False

print("\n" + "="*70)
print("🤖 PRISE DE DÉCISION INITIALE")
print("="*70)


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

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def calcul_angle_vers_point(x_actuel, y_actuel, x_suivant, y_suivant):
    """
    Calcule l'angle absolu (en degrés 0–360) à viser pour aller vers (x_suivant, y_suivant)
    depuis (x_actuel, y_actuel).
    """
    dx = x_suivant - x_actuel
    dy = y_suivant - y_actuel
    angle = (math.degrees(math.atan2(dy, dx)) + 360) % 360
    return round(angle, 2)

def actualiser_affichage_obstacles():
    """
    Fonction pour régénérer la grille et mettre à jour l'affichage des obstacles.
    Appelée à chaque fois qu'une zone est activée/désactivée.
    """
    global grid, grid_expanded, obstacle_array, expanded_array, distance_map
    global obstacle_scatter, expanded_scatter
    
    # Régénérer les grilles des deux gestionnaires
    grid_zones, grid_zones_expanded, obstacle_array_zones, expanded_array_zones = obs_manager.generer_grille()
    grid_noisettes, grid_noisettes_expanded, obstacle_array_noisettes, expanded_array_noisettes = obs_manager_noisettes.generer_grille()
    
    # Combiner les deux grilles
    grid = np.logical_or(grid_zones, grid_noisettes).astype(int)
    grid_expanded = np.logical_or(grid_zones_expanded, grid_noisettes_expanded).astype(int)
    
    # Combiner les arrays d'obstacles pour l'affichage
    obstacle_array = np.vstack([obstacle_array_zones, obstacle_array_noisettes]) if len(obstacle_array_zones) > 0 and len(obstacle_array_noisettes) > 0 else (obstacle_array_zones if len(obstacle_array_zones) > 0 else obstacle_array_noisettes)
    expanded_array = np.vstack([expanded_array_zones, expanded_array_noisettes]) if len(expanded_array_zones) > 0 and len(expanded_array_noisettes) > 0 else (expanded_array_zones if len(expanded_array_zones) > 0 else expanded_array_noisettes)
    
    # Mettre à jour la distance map pour A*
    distance_map = distance_transform_edt(~grid_expanded)
    initialiser_grille(grid_expanded, distance_map, width, height, case_mm)
    
    # Supprimer les anciens objets graphiques
    if obstacle_scatter is not None:
        obstacle_scatter.remove()
    if expanded_scatter is not None:
        expanded_scatter.remove()
    
    # Recréer l'affichage des obstacles
    obstacle_scatter, expanded_scatter = afficher_obstacles(
        ax, obstacle_array, expanded_array, 
        case_mm=case_mm, show_expanded=True, show_obstacles=False
    )
    
    # Redessiner
    fig.canvas.draw_idle()


def toggle_zone(event, zone_num):
    """
    Callback pour basculer l'état d'une zone (actif/inactif).
    VERSION SIMPLIFIÉE : Modifie UNIQUEMENT les listes.
    La mise à jour se fait automatiquement via verifier_changement_listes().
    
    Args:
        event: Événement du bouton
        zone_num: Numéro de la zone (1 à 10)
    """
    global Liste_GM_libres, Liste_GM_occuper
    
    # Vérifier l'état actuel dans les listes
    if zone_num in Liste_GM_libres:
        # Zone libre → la rendre occupée
        Liste_GM_libres.remove(zone_num)
        Liste_GM_occuper.append(zone_num)
        print(f"📦 Bouton Zone {zone_num} : Libre → Occupée")
        
    elif zone_num in Liste_GM_occuper:
        # Zone occupée → la rendre libre
        Liste_GM_occuper.remove(zone_num)
        Liste_GM_libres.append(zone_num)
        print(f"📦 Bouton Zone {zone_num} : Occupée → Libre")
        
    else:
        print(f"⚠️ Zone {zone_num} dans un état incohérent !")
        print(f"   Libres: {Liste_GM_libres}")
        print(f"   Occupées: {Liste_GM_occuper}")
    
    # La mise à jour visuelle se fera automatiquement
    # via verifier_changement_listes() dans la boucle principale


def toggle_noisette(event, noisette_num):
    """
    Callback pour basculer l'état d'une Noisette (actif/inactif).
    VERSION SIMPLIFIÉE : Modifie UNIQUEMENT les listes.
    La mise à jour se fait automatiquement via verifier_changement_listes().
    
    Args:
        event: Événement du bouton
        noisette_num: Numéro de la Noisette (1 à 8)
    """
    global Liste_noisettes_libres, Liste_noisettes_prises
    
    # Vérifier l'état actuel dans les listes
    if noisette_num in Liste_noisettes_libres:
        # Noisette libre (obstacle) → la prendre (passable)
        Liste_noisettes_libres.remove(noisette_num)
        Liste_noisettes_prises.append(noisette_num)
        print(f"🌰 Bouton Noisette {noisette_num} : Libre (obstacle) → Prise (passable)")
        
    elif noisette_num in Liste_noisettes_prises:
        # Noisette prise → la remettre libre
        Liste_noisettes_prises.remove(noisette_num)
        Liste_noisettes_libres.append(noisette_num)
        print(f"🌰 Bouton Noisette {noisette_num} : Prise → Libre (obstacle)")
        
    else:
        print(f"⚠️ Noisette {noisette_num} dans un état incohérent !")
        print(f"   Libres: {Liste_noisettes_libres}")
        print(f"   Prises: {Liste_noisettes_prises}")



def bouton_attraper_callback(event):
    """
    Callback pour le bouton Attraper.
    Modifie ordre_receive à 21 si l action en cours est "Attraper".
    """
    global ordre_receive, action_voulu
    
    # Vérifier qu il y a une action en cours
    if len(Liste_actions) > 0 and action_voulu == "Attraper":
        ordre_receive = 21
        print(f"🔘 Bouton ATTRAPER pressé → ordre_receive = 21")
        print(f"   ✅ Action Attraper confirmée !")
    else:
        print(f"⚠️  Bouton ATTRAPER pressé mais action en cours : {action_voulu}")
        print(f"   Le bouton ne fonctionne que si l action est Attraper")


def bouton_relacher_callback(event):
    """
    Callback pour le bouton Relacher.
    Modifie ordre_receive à 22 si l action en cours est "Relacher".
    """
    global ordre_receive, action_voulu
    
    # Vérifier qu il y a une action en cours
    if len(Liste_actions) > 0 and action_voulu == "Relacher":
        ordre_receive = 22
        print(f"🔘 Bouton RELACHER pressé → ordre_receive = 22")
        print(f"   ✅ Action Relacher confirmée !")
    else:
        print(f"⚠️  Bouton RELACHER pressé mais action en cours : {action_voulu}")
        print(f"   Le bouton ne fonctionne que si l action est Relacher")

def actualiser_zones_jeu():
    """
    Met à jour l'état des zones et régénère les grilles si nécessaire.
    Version ultra-simple : ne touche PAS aux boutons.
    """
    global grid, grid_expanded, obstacle_array, expanded_array
    global obs_manager, obs_manager_noisettes
    global Liste_GM_libres, Liste_GM_occuper
    global Liste_noisettes_libres, Liste_noisettes_prises
    global obstacle_scatter, expanded_scatter
    global distance_map, ax
    
    print("\n🔍 DEBUG - État des listes AVANT mise à jour :")
    print(f"   Liste_gardemanger_libres  : {Liste_GM_libres}")
    print(f"   Liste_gardemanger_occuper : {Liste_GM_occuper}")
    print(f"   Liste_noisettes_libres    : {Liste_noisettes_libres}")
    print(f"   Liste_noisettes_prises    : {Liste_noisettes_prises}")
    
    # Mettre à jour les états selon les listes
    grilles_modifiees, _, _ = mettre_a_jour_zones_dynamiques(
        obs_manager, obs_manager_noisettes,
        Liste_GM_libres, Liste_GM_occuper,
        Liste_noisettes_libres, Liste_noisettes_prises
    )
    
    # Régénérer si nécessaire
    if grilles_modifiees:
        print("🔄 Régénération des grilles...")
        
        grid, grid_expanded, obstacle_array, expanded_array = \
            regenerer_grilles_combinees(obs_manager, obs_manager_noisettes)
        
        # Recalculer distance map
        from scipy.ndimage import distance_transform_edt
        distance_map = distance_transform_edt(~grid_expanded)
        initialiser_grille(grid_expanded, distance_map, width, height, case_mm)
        
        # Mettre à jour affichage
        if obstacle_scatter is not None:
            obstacle_scatter.remove()
            obstacle_scatter = None
        if expanded_scatter is not None:
            expanded_scatter.remove()
            expanded_scatter = None
        
        obstacle_scatter, expanded_scatter = afficher_obstacles(
            ax, obstacle_array, expanded_array, case_mm,
            show_expanded=True, show_obstacles=False
        )
        
        print("✅ Grilles mises à jour")
        return True
    else:
        print("ℹ️  Aucune modification des grilles nécessaire")
        return False


def prendre_noisette(numero_noisette):
    """
    Marque une noisette comme prise.
    VERSION ULTRA-SIMPLE : Modifie UNIQUEMENT les listes.
    Les boutons ne sont PAS touchés.
    """
    global Liste_noisettes_libres, Liste_noisettes_prises
    
    print(f"\n🌰 Tentative de prendre noisette {numero_noisette}...")
    print(f"   État avant : libres={Liste_noisettes_libres}, prises={Liste_noisettes_prises}")
    
    if numero_noisette in Liste_noisettes_libres:
        # Modifier les listes
        Liste_noisettes_libres.remove(numero_noisette)
        Liste_noisettes_prises.append(numero_noisette)
        print(f"   ✅ Noisette {numero_noisette} retirée de 'libres' et ajoutée à 'prises'")
        print(f"   État après : libres={Liste_noisettes_libres}, prises={Liste_noisettes_prises}")
        
        # Mettre à jour obstacles et grilles
        actualiser_zones_jeu()
        return True
    else:
        print(f"   ⚠️  Noisette {numero_noisette} PAS dans Liste_noisettes_libres")
        print(f"   État actuel : libres={Liste_noisettes_libres}, prises={Liste_noisettes_prises}")
        return False


def occuper_zone(numero_zone):
    """
    Marque une zone comme occupée.
    VERSION ULTRA-SIMPLE : Modifie UNIQUEMENT les listes.
    Les boutons ne sont PAS touchés.
    """
    global Liste_GM_libres, Liste_GM_occuper
    
    print(f"\n📦 Tentative d'occuper zone {numero_zone}...")
    print(f"   État avant : libres={Liste_GM_libres}, occupées={Liste_GM_occuper}")
    
    if numero_zone in Liste_GM_libres:
        # Modifier les listes
        Liste_GM_libres.remove(numero_zone)
        Liste_GM_occuper.append(numero_zone)
        print(f"   ✅ Zone {numero_zone} retirée de 'libres' et ajoutée à 'occupées'")
        print(f"   État après : libres={Liste_GM_libres}, occupées={Liste_GM_occuper}")
        
        # Mettre à jour obstacles et grilles
        actualiser_zones_jeu()
        return True
    else:
        print(f"   ⚠️  Zone {numero_zone} PAS dans Liste_gardemanger_libres")
        print(f"   État actuel : libres={Liste_GM_libres}, occupées={Liste_GM_occuper}")
        return False


def liberer_zone(numero_zone):
    """
    Libère une zone précédemment occupée.
    VERSION ULTRA-SIMPLE : Modifie UNIQUEMENT les listes.
    """
    global Liste_GM_libres, Liste_GM_occuper
    
    print(f"\n📦 Tentative de libérer zone {numero_zone}...")
    
    if numero_zone in Liste_GM_occuper:
        Liste_GM_occuper.remove(numero_zone)
        Liste_GM_libres.append(numero_zone)
        print(f"   ✅ Zone {numero_zone} retirée de 'occupées' et ajoutée à 'libres'")
        
        actualiser_zones_jeu()
        return True
    else:
        print(f"   ⚠️  Zone {numero_zone} PAS dans Liste_gardemanger_occuper")
        return False


def relacher_noisette(numero_noisette):
    """
    Remet une noisette dans les noisettes libres.
    VERSION ULTRA-SIMPLE : Modifie UNIQUEMENT les listes.
    """
    global Liste_noisettes_libres, Liste_noisettes_prises
    
    print(f"\n🌰 Tentative de relâcher noisette {numero_noisette}...")
    
    if numero_noisette in Liste_noisettes_prises:
        Liste_noisettes_prises.remove(numero_noisette)
        Liste_noisettes_libres.append(numero_noisette)
        print(f"   ✅ Noisette {numero_noisette} retirée de 'prises' et ajoutée à 'libres'")
        
        actualiser_zones_jeu()
        return True
    else:
        print(f"   ⚠️  Noisette {numero_noisette} PAS dans Liste_noisettes_prises")
        return False

def distance_euclidienne(x1, y1, x2, y2):
    """Calcule la distance euclidienne entre deux points"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# ==================== CALCUL DU SCORE ====================

def calculer_score_tache(x_tache, y_tache, type_tache, robot_a_objets,
                         x_robot_actuel, y_robot_actuel, 
                         x_ennemi, y_ennemi, R_securite):
    """
    Calcule le score d'une tâche selon la moyenne pondérée.
    """
    
    # 1. FACTEUR DISTANCE
    distance = distance_euclidienne(x_robot_actuel, y_robot_actuel, x_tache, y_tache)
    distance_normalisee = distance / DISTANCE_MAX_TERRAIN
    f_distance = 1.0 / (1.0 + distance_normalisee)
    
    # 2. FACTEUR SÉCURITÉ
    distance_tache_ennemi = distance_euclidienne(x_tache, y_tache, x_ennemi, y_ennemi)
    
    if distance_tache_ennemi < R_securite:
        f_securite = 0.0
    else:
        f_securite = min(1.0, distance_tache_ennemi / (2.5 * R_securite))
    
    # 3. FACTEUR PRIORITÉ
    if type_tache == "Attraper":
        f_priorite = 1.0
    else:
        f_priorite = 0.8
    
    # 4. FACTEUR EFFICACITÉ
    if robot_a_objets and type_tache == "Relacher":
        f_efficacite = 1.0
    elif not robot_a_objets and type_tache == "Attraper":
        f_efficacite = 1.0
    else:
        f_efficacite = 0.3
    
    # SCORE TOTAL
    score = (W_DISTANCE * f_distance + 
             W_SECURITE * f_securite + 
             W_PRIORITE * f_priorite + 
             W_EFFICACITE * f_efficacite)
    
    return score

# ==================== ALGORITHME PRINCIPAL ====================

def choisir_prochaine_action(x_robot_actuel, y_robot_actuel, 
                            x_ennemi, y_ennemi, 
                            robot_a_objets,
                            Liste_noisettes_libres, Liste_GM_libres,
                            Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
                            Liste_zones_gm_xy, Liste_zones_gm_angle,
                            R_securite):
    """
    Choisit la prochaine action à effectuer selon l'algorithme de moyenne pondérée.
    
    ⚠️ IMPORTANT : Cette fonction NE modifie PAS les listes de zones.
    Les listes seront modifiées uniquement après confirmation de l'action (ordre 21 ou 22).
    
    Returns:
        dict ou None
    """
    
    meilleur_score = -float('inf')
    meilleure_action = None
    
    # PHASE 1 : Si le robot n'a pas d'objets → ATTRAPER
    if not robot_a_objets:
        for num_noisette in Liste_noisettes_libres:
            index = num_noisette - 1
            
            x_noisette = Liste_zones_recup_noisettes_xy[index][0]
            y_noisette = Liste_zones_recup_noisettes_xy[index][1]
            angle_noisette = Liste_zones_recup_noisettes_angle[index]
            
            score = calculer_score_tache(
                x_noisette, y_noisette, "Attraper", 
                robot_a_objets,
                x_robot_actuel, y_robot_actuel,
                x_ennemi, y_ennemi, R_securite
            )
            
            if score > meilleur_score:
                meilleur_score = score
                meilleure_action = {
                    'type': 'Attraper',
                    'numero_zone': num_noisette,
                    'x': x_noisette,
                    'y': y_noisette,
                    'angle': angle_noisette,
                    'score': score
                }
    
    # PHASE 2 : Si le robot a des objets → RELACHER
    else:
        for num_gm in Liste_GM_libres:
            index = num_gm - 1
            
            x_gm = Liste_zones_gm_xy[index][0]
            y_gm = Liste_zones_gm_xy[index][1]
            angle_gm = Liste_zones_gm_angle[index]
            
            score = calculer_score_tache(
                x_gm, y_gm, "Relacher",
                robot_a_objets,
                x_robot_actuel, y_robot_actuel,
                x_ennemi, y_ennemi, R_securite
            )
            
            if score > meilleur_score:
                meilleur_score = score
                meilleure_action = {
                    'type': 'Relacher',
                    'numero_zone': num_gm,
                    'x': x_gm,
                    'y': y_gm,
                    'angle': angle_gm,
                    'score': score
                }
    
    return meilleure_action

# ==================== AJOUT À LA LISTE D'ACTIONS ====================

def ajouter_action_a_liste(Liste_actions, action_choisie, verbose=True):
    """
    Ajoute les commandes nécessaires à Liste_actions.
    
    ⚠️ IMPORTANT : Cette fonction NE modifie PAS les listes de zones.
    
    Returns:
        dict ou None: action_choisie (pour la stocker et l'utiliser plus tard)
    """
    if action_choisie is None:
        if verbose:
            print("⚠️  AUCUNE ACTION POSSIBLE !")
        return None
    
    type_action = action_choisie['type']
    numero_zone = action_choisie['numero_zone']
    x = action_choisie['x']
    y = action_choisie['y']
    angle = action_choisie['angle']
    score = action_choisie['score']
    
    if verbose:
        print(f"\n{'='*70}")
        print(f"✅ ACTION CHOISIE : {type_action} zone {numero_zone}")
        print(f"{'='*70}")
        print(f"   📍 Position : ({x}, {y}) mm")
        print(f"   🧭 Angle    : {angle}°")
        print(f"   ⭐ Score    : {score:.3f}")
        print(f"{'='*70}\n")
    
    # Ajouter les commandes à la liste
    Liste_actions.append(["Consigne", x, y])
    Liste_actions.append(["Rotation", angle])
    Liste_actions.append([type_action])
    
    return action_choisie

# ==================== NOUVELLE FONCTION : CONFIRMATION D'ACTION ====================

def confirmer_action_terminee(action_en_cours,
                              robot_a_objets,
                              Liste_noisettes_libres, Liste_noisettes_prises,
                              Liste_GM_libres, Liste_GM_occuper,
                              verbose=True):
    """
    Met à jour les listes de zones et l'état du robot APRÈS confirmation
    de la réalisation de l'action (ordre 21 ou 22 reçu).
    
    ⭐ CETTE FONCTION DOIT ÊTRE APPELÉE APRÈS AVOIR REÇU L'ORDRE 21 ou 22 ⭐
    
    Args:
        action_en_cours: dict contenant les infos de l'action (type, numero_zone, etc.)
        robot_a_objets: État actuel du robot
        Liste_noisettes_libres, Liste_noisettes_prises: Listes à mettre à jour
        Liste_GM_libres, Liste_GM_occuper: Listes à mettre à jour
        verbose: Afficher les infos
    
    Returns:
        bool: Nouvel état de robot_a_objets
    """
    if action_en_cours is None:
        return robot_a_objets
    
    type_action = action_en_cours['type']
    numero_zone = action_en_cours['numero_zone']
    
    if type_action == "Attraper":
        # L'action Attraper est terminée → mettre à jour les listes
        if numero_zone in Liste_noisettes_libres:
            Liste_noisettes_libres.remove(numero_zone)
            Liste_noisettes_prises.append(numero_zone)
            if verbose:
                print(f"✅ Noisette {numero_zone} RÉCUPÉRÉE avec succès !")
                print(f"   Noisettes restantes : {Liste_noisettes_libres}")
        robot_a_objets = True
        
    elif type_action == "Relacher":
        # L'action Relacher est terminée → mettre à jour les listes
        if numero_zone in Liste_GM_libres:
            Liste_GM_libres.remove(numero_zone)
            Liste_GM_occuper.append(numero_zone)
            if verbose:
                print(f"✅ GM {numero_zone} REMPLI avec succès !")
                print(f"   GM restants : {Liste_GM_libres}")
        robot_a_objets = False
    
    return robot_a_objets

# ==================== FONCTION DE MISE À JOUR (VERSION CORRIGÉE) ====================

def mise_a_jour_decision(Liste_actions, 
                        x_robot_actuel, y_robot_actuel, 
                        x_ennemi, y_ennemi, 
                        robot_a_objets,
                        Liste_noisettes_libres,
                        Liste_GM_libres,
                        Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
                        Liste_zones_gm_xy, Liste_zones_gm_angle,
                        R_securite,
                        verbose=True):
    """
    Fonction de décision qui choisit la prochaine action et l'ajoute à Liste_actions.
    
    ⚠️ MODIFICATION IMPORTANTE : Cette fonction NE met PLUS à jour les listes de zones.
    Les listes seront mises à jour par confirmer_action_terminee() après réception de l'ordre 21/22.
    
    Returns:
        dict ou None: L'action choisie (à stocker pour confirmer plus tard)
    """
    
    # 1. Choisir la meilleure action
    action = choisir_prochaine_action(
        x_robot_actuel, y_robot_actuel,
        x_ennemi, y_ennemi,
        robot_a_objets,
        Liste_noisettes_libres, Liste_GM_libres,
        Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
        Liste_zones_gm_xy, Liste_zones_gm_angle,
        R_securite
    )
    
    # 2. Ajouter l'action à la liste (sans modifier les listes de zones)
    action_ajoutee = ajouter_action_a_liste(Liste_actions, action, verbose)
    
    # 3. Retourner l'action pour pouvoir la confirmer plus tard
    return action_ajoutee

# ==================== FONCTION DE DEBUG ====================

def afficher_tous_les_scores(x_robot_actuel, y_robot_actuel,
                             x_ennemi, y_ennemi,
                             robot_a_objets,
                             Liste_noisettes_libres, Liste_GM_libres,
                             Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
                             Liste_zones_gm_xy, Liste_zones_gm_angle,
                             R_securite):
    """
    Fonction de debug : affiche les scores de TOUTES les actions possibles.
    """
    print("\n" + "="*70)
    print("DEBUG : SCORES DE TOUTES LES ACTIONS POSSIBLES")
    print("="*70)
    
    if not robot_a_objets:
        print("\n🔵 PHASE RÉCOLTE (Attraper)")
        print("-" * 70)
        for num_noisette in Liste_noisettes_libres:
            index = num_noisette - 1
            x_n = Liste_zones_recup_noisettes_xy[index][0]
            y_n = Liste_zones_recup_noisettes_xy[index][1]
            
            score = calculer_score_tache(
                x_n, y_n, "Attraper", robot_a_objets,
                x_robot_actuel, y_robot_actuel,
                x_ennemi, y_ennemi, R_securite
            )
            
            dist = distance_euclidienne(x_robot_actuel, y_robot_actuel, x_n, y_n)
            dist_ennemi = distance_euclidienne(x_n, y_n, x_ennemi, y_ennemi)
            
            print(f"  Noisette {num_noisette:2d} → Score: {score:.3f} | "
                  f"Dist robot: {dist:4.0f}mm | Dist ennemi: {dist_ennemi:4.0f}mm")
    
    else:
        print("\n🔴 PHASE DÉPÔT (Relacher)")
        print("-" * 70)
        for num_gm in Liste_GM_libres:
            index = num_gm - 1
            x_g = Liste_zones_gm_xy[index][0]
            y_g = Liste_zones_gm_xy[index][1]
            
            score = calculer_score_tache(
                x_g, y_g, "Relacher", robot_a_objets,
                x_robot_actuel, y_robot_actuel,
                x_ennemi, y_ennemi, R_securite
            )
            
            dist = distance_euclidienne(x_robot_actuel, y_robot_actuel, x_g, y_g)
            dist_ennemi = distance_euclidienne(x_g, y_g, x_ennemi, y_ennemi)
            
            print(f"  GM {num_gm:2d} → Score: {score:.3f} | "
                  f"Dist robot: {dist:4.0f}mm | Dist ennemi: {dist_ennemi:4.0f}mm")
    
    print("="*70 + "\n")

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
    actualiser_zones_jeu()
    # création du trait, initialement à la position du robot
    ax.add_line(robot_angle_line)
    ax.add_line(robot_angle_voulu_line)

    # ========== CRÉATION DES 10 BOUTONS POUR LES ZONES ==========
    # Position de départ des boutons (à droite de l'écran)
    button_width = 0.12
    button_height = 0.08
    button_x = 0.87  # Position X à droite
    button_y_start = 0.88  # Position Y de départ (en haut)
    button_spacing = 0.08  # Espacement vertical entre les boutons
    
    for i in range(1, 11):  # Créer 10 boutons pour les zones 1 à 10
        # Calculer la position Y de chaque bouton
        button_y = button_y_start - (i - 1) * button_spacing
        
        # Créer l'axe pour le bouton
        ax_zone_button = plt.axes([button_x, button_y, button_width, button_height])
        
        # Créer le bouton avec le texte initial (INACTIF au départ)
        zone_button = Button(ax_zone_button, f'Zone {i}\n✗ INACTIF', color='lightgreen', hovercolor='green')
        
        # Connecter le callback avec le numéro de zone
        zone_button.on_clicked(partial(toggle_zone, zone_num=i))
        
        # Stocker le bouton dans la liste
        boutons_zones.append(zone_button)
        
    # ========== CRÉATION DES 8 BOUTONS POUR LES NOISETTES ==========
    # Position des boutons (en bas de l'écran, horizontalement)
    noisette_button_width = 0.08
    noisette_button_height = 0.06
    noisette_button_y = 0.02  # Position Y en bas
    noisette_button_x_start = 0.15  # Position X de départ (centrée)
    noisette_button_spacing = 0.09  # Espacement horizontal entre les boutons
    
    for i in range(1, 9):  # Créer 8 boutons pour les Noisettes 1 à 8
        # Calculer la position X de chaque bouton
        noisette_button_x = noisette_button_x_start + (i - 1) * noisette_button_spacing
        
        # Créer l'axe pour le bouton
        ax_noisette_button = plt.axes([noisette_button_x, noisette_button_y, noisette_button_width, noisette_button_height])
        
        # Créer le bouton avec le texte initial (ACTIF au départ)
        noisette_button = Button(ax_noisette_button, f'N{i}\n✓ ACTIF', color='lightcoral', hovercolor='red')
        
        # Connecter le callback avec le numéro de Noisette
        noisette_button.on_clicked(partial(toggle_noisette, noisette_num=i))
        
        # Stocker le bouton dans la liste
        boutons_noisettes.append(noisette_button)


    # ========== CRÉATION DES 2 BOUTONS ATTRAPER ET RELACHER ==========
    # Position des boutons (en haut de l écran, centrés)
    action_button_width = 0.12
    action_button_height = 0.08
    action_button_y = 0.92  # Position Y en haut
    action_button_x_start = 0.38  # Position X de départ (centrés)
    action_button_spacing = 0.14  # Espacement horizontal entre les boutons
    
    # Bouton ATTRAPER
    ax_attraper_button = plt.axes([action_button_x_start, action_button_y, action_button_width, action_button_height])
    bouton_attraper = Button(ax_attraper_button, "🤖 ATTRAPER", color="lightblue", hovercolor="blue")
    bouton_attraper.on_clicked(bouton_attraper_callback)
    
    # Bouton RELACHER
    ax_relacher_button = plt.axes([action_button_x_start + action_button_spacing, action_button_y, action_button_width, action_button_height])
    bouton_relacher = Button(ax_relacher_button, "🤖 RELACHER", color="lightyellow", hovercolor="orange")
    bouton_relacher.on_clicked(bouton_relacher_callback)
    
    print("✅ Boutons ATTRAPER et RELACHER créés")

    if Reel: 
        tache_lidar = threading.Thread(target=calcul_points, args=(stop_event,), daemon=False)
        tache_calcul = threading.Thread(target=calcul_ennemi, args=(stop_event,), daemon=False)
        tache_LectureCAN = threading.Thread(target=LectureCAN, args=(stop_event,), daemon=True)
        
        tache_lidar.start()
        tache_calcul.start()
        tache_LectureCAN.start()
    
    try:
        dico_envoi[0x200]=x_robot_depart
        dico_envoi[0x201]=y_robot_depart
        dico_envoi[0x202]=angle_robot_depart
        if Reel: 
            bus.send(can.Message(arbitration_id=0x200, data=struct.pack('<f',dico_envoi[0x200]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x201, data=struct.pack('<f',dico_envoi[0x201]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x202, data=struct.pack('<f',dico_envoi[0x202]), is_extended_id=False))
        
        # Initialisation des variables de trajectoire
        path_plot = None
        path_simplified_plot = None
        path_smooth_plot = None
        action_en_cours = mise_a_jour_decision(
            Liste_actions,
            x_robot_actuel, y_robot_actuel,
            x_ennemi, y_ennemi,
            robot_a_objets,
            Liste_noisettes_libres,
            Liste_GM_libres,
            Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
            Liste_zones_gm_xy, Liste_zones_gm_angle,
            R_securite,
            verbose=True
        )

        while (not stop_event.is_set() and Batteries[2][3] > 5 and len(Liste_actions)!=0): # Tant que le Flag de Thread n'est pas levé et que les batteries sont suffisamment chargées
            dico_envoi[0x01]=1
            temps = 0
            verif = False
            step +=1
            actualiser_zones_jeu()
            print("\n🔬 TEST 1 : Affichage de tous les scores")
            afficher_tous_les_scores(
                x_robot_actuel, y_robot_actuel,
                x_ennemi, y_ennemi,
                robot_a_objets,
                Liste_noisettes_libres, Liste_GM_libres,
                Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
                Liste_zones_gm_xy, Liste_zones_gm_angle,
                R_securite
            )
            # === TEST 2 : Prendre une décision ===
            print("\n🤖 TEST 2 : Prise de décision")
            """robot_a_objets = mise_a_jour_decision(
                Liste_actions,
                x_robot_actuel, y_robot_actuel,
                x_ennemi, y_ennemi,
                robot_a_objets,
                Liste_noisettes_libres, Liste_noisettes_prises,
                Liste_GM_libres, Liste_GM_occuper,
                Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
                Liste_zones_gm_xy, Liste_zones_gm_angle,
                R_securite,
                verbose=True
            )"""
            if not Reel: 
                x_ennemi += 6
                y_ennemi -= 4
                if(step == 130):
                    x_ennemi = 1500
                    y_ennemi = 450
                """if(step == 40):
                    Liste_GM_libres = [1,2,3,5,6,7,8,9,10]
                    Liste_GM_occuper = [4] 
                    Liste_noisettes_libres = [2,3,4,5,6,7,8]
                    Liste_noisettes_prises = [1]
                if(step == 60):
                    Liste_GM_libres = [2,3,5,6,7,8,9,10]
                    Liste_GM_occuper = [1,4] 
                    Liste_noisettes_libres = [2,3,4,5,6,7,8]
                    Liste_noisettes_prises = [1]"""
                if(step==700):
                    x_ennemi = 1000
                    y_ennemi = 1000

            if type(Liste_actions[0]) == list and len(Liste_actions[0])==3: # Si la consigne est une coordonnée
                action_voulu = Liste_actions[0][0]
                x_robot_voulu = Liste_actions[0][1]
                y_robot_voulu = Liste_actions[0][2]
                verif = True
                mouvement = True
            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==2: # Si la consigne est un angle
                action_voulu = Liste_actions[0][0]
                angle_robot_voulu = round(Liste_actions[0][1],0)
                mouvement = True

            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==1:
                print("Appeler Carte Moteur pour : " + Liste_actions[0][0])
                action_voulu = Liste_actions[0][0]
                mouvement = False
                 
            print(step)
            print(trajectoire_bloquee)
            print("\nAvant verif :")
            print("Action en cours : "+action_voulu)
            print(f"X_actuel = {x_robot_actuel} Y_actuel = {y_robot_actuel} Angle_actuel = {angle_robot_actuel}°")
            print(f"X_voulu = {x_robot_voulu} Y_voulu = {y_robot_voulu} Angle_voulu = {angle_robot_voulu}°")

            distance_robot_ennemi = math.sqrt((x_ennemi - x_robot_actuel)**2 + (y_ennemi - y_robot_actuel)**2)
            angle_ennemi = np.degrees(math.atan2(y_ennemi-y_ennemi_old,x_ennemi-x_ennemi_old))
             
            # === CALCUL DE LA TRAJECTOIRE A* === 
            if action_voulu in ["Rotation", "Tourner","Attraper","Relacher"]:
                pass  # rien, on attend la fin de la rotation
            else:
                if action_voulu in ["Consigne",'Avancer'] or ((x_ennemi != x_ennemi_old) or (y_ennemi != y_ennemi_old)):
                    Liste_actions = [action for action in Liste_actions if action[0] in ["Consigne","Rotation","Attraper","Relacher"]]

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
                    path = astar_safe(start_case, goal_case,1.0,grid_dynamique=grid_avec_ennemi)
                    
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
                        nbr_point = 0
                        last_inserted = None  # (x,y) du dernier waypoint inséré pour éviter doublons consécutifs

                        consigne_existante = None
                        if len(Liste_actions) > 0 and type(Liste_actions[0]) == list and len(Liste_actions[0]) == 3:
                            # action [ "Consigne", x, y ] typiquement
                            consigne_existante = (round(Liste_actions[0][1]), round(Liste_actions[0][2]))

                        for i in range(len(py_mm)-1, 0, -1):  # Parcourir à l'envers (comme avant)
                            x_cible = round(px_mm[i])
                            y_cible = round(py_mm[i])

                            # Eviter d'insérer si c'est (presque) la même position que la consigne existante
                            if consigne_existante is not None:
                                dxc = abs(x_cible - consigne_existante[0])
                                dyc = abs(y_cible - consigne_existante[1])
                                if dxc <= EPS_EQ and dyc <= EPS_EQ:
                                    # -> Ne pas insérer ce waypoint (évite Avancer identique à Consigne)
                                    print(f"  → Skip waypoint identique à la Consigne ({x_cible},{y_cible})")
                                    continue

                            # Eviter doublons consécutifs (si même coordonnée que le dernier inséré)
                            if last_inserted is not None:
                                if abs(x_cible - last_inserted[0]) <= EPS_EQ and abs(y_cible - last_inserted[1]) <= EPS_EQ:
                                    print(f"  → Skip waypoint dupliqué consécutif ({x_cible},{y_cible})")
                                    continue

                            # Insérer le waypoint
                            Liste_actions.insert(0, ["Avancer", x_cible, y_cible])
                            last_inserted = (x_cible, y_cible)
                            nbr_point += 1
                            print(f"  → Waypoint {nbr_point}: Avancer vers ({x_cible:.0f}, {y_cible:.0f})")

                        # Construire Liste_trajectoire en corrigant le test buggué
                        if nbr_point >= 1:
                            Liste_trajectoire = []
                            Liste_trajectoire.append(nbr_point)
                            for action in Liste_actions:
                                # CORRECTION : utiliser == au lieu de in "Avancer"
                                if action[0] == "Avancer" and len(action) == 3:
                                    Liste_trajectoire.append([action[1], action[2]])


                        print(Liste_trajectoire)
                        path_simplified_plot, = ax.plot(px_mm, py_mm, 'g-', linewidth=2,label='Chemin A*', marker='o', markersize=4, zorder=10)
                        
                        # Afficher le chemin lissé
                        if x_smooth is not None and y_smooth is not None:
                            # Convertir en millimètres pour l'affichage
                            x_smooth_mm = [x * case_mm for x in x_smooth]
                            y_smooth_mm = [y * case_mm for y in y_smooth]
                            #path_smooth_plot, = ax.plot(x_smooth_mm, y_smooth_mm, 'orange',linewidth=3, label='Trajectoire lissée', zorder=11)
                        
                    else:
                        print("❌ Aucun chemin trouvé par A*")
                        if distance_robot_ennemi < R_securite :
                            print("BESOIN DE RECULER")
                            ratio = R_securite / distance_robot_ennemi
                            x_recul = round(x_ennemi - (x_ennemi-x_robot_actuel) * ratio,0)
                            y_recul = round(y_ennemi - (y_ennemi-y_robot_actuel) * ratio,0)
                            x_recul = clamp(x_recul, r_robot, x_piste-r_robot)
                            y_recul = clamp(y_recul, r_robot, y_piste-r_robot)
                            Liste_actions = [action for action in Liste_actions if(action[0] in ["Consigne","Rotation"])]
                            Liste_actions.insert(0,["Avancer",int(x_recul),int(y_recul)])
                            Liste_actions.insert(0,["Recul",int(x_recul),int(y_recul)])
                            angle_prochain_point = calcul_angle_vers_point(
                                x_robot_actuel, y_robot_actuel,
                                Liste_actions[0][1], Liste_actions[0][2]
                            )
                            if(angle_prochain_point != angle_robot_actuel):
                                Liste_actions.insert(0, ["Tourner", angle_prochain_point])

                        else :
                            print("CHEMIN INACCESSIBLE, RECALCUL DE TRAJECTOIRE")
                            x_robot_voulu = x_robot_actuel
                            y_robot_voulu = y_robot_actuel
            """if type(Liste_actions[0]) == list and len(Liste_actions[0])==3: # Si la consigne est une coordonnée
                if Liste_actions[0][0] == "Avancer" and Liste_actions[1][0] == "Consigne":
                    if Liste_actions[0][1]==Liste_actions[1][1] and Liste_actions[0][2]==Liste_actions[1][2]:
                        Liste_actions.pop(0)"""
            if type(Liste_actions[0]) == list and len(Liste_actions[0])==3: # Si la consigne est une coordonnée
                action_voulu = Liste_actions[0][0]
                x_robot_voulu = Liste_actions[0][1]
                y_robot_voulu = Liste_actions[0][2]
                verif = True
                mouvement = True
            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==2: # Si la consigne est un angle
                action_voulu = Liste_actions[0][0]
                angle_robot_voulu = round(Liste_actions[0][1],0)
                mouvement = True

            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==1:
                print("Appeler Carte Moteur pour : " + Liste_actions[0][0])
                action_voulu = Liste_actions[0][0]
                mouvement = False
                    

            verif = False
            # ✅ Ne PAS recalculer pendant rotation (laisser tourner)
            if action_voulu in ["Consigne","Avancer"]  or (x_ennemi != x_ennemi_old) or (y_ennemi != y_ennemi_old):
                # Calculer distance à l'ennemi
                dx_ennemi = x_ennemi - x_robot_actuel
                dy_ennemi = y_ennemi - y_robot_actuel
                distance_ennemi = math.sqrt(dx_ennemi**2 + dy_ennemi**2)
                
                # ✅ Vérifier si waypoints futurs bloqués par ennemi
                trajectoire_bloquee = False
                
                # ✅ Vérifier SI ennemi bouge OU SI robot proche ennemi
                delta_x_ennemi = abs(x_ennemi - x_ennemi_old)
                delta_y_ennemi = abs(y_ennemi - y_ennemi_old)
                ennemi_bouge_significatif = (delta_x_ennemi > 10 or delta_y_ennemi > 10)
                robot_proche_ennemi = distance_ennemi < R_securite   # ✅ Nouveau
                
                # ✅ Recalculer si ennemi bouge OU si robot se rapproche
                if ennemi_bouge_significatif or robot_proche_ennemi:
                    if ennemi_bouge_significatif:
                        print(f"🔄 Ennemi a bougé : ({x_ennemi_old}, {y_ennemi_old}) → ({x_ennemi}, {y_ennemi})")
                    if robot_proche_ennemi:
                        print(f"⚠️  Robot proche ennemi : {distance_ennemi:.0f}mm (seuil {R_securite :.0f}mm)")
                    
                    # Vérifier tous les waypoints
                    for action in Liste_actions:
                        if action[0] == "Avancer" and len(action) == 3:
                            x_waypoint = action[1]
                            y_waypoint = action[2]
                            dx_w = x_ennemi - x_waypoint
                            dy_w = y_ennemi - y_waypoint
                            dist_waypoint_ennemi = math.sqrt(dx_w**2 + dy_w**2)
                            if dist_waypoint_ennemi < R_securite:
                                print(f"⚠️  Waypoint ({x_waypoint:.0f}, {y_waypoint:.0f}) bloqué par ennemi à {dist_waypoint_ennemi:.0f}mm")
                                trajectoire_bloquee = True
                                break  # ✅ Sort dès qu'un waypoint bloqué
                
                # Si ennemi dans zone critique ( rayon sécurité)
                if distance_ennemi < (R_securite) or trajectoire_bloquee:
                    print(f"⚠️  ENNEMI DÉTECTÉ à {distance_ennemi:.0f}mm ! Recalcul trajectoire...")
                    
                    # Nettoyer waypoints intermédiaires
                    Liste_actions = [act for act in Liste_actions if act[0] in ["Consigne", "Rotation", "Attraper", "Relacher"]]
                    print(Liste_actions)
                    # Préparer recalcul
                    x_robot_voulu = Liste_actions[0][1]
                    y_robot_voulu = Liste_actions[0][2]
                    
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
                    path = astar_safe(start_case, goal_case, safety_weight=1.5,grid_dynamique=grid_avec_ennemi)
                    
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
                        
                        for i in range(len(py_mm)-1, 0, -1):
                            x_cible = px_mm[i]
                            y_cible = py_mm[i]
                            
                            if i > 0:
                                x_precedent = px_mm[i-1]
                                y_precedent = py_mm[i-1]
                            else:
                                x_precedent = x_robot_actuel
                                y_precedent = y_robot_actuel
                            
                            
                            Liste_actions.insert(0, ["Avancer", x_cible, y_cible])
                            print(f"  → Nouveau waypoint: Avancer vers ({x_cible:.0f}, {y_cible:.0f})")
                        
                        # Afficher nouveau chemin
                        path_simplified_plot, = ax.plot(px_mm, py_mm, 'g-', linewidth=2,label='Nouveau chemin', marker='o', markersize=4, zorder=10)
               
            print("\nAprès verif :")
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
                    x_robot_actuel += round(15*np.cos(angle_robot_consigne),0)
                    y_robot_actuel += round(15*np.sin(angle_robot_consigne),0)

            # Envoi des Ordres de Consigne à la Carte Moteur
            dico_envoi[0x203]=x_robot_voulu
            dico_envoi[0x204]=y_robot_voulu
            dico_envoi[0x205]=angle_robot_voulu
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
            if Reel: 
                if(abs(x_robot_actuel-x_robot_voulu)<10 and abs(y_robot_actuel-y_robot_voulu)<10 and action_voulu in ["Consigne","Avancer","Recul","Contournement"]):
                    print("Bonne position")
                    ordre_receive = 11
                if(abs(angle_robot_actuel-angle_robot_voulu)<1) and action_voulu in ["Rotation","Tourner"]:
                    print("Bon Angle")
                    ordre_receive = 12
            else :
                if(abs(x_robot_actuel-x_robot_voulu)<30 and abs(y_robot_actuel-y_robot_voulu)<30 and action_voulu in ["Consigne","Avancer","Recul","Contournement"]):
                    print("Bonne position")
                    ordre_receive = 11
                    print(ordre_receive)
                if(abs(angle_robot_actuel-angle_robot_voulu)<5) and action_voulu in ["Rotation","Tourner"]:
                    print("Bon Angle")
                    ordre_receive = 12
                    
            if (action_voulu in ["Consigne","Avancer"] and ordre_receive == 11) or \
               (action_voulu in ["Rotation"] and ordre_receive == 12) or \
               (action_voulu == "Attraper" and ordre_receive == 21) or \
               (action_voulu == "Relacher" and ordre_receive == 22):
                
                print(f"\n{'='*70}")
                print(f"✅ ACTION TERMINÉE : {action_voulu}")
                print(f"{'='*70}")
                
                # Retirer l'action de la liste 
                Liste_actions.pop(0) 
                
                # ⭐ SI c'est une action ATTRAPER ou RELACHER, confirmer l'action ⭐
                if action_voulu in ["Attraper", "Relacher"]:
                    robot_a_objets = confirmer_action_terminee(
                        action_en_cours,
                        robot_a_objets,
                        Liste_noisettes_libres, Liste_noisettes_prises,
                        Liste_GM_libres, Liste_GM_occuper,
                        verbose=True
                    )
                    print(f"🤖 État du robot : {'Transporte des objets' if robot_a_objets else 'Vide'}")
                    print(f"{'='*70}\n")
                
                ordre_receive = 0
                
                # ⭐ SI la liste d'actions est vide, prendre une nouvelle décision ⭐
                if len(Liste_actions) == 0:
                    print("\n" + "="*70)
                    print("🔄 LISTE D'ACTIONS VIDE - PRISE DE NOUVELLE DÉCISION")
                    print("="*70)
                    
                    # Nouvelle décision
                    action_en_cours = mise_a_jour_decision(
                        Liste_actions,
                        x_robot_actuel, y_robot_actuel,
                        x_ennemi, y_ennemi,
                        robot_a_objets,
                        Liste_noisettes_libres,
                        Liste_GM_libres,
                        Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
                        Liste_zones_gm_xy, Liste_zones_gm_angle,
                        R_securite,
                        verbose=True
                    )
                    
                    if len(Liste_actions) == 0:
                        print("⚠️  AUCUNE ACTION DISPONIBLE - Mission terminée ou zones bloquées")



            # MAJ de l'affichage et des Variables de Bouncing
            plt.draw()
            fig.canvas.draw()
            fig.canvas.flush_events()
            time.sleep(0.001)
            U_last = [Batteries[0][2],Batteries[1][2],Batteries[2][2]]
            x_robot_voulu_last = x_robot_voulu
            y_robot_voulu_last = y_robot_voulu
            x_ennemi_old = x_ennemi
            y_ennemi_old = y_ennemi
            print("")
            
            # On supprime les anciens points de la trajectoire
            for key in list(dico_envoi.keys()):
                if 0x206 <= key <= 0x2FF:
                    del dico_envoi[key]
            # On remplit le dictionnaire d'envoi du CAN avec les points de la trajectoire
            dico_envoi[0x206] = Liste_trajectoire[0]
            for couple in range(1,len(Liste_trajectoire)):
                dico_envoi[0x207+2*(couple-1)]=Liste_trajectoire[couple][0]
                dico_envoi[0x208+2*(couple-1)]=Liste_trajectoire[couple][1]

            """for couple in dico_envoi.items():
                print(hex(couple[0])," : ",couple[1])"""

            if Reel :
                for key, value in dico_envoi.items() :
                    if value != 0:
                        if key in [0x01,0x300,0x301,0x302]:
                            format_value = struct.pack('<I',dico_envoi[key])
                        else :
                            format_value = struct.pack('<f',dico_envoi[key])
                        msg = can.Message(arbitration_id=key, data=format_value, is_extended_id=False)
                        bus.send(msg)
                        dico_envoi[key]=0

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
