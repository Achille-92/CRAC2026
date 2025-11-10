################## Librairies ##########################################
import matplotlib
matplotlib.use('Qt5Agg')
import math
import numpy as np
import threading
import time
import os
import socket
import json
import matplotlib.pyplot as plt
from functools import partial
from matplotlib.widgets import Button
from scipy.ndimage import binary_dilation
from affichage import init_affichage, bring_to_front,afficher_obstacles,afficher_zone_securite_ennemi, mettre_a_jour_zone_ennemi,afficher_batteries
from calcul_mouv import euclidienne,astar_safe,simplify_path_safe,smooth_path_safe,creer_grille_avec_ennemi,initialiser_grille
from fonction import Obstacles,gerer_basculement_batteries,clamp,calcul_angle_vers_point
from gestion_zones_dynamiques import actualiser_zones_jeu, verifier_et_changer_cible_si_necessaire, mise_a_jour_decision, confirmer_action_terminee
########################################################################

# A envoyer : Liste_actions, Liste_trajectoire, ordre_receive
# A recevoir : X_robot_actuel, Y_robot_actuel, Angle_robot_actuel, x_ennemi, y_ennemi, Batteries

# Configuration pour l'envoi
HOST_PC = "192.168.0.99"  # IP de l'ordinateur
PORT_ENVOI = 5000

# Configuration pour la réception
HOST_RPI = '0.0.0.0'  # Écoute sur toutes les interfaces
PORT_RECEPTION = 5001

Reel = False

Liste_actions = []
Liste_trajectoire = []

Liste_GM_libres = [1,2,3,4,5,6,7,8,9,10]
Liste_GM_occuper = []
Liste_noisettes_libres = [1,2,3,4,5,6,7,8]
Liste_noisettes_prises = []

Liste_GM_libres_old = Liste_GM_libres.copy()
Liste_noisettes_libres_old = Liste_noisettes_libres.copy()

Liste_zones_recup_noisettes_xy = [(400,1200),
                                  (400,400),
                                  (2600,1200),
                                  (2600,400),
                                  ((1150,1025),(1150,575)),
                                  ((1850,1025),(1850,575)),
                                  (1100,400),
                                  (1900,400)]

Liste_zones_recup_noisettes_angle = [180,
                                     180,
                                     0,
                                     0,
                                     (-90,90),
                                     (-90,90),
                                     -90,
                                     -90]
Liste_zones_gm_xy = [(1250,1200),
                     (1750,1200),
                     (350,800),
                     ((800,1050),(800,550),(550,800),(1050,800)),
                     ((1500,1050),(1500,550),(1250,800),(1750,800)),
                     ((2200,1050),(2200,550),(1950,800),(2450,800)),
                     (2650,800),
                     (700,350),
                     (1500,350),
                     (2300,350)]

Liste_zones_gm_angle = [90,
                        90,
                        180,
                        (-90,90,0,180),
                        (-90,90,0,180),
                        (-90,90,0,180),
                        0,
                        -90,
                        -90,
                        -90]

# Piste
X_PISTE = 3000
Y_PISTE = 2000
MARGE_BORDUREPISTE_X = 120 # Détection Lidar
MARGE_BORDUREPISTE_Y = 80 # Détection Lidar

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
x_ennemi = -1
y_ennemi = -1
x_ennemi_old = x_ennemi
y_ennemi_old = y_ennemi
########

# Perimètre de sécurité
R_ROBOT = 130
R_ENNEMI = 150
MARGE_MIN = 100
R_securite = R_ROBOT + R_ENNEMI + MARGE_MIN  # = 400mm
MARGE_SECURITE_ENNEMI = 5  # Marge pour l'affichage de la zone ennemi

# rayon_total_case : Pour les zones interdites STATIQUES (robot + marge réduite)

EPS_EQ = 5
CASE_MM = 10
width = X_PISTE//CASE_MM
height = Y_PISTE//CASE_MM
MARGE_OBSTACLES_MM = 0  # Marge standard pour les obstacles fixes
MARGE_NOISETTES_MM = 0  # Marge réduite pour les Noisettes (plus précis)
rayon_total_case = (R_ROBOT + MARGE_OBSTACLES_MM) // CASE_MM  # = 20 cases = 200mm

# === CRÉATION DES OBSTACLES avec la classe Obstacles ===
obs_manager = Obstacles(X_PISTE,Y_PISTE,R_ROBOT,MARGE_OBSTACLES_MM,CASE_MM)
obs_manager_noisettes = Obstacles(X_PISTE,Y_PISTE,R_ROBOT,MARGE_NOISETTES_MM,CASE_MM)

zones_centres = [
    (1250, 1450), (1750, 1450),           # 2 zones centrales
    (100, 800), (800, 800), (1500, 800), (2200, 800), (2900, 800),  # 5 zones ligne médiane
    (700, 100), (1500, 100), (2300, 100),  # 3 zones ligne basse
]

for i, (x, y) in enumerate(zones_centres, 1):
    obs_manager.ajouter_carre(f"zone{i}", (x, y), 200, actif=False)  # INACTIVES au départ

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

for i, coords in enumerate(zones_noisettes, 1):
    coin_bg = coords[0]  # Premier coin (bas-gauche)
    coin_hd = coords[1]  # Deuxième coin (haut-droit)
    obs_manager_noisettes.ajouter_rectangle(f"Noisette{i}", coin_bg, coin_hd, actif=True)  # ACTIVES au départ

obs_manager.ajouter_rectangle("grenier", (600, 1550), (2400, 2000), actif=True)

# Générer les grilles séparément
grid_zones, grid_zones_expanded, obstacle_array_zones, expanded_array_zones = obs_manager.generer_grille()
grid_noisettes, grid_noisettes_expanded, obstacle_array_noisettes, expanded_array_noisettes = obs_manager_noisettes.generer_grille()

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
robot_a_objets = False

# ==================== PARAMÈTRES DE L'ALGORITHME ====================
# Poids de la moyenne pondérée (somme = 1.0)
W_DISTANCE = 0.65     # Importance de la proximité de l'objectif
W_SECURITE = 0.2     # Importance de l'évitement du robot ennemi
W_PRIORITE = 0.1     # Importance du type de tâche (récolte vs dépôt)
W_EFFICACITE = 0.05   # Importance de la cohérence (robot a objets → déposer)

# Distance max sur le terrain (pour normalisation)
DISTANCE_MAX_TERRAIN = 3500  # Diagonale du terrain ≈ 3605mm

donnees_pour_robot = {
    "Liste_actions": Liste_actions,
    "Liste_trajectoire": Liste_trajectoire,
    "Ordre_receive": ordre_receive
}

message = json.dumps(donnees_pour_robot)

################## Fonction  ###########################################

# Fonction pour recevoir des données (OPTIMISÉE)
def recevoir_donnees(stop_event):
    global x_robot_actuel,y_robot_actuel,angle_robot_actuel,x_ennemi,y_ennemi,Batteries
    print(f"[Récepteur] Serveur en attente sur le port {PORT_RECEPTION}...")
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.settimeout(0.1)  # ✅ OPTIMISATION : Timeout pour éviter blocage
    server_socket.bind((HOST_RPI, PORT_RECEPTION))
    server_socket.listen(1)
    
    try:
        while not stop_event.is_set():
            try:
                conn, addr = server_socket.accept()
                conn.settimeout(0.1)  # ✅ OPTIMISATION : Timeout pour recv
                print(f"\n[Récepteur] --- Connexion depuis {addr} ---")
                
                # Réception des données
                data = b""
                while True:
                    try:
                        packet = conn.recv(1024)
                        if not packet:
                            break
                        data += packet
                    except socket.timeout:
                        break
                
                conn.close()
                
                # Décodage et affichage
                if data:
                    try:
                        donnees_recues = json.loads(data.decode())
                        
                        print("[Récepteur] Données reçues depuis le robot")
                        
                        x_robot_actuel = donnees_recues["x_robot_actuel"]
                        y_robot_actuel = donnees_recues["y_robot_actuel"]
                        angle_robot_actuel = donnees_recues["angle_robot_actuel"]
                        x_ennemi = donnees_recues["x_ennemi"]
                        y_ennemi = donnees_recues["y_ennemi"]
                        Batteries = donnees_recues["Batteries"]
                        
                    except json.JSONDecodeError:
                        print("[Récepteur] Erreur : données JSON invalides")
            except socket.timeout:
                # Normal, on continue
                pass
            
    except KeyboardInterrupt:
        print("[Récepteur] Arrêt.")
    finally:
        server_socket.close()

def arret_programme(event, stop_event=None):
    print("Bouton STOP pressé — arrêt demandé.")
    if stop_event is not None:
        stop_event.set()
        bring_to_front(fig)

def on_click(event):
    """
    Callback pour les clics sur la piste.
    
    COMPORTEMENT :
    - Clic GAUCHE (bouton 1) : Ajoute une consigne PRIORITAIRE en début de Liste_actions
    - Clic DROIT (bouton 3) : Affiche juste le point voulu (ancien comportement)
    """
    global x_robot_voulu, y_robot_voulu, Liste_actions
    
    if event.inaxes == ax:  # Clic dans la zone du graphique
        x_clic = event.xdata
        y_clic = event.ydata
        
        # Mettre à jour l'affichage du point voulu
        x_robot_voulu = x_clic
        y_robot_voulu = y_clic
        point_voulu_plot.set_data([round(x_robot_voulu, 0)], [round(y_robot_voulu, 0)])
        
        # ⭐ CLIC GAUCHE : Ajouter une consigne PRIORITAIRE ⭐
        if event.button == 1:  # Bouton gauche
            # Vider la liste (modifie la liste globale, pas une copie locale)
            Liste_actions.clear()
            # Ajouter la consigne cliquée
            Liste_actions.append(["Consigne", round(int(x_clic), 0), round(int(y_clic), 0)])
        
        # ⭐ CLIC DROIT : Juste afficher (pas d'ajout) ⭐
        elif event.button == 3:  # Bouton droit
            print(f"🖱️  Point affiché : ({round(x_clic, 0)}, {round(y_clic, 0)}) mm")
        
        # ✅ OPTIMISATION : draw_idle au lieu de draw
        fig.canvas.draw_idle()
        bring_to_front(fig)


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
        
    elif zone_num in Liste_GM_occuper:
        # Zone occupée → la rendre libre
        Liste_GM_occuper.remove(zone_num)
        Liste_GM_libres.append(zone_num)
    
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
        
    elif noisette_num in Liste_noisettes_prises:
        # Noisette prise → la remettre libre
        Liste_noisettes_prises.remove(noisette_num)
        Liste_noisettes_libres.append(noisette_num)

def bouton_attraper_callback(event):
    """
    Callback pour le bouton Attraper.
    Modifie ordre_receive à 21 si l action en cours est "Attraper".
    """
    global ordre_receive, action_voulu
    
    # Vérifier qu il y a une action en cours
    if len(Liste_actions) > 0 and action_voulu == "Attraper":
        ordre_receive = 21

def bouton_relacher_callback(event):
    """
    Callback pour le bouton Relacher.
    Modifie ordre_receive à 22 si l action en cours est "Relacher".
    """
    global ordre_receive, action_voulu
    
    # Vérifier qu il y a une action en cours
    if len(Liste_actions) > 0 and action_voulu == "Relacher":
        ordre_receive = 22   

########################################################################

################## Lancement du programme principal ####################

if __name__ == '__main__':

    stop_event = threading.Event()

    # ✅ OPTIMISATION : Activer mode interactif matplotlib
    plt.ion()

    fig, ax, robot_plot, ennemi_plot, consigne_plot, scat, robot_info_text,ax_button,bouton_stop,point_voulu_plot,x_voulu_text,y_voulu_text,A_voulu_text, robot_angle_line,robot_angle_voulu_line = init_affichage()
    cid = fig.canvas.mpl_connect('button_press_event', on_click) # Choix des coordonnées voulues avec la souris
    bouton_stop.on_clicked(partial(arret_programme, stop_event=stop_event))
    
    obstacle_scatter, expanded_scatter = afficher_obstacles(ax,obstacle_array,expanded_array,CASE_MM,show_expanded=True,show_obstacles=False)
    zone_ennemi_scatter, cercle_ennemi_patch = afficher_zone_securite_ennemi(ax,x_ennemi, y_ennemi,R_ROBOT, R_ENNEMI,MARGE_SECURITE_ENNEMI,CASE_MM,X_PISTE,Y_PISTE,show_zone=True,show_cercle=False)
    initialiser_grille(grid_expanded, distance_map, width, height, CASE_MM)
    grid, grid_expanded, obstacle_array, expanded_array, obs_manager, obs_manager_noisettes, Liste_GM_libres, Liste_GM_occuper, Liste_noisettes_libres, Liste_noisettes_prises, obstacle_scatter, expanded_scatter, distance_map, ax, width, height, CASE_MM = actualiser_zones_jeu(grid, grid_expanded, obstacle_array, expanded_array, obs_manager, obs_manager_noisettes, Liste_GM_libres, Liste_GM_occuper, Liste_noisettes_libres, Liste_noisettes_prises, obstacle_scatter, expanded_scatter, distance_map, ax, width, height, CASE_MM)
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

    thread_reception = threading.Thread(target=recevoir_donnees, args=(stop_event,), daemon=True)
    thread_reception.start()

    try:
        # Initialisation des variables de trajectoire
        path_plot = None
        path_simplified_plot = None
        path_smooth_plot = None
        
        # ✅ OPTIMISATION : Compteur pour limiter l'affichage
        compteur_affichage = 0
        
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
            DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE,
            verbose=True
        )
 
        while (not stop_event.is_set() and Batteries[2][3] > 5): # ✅ CORRECTION : Retirer len(Liste_actions)!=0
            temps = 0
            verif = False
            step +=1
            
            # ✅ OPTIMISATION : Initialiser Liste_trajectoire
            Liste_trajectoire = [0]
            
            if Liste_GM_libres != Liste_GM_libres_old or Liste_noisettes_libres != Liste_noisettes_libres_old:
                print("🔄 Mise à jour des zones...")
                grid, grid_expanded, obstacle_array, expanded_array, obs_manager, obs_manager_noisettes, Liste_GM_libres, Liste_GM_occuper, Liste_noisettes_libres, Liste_noisettes_prises, obstacle_scatter, expanded_scatter, distance_map, ax, width, height, CASE_MM = actualiser_zones_jeu(grid, grid_expanded, obstacle_array, expanded_array, obs_manager, obs_manager_noisettes, Liste_GM_libres, Liste_GM_occuper, Liste_noisettes_libres, Liste_noisettes_prises, obstacle_scatter, expanded_scatter, distance_map, ax, width, height, CASE_MM)
                Liste_GM_libres_old = Liste_GM_libres.copy()
                Liste_noisettes_libres_old = Liste_noisettes_libres.copy()
            # ⭐ NOUVELLE DÉCISION si liste vide ⭐
            if len(Liste_actions) == 0:
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
                    DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE,
                    verbose=True
                )
                
                if len(Liste_actions) == 0:
                    print("⏳ Aucune action disponible, attente...")
                    time.sleep(0.1)
                    continue
            
            if not Reel: 
                x_ennemi += 6
                y_ennemi -= 4
                if(step == 130):
                    x_ennemi = 1500
                    y_ennemi = 450
                if(step==700):
                    x_ennemi = 1000
                    y_ennemi = 1000

            # Lire l'action courante
            if type(Liste_actions[0]) == list and len(Liste_actions[0])==3:
                action_voulu = Liste_actions[0][0]
                x_robot_voulu = Liste_actions[0][1]
                y_robot_voulu = Liste_actions[0][2]
                verif = True
                mouvement = True
            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==2:
                action_voulu = Liste_actions[0][0]
                angle_robot_voulu = round(Liste_actions[0][1],0)
                mouvement = True
            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==1:
                print("Appeler Carte Actionneur pour : " + Liste_actions[0][0])
                action_voulu = Liste_actions[0][0]
                mouvement = False
            
            # ⭐ RÉÉVALUATION : Vérifier si la cible est toujours disponible ⭐
            action_en_cours = verifier_et_changer_cible_si_necessaire(
                action_en_cours,
                Liste_actions,
                Liste_noisettes_libres,
                Liste_GM_libres,
                x_robot_actuel, y_robot_actuel,
                x_ennemi, y_ennemi,
                robot_a_objets,
                Liste_zones_recup_noisettes_xy,
                Liste_zones_recup_noisettes_angle,
                Liste_zones_gm_xy,
                Liste_zones_gm_angle,
                R_securite,
                action_voulu,
                DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE
            )
            
            # Si la cible a changé, relire Liste_actions[0]
            if len(Liste_actions) > 0:
                if type(Liste_actions[0]) == list and len(Liste_actions[0])==3:
                    action_voulu = Liste_actions[0][0]
                    x_robot_voulu = Liste_actions[0][1]
                    y_robot_voulu = Liste_actions[0][2]
                    verif = True
                    mouvement = True
                elif type(Liste_actions[0]) == list and len(Liste_actions[0])==2:
                    action_voulu = Liste_actions[0][0]
                    angle_robot_voulu = round(Liste_actions[0][1],0)
                    mouvement = True
                elif type(Liste_actions[0]) == list and len(Liste_actions[0])==1:
                    action_voulu = Liste_actions[0][0]
                    mouvement = False
                 
            print(step)
            print("\nAvant verif :")
            print("Action en cours : "+action_voulu)
            print(f"X_actuel = {x_robot_actuel} Y_actuel = {y_robot_actuel} Angle_actuel = {angle_robot_actuel}°")
            print(f"X_voulu = {x_robot_voulu} Y_voulu = {y_robot_voulu} Angle_voulu = {angle_robot_voulu}°")

            distance_robot_ennemi = math.sqrt((x_ennemi - x_robot_actuel)**2 + (y_ennemi - y_robot_actuel)**2)
            angle_ennemi = np.degrees(math.atan2(y_ennemi-y_ennemi_old,x_ennemi-x_ennemi_old))
            
            # ✅ OPTIMISATION 3 : Ne recalculer A* que si nécessaire
            delta_x_ennemi = abs(x_ennemi - x_ennemi_old)
            delta_y_ennemi = abs(y_ennemi - y_ennemi_old)
            ennemi_a_bouge = (delta_x_ennemi > 50 or delta_y_ennemi > 50)  # Seuil 50mm
             
            # === CALCUL DE LA TRAJECTOIRE A* === 
            if action_voulu in ["Rotation", "Tourner","Attraper","Relacher"]:
                pass  # rien, on attend la fin de la rotation
            else:
                # ✅ OPTIMISATION : Ne recalculer que si ennemi a significativement bougé
                if action_voulu in ["Consigne",'Avancer'] and (ennemi_a_bouge or step == 1):
                    Liste_actions = [action for action in Liste_actions if action[0] in ["Consigne","Rotation","Attraper","Relacher"]]

                    # Convertir les positions en cases
                    start_case = (int(x_robot_actuel // CASE_MM), int(y_robot_actuel // CASE_MM))
                    goal_case = (int(x_robot_voulu // CASE_MM), int(y_robot_voulu // CASE_MM))
                    
                    print(f"🗺️  Calcul trajectoire A* : {start_case} → {goal_case}")
                    
                    # Calculer le chemin avec A*
                    print(f"🎯 Position ennemi : ({x_ennemi}, {y_ennemi})")
                    grid_avec_ennemi = creer_grille_avec_ennemi(
                        grid_expanded, x_ennemi, y_ennemi, 
                        R_ROBOT, R_ENNEMI, MARGE_MIN, CASE_MM
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
                        path_length_mm = path_length * CASE_MM
                        print(f"📏 Longueur trajectoire : {path_length_mm:.0f} mm")
                        
                        # Afficher le chemin brut (optionnel, décommenter si souhaité)
                        px, py = zip(*path)
                        if path_plot is not None:
                            path_plot.remove()
                        path_plot, = ax.plot(px, py, 'b-', linewidth=1, alpha=0.3, label='A* brut')
                        
                        # Afficher le chemin simplifié
                        px, py = zip(*path_simplified)
                        # Convertir en millimètres pour l'affichage
                        px_mm = [x * CASE_MM for x in px]
                        py_mm = [y * CASE_MM for y in py]
                        # Insérer actions avec rotation vers chaque point
                        print(f"📍 Création de {len(px_mm)} waypoints avec rotations")
                        
                        # Effacer anciens plots juste avant création nouveaux
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

                        # Construire Liste_trajectoire
                        if nbr_point >= 1:
                            Liste_trajectoire = []
                            Liste_trajectoire.append(nbr_point)
                            for action in Liste_actions:
                                if action[0] == "Avancer" and len(action) == 3:
                                    Liste_trajectoire.append([action[1], action[2]])

                        print(Liste_trajectoire)
                        
                        if path_simplified_plot is not None:
                            path_simplified_plot.remove()
                        path_simplified_plot, = ax.plot(px_mm, py_mm, 'g-', linewidth=2,label='Chemin A*', marker='o', markersize=4, zorder=10)
    
                    else:
                        print("❌ Aucun chemin trouvé par A*")
                        if distance_robot_ennemi < R_securite :
                            print("BESOIN DE RECULER")
                            ratio = R_securite / distance_robot_ennemi
                            x_recul = round(x_ennemi - (x_ennemi-x_robot_actuel) * ratio,0)
                            y_recul = round(y_ennemi - (y_ennemi-y_robot_actuel) * ratio,0)
                            x_recul = clamp(x_recul, R_ROBOT, X_PISTE-R_ROBOT)
                            y_recul = clamp(y_recul, R_ROBOT, Y_PISTE-R_ROBOT)
                            Liste_actions = [action for action in Liste_actions if(action[0] in ["Consigne","Rotation"])]
                            Liste_actions.insert(0,["Avancer",int(x_recul),int(y_recul)])
                            Liste_actions.insert(0,["Recul",int(x_recul),int(y_recul)])
                            angle_prochain_point = calcul_angle_vers_point(
                                x_robot_actuel, y_robot_actuel,
                                Liste_actions[0][1], Liste_actions[0][2]
                            )
                            if(angle_prochain_point != angle_robot_actuel):
                                Liste_actions.insert(0, ["Rotation", angle_prochain_point])

                        else :
                            print("CHEMIN INACCESSIBLE, RECALCUL DE TRAJECTOIRE")
                            x_robot_voulu = x_robot_actuel
                            y_robot_voulu = y_robot_actuel
            
            
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
                print("Appeler Carte Actionneur pour : " + Liste_actions[0][0])
                action_voulu = Liste_actions[0][0]
                mouvement = False

            verif = False
            # ✅ Ne PAS recalculer pendant rotation (laisser tourner)
            if action_voulu in ["Consigne","Avancer"]  or ennemi_a_bouge:
                # Calculer distance à l'ennemi
                dx_ennemi = x_ennemi - x_robot_actuel
                dy_ennemi = y_ennemi - y_robot_actuel
                distance_ennemi = math.sqrt(dx_ennemi**2 + dy_ennemi**2)
                
                # ✅ Vérifier si waypoints futurs bloqués par ennemi
                trajectoire_bloquee = False
                
                robot_proche_ennemi = distance_ennemi < R_securite
                
                # ✅ Recalculer si ennemi bouge OU si robot se rapproche
                if ennemi_a_bouge or robot_proche_ennemi:
                    if ennemi_a_bouge:
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
                                break
                
                # Si ennemi dans zone critique ( rayon sécurité)
                if distance_ennemi < (R_securite) or trajectoire_bloquee:
                    print(f"⚠️  ENNEMI DÉTECTÉ à {distance_ennemi:.0f}mm ! Recalcul trajectoire...")
                    
                    # Nettoyer waypoints intermédiaires
                    Liste_actions = [act for act in Liste_actions if act[0] in ["Consigne", "Rotation", "Attraper", "Relacher"]]
                    
                    # Préparer recalcul
                    x_robot_voulu = Liste_actions[0][1]
                    y_robot_voulu = Liste_actions[0][2]
                    
                    start_case = (int(x_robot_actuel // CASE_MM), int(y_robot_actuel // CASE_MM))
                    goal_case = (int(x_robot_voulu // CASE_MM), int(y_robot_voulu // CASE_MM))
                    
                    print(f"🗺️  RECALCUL A* : {start_case} → {goal_case}")
                    print(f"🎯 Position ennemi : ({x_ennemi}, {y_ennemi}) mm")
                    
                    # Créer grille avec ennemi
                    grid_avec_ennemi = creer_grille_avec_ennemi(
                        grid_expanded, x_ennemi, y_ennemi,
                        R_ROBOT, R_ENNEMI, MARGE_MIN, CASE_MM
                    )
                    
                    # Calculer nouveau chemin
                    path = astar_safe(start_case, goal_case, safety_weight=1.5,grid_dynamique=grid_avec_ennemi)
                    
                    if path and len(path) > 1:
                        path_simplified = simplify_path_safe(path, min_clearance=3)
                        print(f"✅ NOUVELLE trajectoire : {len(path)} → {len(path_simplified)} points")
                        
                        # Convertir en mm
                        px, py = zip(*path_simplified)
                        px_mm = [x * CASE_MM for x in px]
                        py_mm = [y * CASE_MM for y in py]
                        
                        # Effacer anciens plots
                        if path_plot is not None:
                            path_plot.remove()
                            path_plot = None
                        if path_simplified_plot is not None:
                            path_simplified_plot.remove()
                            path_simplified_plot = None
                        
                        for i in range(len(py_mm)-1, 0, -1):
                            x_cible = px_mm[i]
                            y_cible = py_mm[i]
                            
                            Liste_actions.insert(0, ["Avancer", x_cible, y_cible])
                            print(f"  → Nouveau waypoint: Avancer vers ({x_cible:.0f}, {y_cible:.0f})")
                        
                        # Afficher nouveau chemin
                        path_simplified_plot, = ax.plot(px_mm, py_mm, 'g-', linewidth=2,label='Nouveau chemin', marker='o', markersize=4, zorder=10)
               
            print("\nAprès verif :")
            print("Action en cours : "+action_voulu)
            print(f"X_actuel = {x_robot_actuel} Y_actuel = {y_robot_actuel} Angle_actuel = {angle_robot_actuel}°")
            print(f"X_voulu = {x_robot_voulu} Y_voulu = {y_robot_voulu} Angle_voulu = {angle_robot_voulu}°")
            print(Liste_actions)


            # Mettre à jour robot, ennemi et consigne sur affichage
            robot_plot.set_offsets([[x_robot_actuel, y_robot_actuel]])
            
            # 🔄 Mettre à jour la zone de sécurité dynamique de l'ennemi
            zone_ennemi_scatter, cercle_ennemi_patch = mettre_a_jour_zone_ennemi(
                zone_ennemi_scatter, cercle_ennemi_patch,
                x_ennemi, y_ennemi, R_ROBOT, R_ENNEMI,
                MARGE_SECURITE_ENNEMI, CASE_MM,
                X_PISTE, Y_PISTE
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
            
            Ordre_Batteries = gerer_basculement_batteries(Batteries, U_last, Ordre_Batteries)
            battery_patches, battery_texts = afficher_batteries(ax, Batteries, Ordre_Batteries,battery_patches, battery_texts,couleurs, seuils,largeur_rect, hauteur_rect, espacement, espacement_salves,y_base, texte_offset_y)
            
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
                if(abs(angle_robot_actuel-angle_robot_voulu)<5) and action_voulu in ["Rotation","Tourner"]:
                    print("Bon Angle")
                    ordre_receive = 12
                    
            if (action_voulu in ["Consigne","Avancer"] and ordre_receive == 11) or \
               (action_voulu in ["Rotation"] and ordre_receive == 12) or \
               (action_voulu == "Attraper" and ordre_receive == 21) or \
               (action_voulu == "Relacher" and ordre_receive == 22):
                
                # Retirer l'action de la liste 
                Liste_actions.pop(0) 
                
                # ⭐ SI c'est une action ATTRAPER ou RELACHER ⭐
                if action_voulu in ["Attraper", "Relacher"]:
                    # Confirmer l'action
                    robot_a_objets = confirmer_action_terminee(
                        action_en_cours,
                        robot_a_objets,
                        Liste_noisettes_libres, Liste_noisettes_prises,
                        Liste_GM_libres, Liste_GM_occuper,
                        verbose=True
                    )
                    
                    # ⭐ VIDER TOUTE LA LISTE car l'objectif change ⭐
                    print(f"🔄 Vidage de la liste d'actions (ancien objectif terminé)")
                    Liste_actions.clear()
                    
                    # ⭐ FORCER une nouvelle décision immédiatement ⭐
                    print(f"🎯 Prise de nouvelle décision (nouvel objectif)")
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
                        DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE,
                        verbose=True
                    )
                    
                    if len(Liste_actions) == 0:
                        print("⚠️  AUCUNE ACTION DISPONIBLE - Mission terminée ou zones bloquées")
                
                ordre_receive = 0
                
                # ⭐ Pour les actions normales (Consigne/Rotation), nouvelle décision si liste vide ⭐
                if action_voulu not in ["Attraper", "Relacher"] and len(Liste_actions) == 0:
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
                        DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE,
                        verbose=True
                    )
                    
                    if len(Liste_actions) == 0:
                        print("⚠️  AUCUNE ACTION DISPONIBLE - Mission terminée ou zones bloquées")


            # ✅ OPTIMISATION 2 : Limiter les mises à jour d'affichage (1 fois sur 3)
            compteur_affichage += 1
            if compteur_affichage >= 3:
                fig.canvas.draw_idle()  # Plus rapide que draw()
                fig.canvas.flush_events()
                compteur_affichage = 0
            
            # ✅ OPTIMISATION 5 : Sleep plus long pour libérer le CPU
            time.sleep(0.01)  # 10ms au lieu de 1ms
            
            U_last = [Batteries[0][2],Batteries[1][2],Batteries[2][2]]
            x_robot_voulu_last = x_robot_voulu
            y_robot_voulu_last = y_robot_voulu
            x_ennemi_old = x_ennemi
            y_ennemi_old = y_ennemi
            
            # ✅ OPTIMISATION 1 : Envoi via socket persistant
            
            donnees_pour_robot = {
                "Liste_actions": Liste_actions,
                "Liste_trajectoire": Liste_trajectoire,
                "Ordre_receive": ordre_receive
            }
            message = json.dumps(donnees_pour_robot)
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((HOST_PC, PORT_ENVOI))
            client_socket.sendall(message.encode())
            client_socket.close()
            time.sleep(0.01)
            
        time.sleep(2)
        stop_event.set()

    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur.")
        stop_event.set()
        
    finally:
        print("Programme terminé proprement.")
        
        # ✅ Fermer la connexion socket
        if client_socket:
            try:
                client_socket.close()
                print("✅ Socket fermé")
            except:
                pass
        
        plt.close(fig)

########################################################################
