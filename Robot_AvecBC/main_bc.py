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
from queue import Queue
from affichage import init_affichage, bring_to_front,afficher_obstacles,afficher_zone_securite_ennemi, mettre_a_jour_zone_ennemi,afficher_batteries
from calcul_mouv import euclidienne,astar_safe,simplify_path_safe,smooth_path_safe,creer_grille_avec_ennemi,initialiser_grille
from fonction import Obstacles,gerer_basculement_batteries,clamp,calcul_angle_vers_point
from gestion_zones_dynamiques import actualiser_zones_jeu, verifier_et_changer_cible_si_necessaire, mise_a_jour_decision, confirmer_action_terminee
########################################################################

# Configuration réseau
HOST_PC = "192.168.0.99"
PORT_ENVOI = 5000
HOST_RPI = '0.0.0.0'
PORT_RECEPTION = 5001

Reel = False
DEBUG_MODE = False  # ✅ Mode debug pour logs

# ✅ QUEUES POUR COMMUNICATION INTER-THREADS
queue_calcul_astar = Queue()      # Thread principal → Thread calcul A*
queue_resultat_astar = Queue()    # Thread calcul A* → Thread principal
queue_calcul_decision = Queue()   # Thread principal → Thread décision
queue_resultat_decision = Queue() # Thread décision → Thread principal

Liste_actions = []
Liste_trajectoire = []

Liste_GM_libres = [1,2,3,4,5,6,7,8,9,10]
Liste_GM_occuper = []
Liste_noisettes_libres = [1,2,3,4,5,6,7,8]
Liste_noisettes_prises = []

Liste_GM_libres_old = Liste_GM_libres.copy()
Liste_noisettes_libres_old = Liste_noisettes_libres.copy()

Liste_zones_recup_noisettes_xy = [(400,1200),(400,400),(2600,1200),(2600,400),
                                  ((1150,1025),(1150,575)),((1850,1025),(1850,575)),
                                  (1100,400),(1900,400)]

Liste_zones_recup_noisettes_angle = [180,180,0,0,(-90,90),(-90,90),-90,-90]

Liste_zones_gm_xy = [(1250,1200),(1750,1200),(350,800),
                     ((800,1050),(800,550),(550,800),(1050,800)),
                     ((1500,1050),(1500,550),(1250,800),(1750,800)),
                     ((2200,1050),(2200,550),(1950,800),(2450,800)),
                     (2650,800),(700,350),(1500,350),(2300,350)]

Liste_zones_gm_angle = [90,90,180,(-90,90,0,180),(-90,90,0,180),(-90,90,0,180),0,-90,-90,-90]

# Piste
X_PISTE = 3000
Y_PISTE = 2000
MARGE_BORDUREPISTE_X = 120
MARGE_BORDUREPISTE_Y = 80

# Coordonnées robot
x_robot_depart = 350
y_robot_depart = 1800
angle_robot_depart = -90

x_robot_actuel = x_robot_depart
y_robot_actuel = y_robot_depart
angle_robot_actuel = angle_robot_depart

x_robot_voulu = -1
y_robot_voulu = -1
angle_robot_voulu = -181

# Ennemi
x_ennemi = -1
y_ennemi = -1
x_ennemi_old = x_ennemi
y_ennemi_old = y_ennemi

# Sécurité
R_ROBOT = 130
R_ENNEMI = 150
MARGE_MIN = 100
R_securite = R_ROBOT + R_ENNEMI + MARGE_MIN
MARGE_SECURITE_ENNEMI = 5

EPS_EQ = 5
CASE_MM = 10
width = X_PISTE//CASE_MM
height = Y_PISTE//CASE_MM
MARGE_OBSTACLES_MM = 0
MARGE_NOISETTES_MM = 0
rayon_total_case = (R_ROBOT + MARGE_OBSTACLES_MM) // CASE_MM

# Obstacles
obs_manager = Obstacles(X_PISTE,Y_PISTE,R_ROBOT,MARGE_OBSTACLES_MM,CASE_MM)
obs_manager_noisettes = Obstacles(X_PISTE,Y_PISTE,R_ROBOT,MARGE_NOISETTES_MM,CASE_MM)

zones_centres = [
    (1250, 1450), (1750, 1450),
    (100, 800), (800, 800), (1500, 800), (2200, 800), (2900, 800),
    (700, 100), (1500, 100), (2300, 100),
]

for i, (x, y) in enumerate(zones_centres, 1):
    obs_manager.ajouter_carre(f"zone{i}", (x, y), 200, actif=False)

zones_noisettes = [
    [(100, 1100), (250, 1300)], [(100, 300), (250, 500)],
    [(2750, 1100), (2900, 1300)], [(2750, 300), (2900, 500)],
    [(1050, 725), (1250, 875)], [(1750, 725), (1950, 875)],
    [(1000, 100), (1200, 250)], [(1800, 100), (2000, 250)]
]

for i, coords in enumerate(zones_noisettes, 1):
    obs_manager_noisettes.ajouter_rectangle(f"Noisette{i}", coords[0], coords[1], actif=True)

obs_manager.ajouter_rectangle("grenier", (600, 1550), (2400, 2000), actif=True)

grid_zones, grid_zones_expanded, obstacle_array_zones, expanded_array_zones = obs_manager.generer_grille()
grid_noisettes, grid_noisettes_expanded, obstacle_array_noisettes, expanded_array_noisettes = obs_manager_noisettes.generer_grille()

grid = np.logical_or(grid_zones, grid_noisettes).astype(int)
grid_expanded = np.logical_or(grid_zones_expanded, grid_noisettes_expanded).astype(int)

obstacle_array = np.vstack([obstacle_array_zones, obstacle_array_noisettes]) if len(obstacle_array_zones) > 0 and len(obstacle_array_noisettes) > 0 else (obstacle_array_zones if len(obstacle_array_zones) > 0 else obstacle_array_noisettes)
expanded_array = np.vstack([expanded_array_zones, expanded_array_noisettes]) if len(expanded_array_zones) > 0 and len(expanded_array_noisettes) > 0 else (expanded_array_zones if len(expanded_array_zones) > 0 else expanded_array_noisettes)

# Batteries
Batteries = [[12,14,14,100],[12,14,14,100],[12,14,14,100]]
U_last = [0,0,0]
Ordre_Batteries = [1,0,0]

battery_patches = []
battery_texts = [] 
couleurs = ['red', 'orange', 'yellow', 'lime', 'green']
seuils = [1, 20, 50, 75, 90]
largeur_rect = 50
hauteur_rect = 150
espacement = 0
espacement_salves = 200
y_base = 2050
marge_texte = 20
texte_offset_y = 80
longueur_trait = 100

fig = None 
ax = None
robot_plot = None
scat = None

zone_ennemi_scatter = None
cercle_ennemi_patch = None

obstacle_scatter = None
expanded_scatter = None
boutons_zones = []
boutons_noisettes = []

from scipy.ndimage import binary_dilation
structure = np.zeros((2*rayon_total_case+1, 2*rayon_total_case+1))
y_grid, x_grid = np.ogrid[-rayon_total_case:rayon_total_case+1, -rayon_total_case:rayon_total_case+1]
mask = x_grid*x_grid + y_grid*y_grid <= rayon_total_case*rayon_total_case
structure[mask] = 1
grid_expanded = binary_dilation(grid, structure=structure)

from scipy.ndimage import distance_transform_edt
distance_map = distance_transform_edt(~grid_expanded)

action_en_cours = None
trajectoire_bloquee = False
ordre_receive = 0
step = 0
robot_a_objets = False

W_DISTANCE = 0.65
W_SECURITE = 0.2
W_PRIORITE = 0.1
W_EFFICACITE = 0.05
DISTANCE_MAX_TERRAIN = 3500

donnees_pour_robot = {
    "Liste_actions": Liste_actions,
    "Liste_trajectoire": Liste_trajectoire,
    "Ordre_receive": ordre_receive
}

################## THREADS ###########################################

def thread_calcul_astar(stop_event):
    """
    🧵 Thread dédié aux calculs A* (pathfinding)
    Tourne en arrière-plan sans bloquer l'interface
    """
    global grid_expanded
    
    if DEBUG_MODE:
        print("🧵 [A*] Thread démarré")
    
    while not stop_event.is_set():
        try:
            if not queue_calcul_astar.empty():
                task = queue_calcul_astar.get(timeout=0.1)
                
                x_robot = task["x_robot"]
                y_robot = task["y_robot"]
                x_voulu = task["x_voulu"]
                y_voulu = task["y_voulu"]
                x_ennemi = task["x_ennemi"]
                y_ennemi = task["y_ennemi"]
                safety_weight = task.get("safety_weight", 1.0)
                
                start_case = (int(x_robot // CASE_MM), int(y_robot // CASE_MM))
                goal_case = (int(x_voulu // CASE_MM), int(y_voulu // CASE_MM))
                
                if DEBUG_MODE:
                    print(f"🗺️  [A*] Calcul : {start_case} → {goal_case}")
                
                grid_avec_ennemi = creer_grille_avec_ennemi(
                    grid_expanded, x_ennemi, y_ennemi, 
                    R_ROBOT, R_ENNEMI, MARGE_MIN, CASE_MM
                )
                
                # ⚡ CALCUL A* (100-500ms)
                path = astar_safe(start_case, goal_case, safety_weight, grid_dynamique=grid_avec_ennemi)
                
                if path:
                    path_simplified = simplify_path_safe(path, 3)
                    
                    px, py = zip(*path_simplified)
                    px_mm = [x * CASE_MM for x in px]
                    py_mm = [y * CASE_MM for y in py]
                    
                    waypoints = []
                    for i in range(len(py_mm)-1, 0, -1):
                        x_cible = round(px_mm[i])
                        y_cible = round(py_mm[i])
                        waypoints.append(["Avancer", x_cible, y_cible])
                    
                    queue_resultat_astar.put({
                        "success": True,
                        "path": path,
                        "path_simplified": path_simplified,
                        "waypoints": waypoints,
                        "px_mm": px_mm,
                        "py_mm": py_mm
                    })
                    
                    if DEBUG_MODE:
                        print(f"✅ [A*] Chemin trouvé : {len(waypoints)} waypoints")
                else:
                    queue_resultat_astar.put({
                        "success": False,
                        "reason": "no_path"
                    })
                    
                    if DEBUG_MODE:
                        print("❌ [A*] Aucun chemin")
            
            else:
                time.sleep(0.01)
                
        except Exception as e:
            print(f"❌ [A*] Erreur : {e}")
            queue_resultat_astar.put({
                "success": False,
                "reason": "error",
                "error": str(e)
            })
            time.sleep(0.1)
    
    if DEBUG_MODE:
        print("🧵 [A*] Thread arrêté")


def thread_calcul_decision(stop_event):
    """
    🧵 Thread dédié aux décisions stratégiques
    """
    global Liste_actions
    
    if DEBUG_MODE:
        print("🧵 [Décision] Thread démarré")
    
    while not stop_event.is_set():
        try:
            if not queue_calcul_decision.empty():
                task = queue_calcul_decision.get(timeout=0.1)
                
                if DEBUG_MODE:
                    print("🎯 [Décision] Calcul...")
                
                # ⚡ CALCUL DÉCISION (50-200ms)
                action_en_cours_local = mise_a_jour_decision(
                    Liste_actions,
                    task["x_robot"], task["y_robot"],
                    task["x_ennemi"], task["y_ennemi"],
                    task["robot_a_objets"],
                    task["Liste_noisettes_libres"],
                    task["Liste_GM_libres"],
                    Liste_zones_recup_noisettes_xy,
                    Liste_zones_recup_noisettes_angle,
                    Liste_zones_gm_xy,
                    Liste_zones_gm_angle,
                    R_securite,
                    DISTANCE_MAX_TERRAIN, W_DISTANCE, W_SECURITE, W_PRIORITE, W_EFFICACITE,
                    verbose=False
                )
                
                queue_resultat_decision.put({
                    "action_en_cours": action_en_cours_local
                })
                
                if DEBUG_MODE:
                    print(f"✅ [Décision] Action : {action_en_cours_local}")
            
            else:
                time.sleep(0.01)
                
        except Exception as e:
            print(f"❌ [Décision] Erreur : {e}")
            time.sleep(0.1)
    
    if DEBUG_MODE:
        print("🧵 [Décision] Thread arrêté")


def recevoir_donnees(stop_event):
    """Thread réception données robot"""
    global x_robot_actuel, y_robot_actuel, angle_robot_actuel, x_ennemi, y_ennemi, Batteries
    
    if DEBUG_MODE:
        print(f"🧵 [Réception] En attente sur port {PORT_RECEPTION}")
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.settimeout(0.1)
    server_socket.bind((HOST_RPI, PORT_RECEPTION))
    server_socket.listen(1)
    
    try:
        while not stop_event.is_set():
            try:
                conn, addr = server_socket.accept()
                conn.settimeout(0.1)
                
                if DEBUG_MODE:
                    print(f"[Réception] Connexion depuis {addr}")
                
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
                
                if data:
                    try:
                        donnees_recues = json.loads(data.decode())
                        
                        x_robot_actuel = donnees_recues["x_robot_actuel"]
                        y_robot_actuel = donnees_recues["y_robot_actuel"]
                        angle_robot_actuel = donnees_recues["angle_robot_actuel"]
                        x_ennemi = donnees_recues["x_ennemi"]
                        y_ennemi = donnees_recues["y_ennemi"]
                        Batteries = donnees_recues["Batteries"]
                        
                    except (json.JSONDecodeError, KeyError) as e:
                        if DEBUG_MODE:
                            print(f"[Réception] Erreur : {e}")
            
            except socket.timeout:
                pass
            
    except KeyboardInterrupt:
        if DEBUG_MODE:
            print("[Réception] Arrêt")
    finally:
        server_socket.close()


################## CALLBACKS INTERFACE ###############################

def arret_programme(event, stop_event=None):
    print("Bouton STOP pressé")
    if stop_event is not None:
        stop_event.set()
        bring_to_front(fig)

def on_click(event):
    global x_robot_voulu, y_robot_voulu, Liste_actions
    
    if event.inaxes == ax:
        x_clic = event.xdata
        y_clic = event.ydata
        
        x_robot_voulu = x_clic
        y_robot_voulu = y_clic
        point_voulu_plot.set_data([round(x_robot_voulu, 0)], [round(y_robot_voulu, 0)])
        
        if event.button == 1:  # Clic gauche
            Liste_actions.clear()
            Liste_actions.append(["Consigne", round(int(x_clic), 0), round(int(y_clic), 0)])
            print(f"✅ Consigne : ({round(x_clic, 0)}, {round(y_clic, 0)})")
        
        elif event.button == 3:
            if DEBUG_MODE:
                print(f"🖱️  Point : ({round(x_clic, 0)}, {round(y_clic, 0)})")
        
        fig.canvas.draw_idle()
        bring_to_front(fig)

def toggle_zone(event, zone_num):
    global Liste_GM_libres, Liste_GM_occuper
    
    if zone_num in Liste_GM_libres:
        Liste_GM_libres.remove(zone_num)
        Liste_GM_occuper.append(zone_num)
    elif zone_num in Liste_GM_occuper:
        Liste_GM_occuper.remove(zone_num)
        Liste_GM_libres.append(zone_num)

def toggle_noisette(event, noisette_num):
    global Liste_noisettes_libres, Liste_noisettes_prises
    
    if noisette_num in Liste_noisettes_libres:
        Liste_noisettes_libres.remove(noisette_num)
        Liste_noisettes_prises.append(noisette_num)
    elif noisette_num in Liste_noisettes_prises:
        Liste_noisettes_prises.remove(noisette_num)
        Liste_noisettes_libres.append(noisette_num)

def bouton_attraper_callback(event):
    global ordre_receive
    if len(Liste_actions) > 0:
        ordre_receive = 21

def bouton_relacher_callback(event):
    global ordre_receive
    if len(Liste_actions) > 0:
        ordre_receive = 22

################## PROGRAMME PRINCIPAL ################################

if __name__ == '__main__':
    stop_event = threading.Event()

    plt.ion()

    fig, ax, robot_plot, ennemi_plot, consigne_plot, scat, robot_info_text, ax_button, bouton_stop, point_voulu_plot, x_voulu_text, y_voulu_text, A_voulu_text, robot_angle_line, robot_angle_voulu_line = init_affichage()
    
    cid = fig.canvas.mpl_connect('button_press_event', on_click)
    bouton_stop.on_clicked(partial(arret_programme, stop_event=stop_event))
    
    obstacle_scatter, expanded_scatter = afficher_obstacles(ax, obstacle_array, expanded_array, CASE_MM, show_expanded=True, show_obstacles=False)
    zone_ennemi_scatter, cercle_ennemi_patch = afficher_zone_securite_ennemi(ax, x_ennemi, y_ennemi, R_ROBOT, R_ENNEMI, MARGE_SECURITE_ENNEMI, CASE_MM, X_PISTE, Y_PISTE, show_zone=True, show_cercle=False)
    
    initialiser_grille(grid_expanded, distance_map, width, height, CASE_MM)
    grid, grid_expanded, obstacle_array, expanded_array, obs_manager, obs_manager_noisettes, Liste_GM_libres, Liste_GM_occuper, Liste_noisettes_libres, Liste_noisettes_prises, obstacle_scatter, expanded_scatter, distance_map, ax, width, height, CASE_MM = actualiser_zones_jeu(grid, grid_expanded, obstacle_array, expanded_array, obs_manager, obs_manager_noisettes, Liste_GM_libres, Liste_GM_occuper, Liste_noisettes_libres, Liste_noisettes_prises, obstacle_scatter, expanded_scatter, distance_map, ax, width, height, CASE_MM)
    
    ax.add_line(robot_angle_line)
    ax.add_line(robot_angle_voulu_line)

    # Boutons zones
    for i in range(1, 11):
        ax_zone_button = plt.axes([0.87, 0.88 - (i-1)*0.08, 0.12, 0.08])
        zone_button = Button(ax_zone_button, f'Zone {i}\n✗ INACTIF', color='lightgreen', hovercolor='green')
        zone_button.on_clicked(partial(toggle_zone, zone_num=i))
        boutons_zones.append(zone_button)
    
    # Boutons noisettes
    for i in range(1, 9):
        ax_noisette_button = plt.axes([0.15 + (i-1)*0.09, 0.02, 0.08, 0.06])
        noisette_button = Button(ax_noisette_button, f'N{i}\n✓ ACTIF', color='lightcoral', hovercolor='red')
        noisette_button.on_clicked(partial(toggle_noisette, noisette_num=i))
        boutons_noisettes.append(noisette_button)

    # Boutons actions
    ax_attraper_button = plt.axes([0.38, 0.92, 0.12, 0.08])
    bouton_attraper = Button(ax_attraper_button, "🤖 ATTRAPER", color="lightblue", hovercolor="blue")
    bouton_attraper.on_clicked(bouton_attraper_callback)
    
    ax_relacher_button = plt.axes([0.52, 0.92, 0.12, 0.08])
    bouton_relacher = Button(ax_relacher_button, "🤖 RELACHER", color="lightyellow", hovercolor="orange")
    bouton_relacher.on_clicked(bouton_relacher_callback)

    # ✅ DÉMARRER LES THREADS
    thread_astar = threading.Thread(target=thread_calcul_astar, args=(stop_event,), daemon=True)
    thread_decision = threading.Thread(target=thread_calcul_decision, args=(stop_event,), daemon=True)
    thread_reception = threading.Thread(target=recevoir_donnees, args=(stop_event,), daemon=True)
    
    thread_astar.start()
    thread_decision.start()
    thread_reception.start()
    print("✅ Threads calcul démarrés")

    # ✅ Socket persistant
    client_socket = None
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((HOST_PC, PORT_ENVOI))
        print(f"✅ Socket connecté à {HOST_PC}:{PORT_ENVOI}")
    except Exception as e:
        print(f"❌ Erreur socket : {e}")
        client_socket = None

    try:
        path_plot = None
        path_simplified_plot = None
        calcul_en_cours = False
        compteur_affichage = 0
        compteur_batteries = 0
        
        # Demande initiale décision
        queue_calcul_decision.put({
            "x_robot": x_robot_actuel,
            "y_robot": y_robot_actuel,
            "x_ennemi": x_ennemi,
            "y_ennemi": y_ennemi,
            "robot_a_objets": robot_a_objets,
            "Liste_noisettes_libres": Liste_noisettes_libres.copy(),
            "Liste_GM_libres": Liste_GM_libres.copy()
        })
        
        while not stop_event.is_set() and Batteries[2][3] > 5:
            step += 1
            Liste_trajectoire = [0]
            
            # ✅ Zones seulement si changement
            if Liste_GM_libres != Liste_GM_libres_old or Liste_noisettes_libres != Liste_noisettes_libres_old:
                if DEBUG_MODE:
                    print("🔄 Mise à jour zones...")
                grid, grid_expanded, obstacle_array, expanded_array, obs_manager, obs_manager_noisettes, Liste_GM_libres, Liste_GM_occuper, Liste_noisettes_libres, Liste_noisettes_prises, obstacle_scatter, expanded_scatter, distance_map, ax, width, height, CASE_MM = actualiser_zones_jeu(grid, grid_expanded, obstacle_array, expanded_array, obs_manager, obs_manager_noisettes, Liste_GM_libres, Liste_GM_occuper, Liste_noisettes_libres, Liste_noisettes_prises, obstacle_scatter, expanded_scatter, distance_map, ax, width, height, CASE_MM)
                Liste_GM_libres_old = Liste_GM_libres.copy()
                Liste_noisettes_libres_old = Liste_noisettes_libres.copy()
            
            # ✅ Vérifier résultats threads
            if not queue_resultat_decision.empty():
                resultat = queue_resultat_decision.get()
                action_en_cours = resultat["action_en_cours"]
                calcul_en_cours = False
                print("✅ Décision reçue")
            
            if not queue_resultat_astar.empty():
                resultat = queue_resultat_astar.get()
                
                if resultat["success"]:
                    waypoints = resultat["waypoints"]
                    px_mm = resultat["px_mm"]
                    py_mm = resultat["py_mm"]
                    
                    for wp in waypoints:
                        Liste_actions.insert(0, wp)
                    
                    if path_simplified_plot is not None:
                        path_simplified_plot.remove()
                    path_simplified_plot, = ax.plot(px_mm, py_mm, 'g-', linewidth=2, marker='o', markersize=4, zorder=10)
                    
                    if len(waypoints) > 0:
                        Liste_trajectoire = [len(waypoints)]
                        for wp in waypoints:
                            Liste_trajectoire.append([wp[1], wp[2]])
                    
                    calcul_en_cours = False
                    print(f"✅ Trajectoire : {len(waypoints)} waypoints")
                else:
                    print(f"❌ Échec A*")
                    calcul_en_cours = False
            
            # Nouvelle décision si vide
            if len(Liste_actions) == 0 and not calcul_en_cours:
                queue_calcul_decision.put({
                    "x_robot": x_robot_actuel,
                    "y_robot": y_robot_actuel,
                    "x_ennemi": x_ennemi,
                    "y_ennemi": y_ennemi,
                    "robot_a_objets": robot_a_objets,
                    "Liste_noisettes_libres": Liste_noisettes_libres.copy(),
                    "Liste_GM_libres": Liste_GM_libres.copy()
                })
                calcul_en_cours = True
                
                if len(Liste_actions) == 0:
                    time.sleep(0.1)
                    continue
            
            # Simulation ennemi
            if not Reel:
                x_ennemi += 6
                y_ennemi -= 4
                if step == 130:
                    x_ennemi = 1500
                    y_ennemi = 450
                if step == 700:
                    x_ennemi = 1000
                    y_ennemi = 1000
            
            # Lire action
            if len(Liste_actions) > 0:
                if type(Liste_actions[0]) == list and len(Liste_actions[0]) == 3:
                    action_voulu = Liste_actions[0][0]
                    x_robot_voulu = Liste_actions[0][1]
                    y_robot_voulu = Liste_actions[0][2]
                elif type(Liste_actions[0]) == list and len(Liste_actions[0]) == 2:
                    action_voulu = Liste_actions[0][0]
                    angle_robot_voulu = round(Liste_actions[0][1], 0)
                else:
                    action_voulu = Liste_actions[0][0]
            else:
                action_voulu = "Aucune"
            
            # ✅ Demander calcul A* si nécessaire
            delta_x_ennemi = abs(x_ennemi - x_ennemi_old)
            delta_y_ennemi = abs(y_ennemi - y_ennemi_old)
            ennemi_a_bouge = (delta_x_ennemi > 50 or delta_y_ennemi > 50)
            
            if not calcul_en_cours and action_voulu in ["Consigne", "Avancer"] and (ennemi_a_bouge or step == 1):
                queue_calcul_astar.put({
                    "x_robot": x_robot_actuel,
                    "y_robot": y_robot_actuel,
                    "x_voulu": x_robot_voulu,
                    "y_voulu": y_robot_voulu,
                    "x_ennemi": x_ennemi,
                    "y_ennemi": y_ennemi,
                    "safety_weight": 1.0
                })
                calcul_en_cours = True
                print(f"🔄 Calcul A* demandé (step {step})")
            
            # Logs
            if DEBUG_MODE:
                print(f"\n[Step {step}] Action: {action_voulu}")
                print(f"Robot: ({x_robot_actuel:.0f}, {y_robot_actuel:.0f}, {angle_robot_actuel:.0f}°)")
            
            # Mise à jour affichage
            robot_plot.set_offsets([[x_robot_actuel, y_robot_actuel]])
            
            zone_ennemi_scatter, cercle_ennemi_patch = mettre_a_jour_zone_ennemi(
                zone_ennemi_scatter, cercle_ennemi_patch,
                x_ennemi, y_ennemi, R_ROBOT, R_ENNEMI,
                MARGE_SECURITE_ENNEMI, CASE_MM, X_PISTE, Y_PISTE
            )
            ennemi_plot.set_offsets([[x_ennemi, y_ennemi]])
            
            if action_voulu in ["Consigne", "Avancer"]:
                consigne_plot.set_offsets([[x_robot_voulu, y_robot_voulu]])
            else:
                consigne_plot.set_offsets([[-20, -20]])
            
            x0, y0 = x_robot_actuel, y_robot_actuel
            x1 = x0 + longueur_trait * math.cos(math.radians(angle_robot_actuel))
            y1 = y0 + longueur_trait * math.sin(math.radians(angle_robot_actuel))
            robot_angle_line.set_data([x0, x1], [y0, y1])
            
            if action_voulu == "Rotation":
                x0, y0 = x_robot_actuel, y_robot_actuel
                x1 = x0 + longueur_trait * math.cos(math.radians(angle_robot_voulu))
                y1 = y0 + longueur_trait * math.sin(math.radians(angle_robot_voulu))
                robot_angle_voulu_line.set_data([x0, x1], [y0, y1])
            else:
                robot_angle_voulu_line.set_data([-20, -20], [-40, -40])
            
            # Batteries (1 fois sur 10)
            compteur_batteries += 1
            if compteur_batteries >= 10:
                Ordre_Batteries = gerer_basculement_batteries(Batteries, U_last, Ordre_Batteries)
                battery_patches, battery_texts = afficher_batteries(ax, Batteries, Ordre_Batteries, battery_patches, battery_texts, couleurs, seuils, largeur_rect, hauteur_rect, espacement, espacement_salves, y_base, texte_offset_y)
                compteur_batteries = 0
            
            # Textes
            robot_info_text.set_text(f"X={x_robot_actuel:.1f} Y={y_robot_actuel:.1f} A={angle_robot_actuel:.1f}°")
            x_voulu_text.set_text(f"X={x_robot_voulu:.1f}")
            y_voulu_text.set_text(f"Y={y_robot_voulu:.1f}")
            A_voulu_text.set_text(f"A={angle_robot_voulu:.1f}°")
            
            if abs(x_robot_actuel - x_robot_voulu) > 10:
                x_voulu_text.set_color('black')
            else:
                x_voulu_text.set_color('green')
            
            if abs(y_robot_actuel - y_robot_voulu) > 10:
                y_voulu_text.set_color('black')
            else:
                y_voulu_text.set_color('green')
            
            if abs(angle_robot_actuel - angle_robot_voulu) > 1:
                A_voulu_text.set_color('black')
            else:
                A_voulu_text.set_color('green')
            
            # Vérifier position atteinte
            if Reel:
                if abs(x_robot_actuel - x_robot_voulu) < 10 and abs(y_robot_actuel - y_robot_voulu) < 10 and action_voulu in ["Consigne", "Avancer", "Recul"]:
                    ordre_receive = 11
                if abs(angle_robot_actuel - angle_robot_voulu) < 1 and action_voulu in ["Rotation"]:
                    ordre_receive = 12
            else:
                if abs(x_robot_actuel - x_robot_voulu) < 30 and abs(y_robot_actuel - y_robot_voulu) < 30 and action_voulu in ["Consigne", "Avancer"]:
                    ordre_receive = 11
                if abs(angle_robot_actuel - angle_robot_voulu) < 5 and action_voulu in ["Rotation"]:
                    ordre_receive = 12
            
            # Action terminée
            if (action_voulu in ["Consigne", "Avancer"] and ordre_receive == 11) or \
               (action_voulu in ["Rotation"] and ordre_receive == 12) or \
               (action_voulu == "Attraper" and ordre_receive == 21) or \
               (action_voulu == "Relacher" and ordre_receive == 22):
                
                Liste_actions.pop(0)
                
                if action_voulu in ["Attraper", "Relacher"]:
                    robot_a_objets = confirmer_action_terminee(
                        action_en_cours, robot_a_objets,
                        Liste_noisettes_libres, Liste_noisettes_prises,
                        Liste_GM_libres, Liste_GM_occuper, verbose=True
                    )
                    Liste_actions.clear()
                    
                    queue_calcul_decision.put({
                        "x_robot": x_robot_actuel,
                        "y_robot": y_robot_actuel,
                        "x_ennemi": x_ennemi,
                        "y_ennemi": y_ennemi,
                        "robot_a_objets": robot_a_objets,
                        "Liste_noisettes_libres": Liste_noisettes_libres.copy(),
                        "Liste_GM_libres": Liste_GM_libres.copy()
                    })
                
                ordre_receive = 0
            
            # ✅ Affichage 1 fois sur 5
            compteur_affichage += 1
            if compteur_affichage >= 5:
                fig.canvas.draw_idle()
                fig.canvas.flush_events()
                compteur_affichage = 0
            
            time.sleep(0.03)
            
            U_last = [Batteries[0][2], Batteries[1][2], Batteries[2][2]]
            x_ennemi_old = x_ennemi
            y_ennemi_old = y_ennemi
            
            # ✅ Envoi socket persistant
            if client_socket:
                try:
                    donnees_pour_robot = {
                        "Liste_actions": Liste_actions,
                        "Liste_trajectoire": Liste_trajectoire,
                        "Ordre_receive": ordre_receive
                    }
                    message = json.dumps(donnees_pour_robot)
                    client_socket.sendall(message.encode() + b'\n')
                except Exception as e:
                    if DEBUG_MODE:
                        print(f"❌ Erreur socket : {e}")
                    try:
                        client_socket.close()
                        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        client_socket.connect((HOST_PC, PORT_ENVOI))
                        print("✅ Reconnexion socket")
                    except:
                        print("❌ Impossible reconnecter")
                        client_socket = None
        
        time.sleep(2)
        stop_event.set()
    
    except KeyboardInterrupt:
        print("Arrêt utilisateur")
        stop_event.set()
    
    finally:
        print("Programme terminé")
        
        if client_socket:
            try:
                client_socket.close()
                print("✅ Socket fermé")
            except:
                pass
        
        plt.close(fig)
