couleur = "B"
Camera = True
Camera_active = Camera
WiFi = False
Pami = True
Strategie = False
Debug_strategie = False
Astars = False

Simul_mvt = True
Simul_mvt_ennemi = False
Debug_Mouv = False

Simul_action = True
Debug_Action = False

Lidar_on = False
Bat_Compet = False
Mode_pince = True
lancement_cartes = False

Noisettes_stockees_dans_robot = [["N","N"],["N","N"]]
################## Librairies ##########################################
import matplotlib
matplotlib.use('Qt5Agg')
import math,time,os,struct,cv2
import numpy as np
import threading
from collections import deque 
import matplotlib.pyplot as plt
from functools import partial
from matplotlib.widgets import Button
import socket
import json
from affichage import init_affichage, bring_to_front,afficher_obstacles,afficher_zone_securite_ennemi, mettre_a_jour_zone_ennemi,afficher_batteries,dessiner_noisettes,fenetre_selection_couleur
from calcul_mouv import  calculer_trajectoire_complete,actualiser_zones_jeu,verifier_segments_trajectoire_ennemi
from fonction import associer_noisette_a_emplacement,detecter_changements_noisettes, distance
from fichier_strategie import trouver_groupes_initiaux, separer_groupe,regrouper_par_quatre
from homographie_couleur import Config,ArUcoTrackingSystem,ButtonManager,Button_A
########################################################################
couleur = fenetre_selection_couleur()
# Config Wi-Fi 
# Configuration pour l'envoi
IP_ROBOT = "192.168.0.102"
PORT_ENVOI = 5001

# Configuration pour la réception
IP_RECEPTION = '0.0.0.0'  
PORT_RECEPTION = 5000

IP_PAMI = ["192.168.0.103","192.168.0.104","192.168.0.105","192.168.0.106","192.168.0.107","192.168.0.108","192.168.0.109"]
PORT_PAMI =  5002
#################################################

# Perimètre de sécurité
R_ROBOT = 160
LARGEUR_ROBOT = 250
LONGUEUR_ROBOT = 220
R_ENNEMI = 150
MARGE_ENNEMI = 100
MARGE_NOISETTE = 20
MARGE_GM = 10
MARGE_TRAJECTOIRE = 20
R_securite = R_ROBOT + R_ENNEMI + MARGE_ENNEMI
############################

if Camera :
    Liste_noisette_xya = [] 
else :
    Liste_noisette_xya = [
        [175-30,1125+20,0+2,"B"],[175-30,1175+20,0+2,"J"],[175-30,1225+20,0+2,"B"],[175-30,1275+20,0+2,"J"],
        [175-30,325-20,0-2,"B"],[175-30,375-20,0-2,"J"],[175-30,425-20,0-2,"B"],[175-30,475-20,0-2,"J"],

        [2825+30,1125+20,0-2,"J"],[2825+30,1175+20,0-2,"J"],[2825+30,1275+20,0+2,"B"],[2825+30,1225+20,0+2,"B"],
        [2825+30,325-20,0-2,"B"],[2825+30,375-20,0-2,"J"],[2825+30,425-20,0+2,"J"],[2825+30,475-20,0+2,"B"],

        [1075-30,800-20,90+2,"J"],[1125-30,800-20,90-2,"J"],[1175-30,800-20,90+2,"B"],[1225-30,800-20,90-2,"B"],
        [1775+30,800-20,90+2,"J"],[1825+30,800-20,90-2,"B"],[1875+30,800-20,90+2,"B"],[1925+30,800-20,90-2,"J"],

        [1025-30,175-20,90-2,"B"],[1075-30,175-20,90+2,"B"],[1125-30,175-20,90-2,"J"],[1175-30,175-20,90+2,"J"],
        [1825+30,175-20,90-2,"B"],[1875+30,175-20,90+2,"J"],[1925+30,175-20,90-2,"J"],[1975+30,175-20,90+2,"B"],

    ] 

Liste_noisette_xya_precedente = [noisette[:] for noisette in Liste_noisette_xya]  # Copie profonde


Liste_zones_gm_coins = [
    [[1150,1350],[1350,1550]],
    [[1650,1350],[1850,1550]],

    [[0,700],[200,900]],
    [[700,700],[900,900]],
    [[1400,700],[1600,900]],
    [[2100,700],[2300,900]],
    [[2800,700],[3000,900]],

    [[600,0],[800,200]],
    [[1400,0],[1600,200]],
    [[2200,0],[2400,200]],
]

###########################################

# Piste
X_PISTE = 3000
Y_PISTE = 2000
MARGE_BORDUREPISTE_X = 120 # Détection Lidar
MARGE_BORDUREPISTE_Y = 80 # Détection Lidar
#############################################

# Coordonnées et angle de notre robot (coordonnées initiales en haut)


x_robot_depart = 0 
y_robot_depart = 0
angle_robot_depart = 0

x_robot_retour = 0
y_robot_retour = 0
angle_robot_retour = 0

x_robot_actuel = x_robot_depart
y_robot_actuel = y_robot_depart
angle_robot_actuel = angle_robot_depart

x_robot_actuel_cam = x_robot_actuel
y_robot_actuel_cam = y_robot_actuel
angle_robot_actuel_cam = angle_robot_actuel

x_robot_voulu = -1
y_robot_voulu = -1
angle_robot_voulu = -181
############

# Coordonnées Ennemi
if couleur == "B":
    x_ennemi = 275
    y_ennemi = 1650
    angle_ennemi = 0

else:
    x_ennemi = 2725 
    y_ennemi = 1650
    angle_ennemi = 0

x_ennemi_cam = x_ennemi
y_ennemi_cam = y_ennemi
angle_ennemi_cam = angle_ennemi

x_ennemi_lidar = x_ennemi
y_ennemi_lidar = y_ennemi

x_ennemi_old = x_ennemi
y_ennemi_old = y_ennemi
########

# Variable pour les Grilles du A*
CASE_MM = 10
width = X_PISTE//CASE_MM
height = Y_PISTE//CASE_MM
rayon_total_case = (R_ROBOT + MARGE_GM) // CASE_MM  # = 20 cases = 200mm
####################

# Variables fonctionnelles des Batteries
Batteries = [100,100,100,100] # V décharge, V charge, V actuel, % de charge
Batteries_interrupteur = [1,1,1]
Batteries_alert = [0,0,0,0]
RPI_decharge = False
############################

# Variables pour l'affichage des rectangles de batteries et des textes
battery_patches = []
battery_texts = [] 
couleurs = ['red', 'orange', 'yellow', 'lime', 'green']
seuils = [1, 20, 50, 60, 80]
largeur_rect = 50       # largeur en mm
hauteur_rect = 150      # hauteur en mm
espacement = 0         # espace entre rectangles
espacement_salves = 100 # espace entre chaque salve
y_base = 2050           # position verticale (en haut de la piste)
marge_texte = 20  # espace horizontal entre texte et rectangle
texte_offset_y = 50  # décalage vertical du texte par rapport aux rectangles
longueur_trait = 100  # longueur trait de direction
compteur_affichage = 0
FREQUENCE_AFFICHAGE = 4
##########################

# Objets et variables pour la fenêtre graphique
fig = None 
ax = None
robot_plot = None
scat = None
zone_ennemi_scatter = None
cercle_ennemi_patch = None
############################

# Variables globales pour les boutons et l'affichage des obstacles
obstacle_scatter = None
expanded_scatter = None
############################################################################

# =========== Variables pour le fonctionnement Logique du Robot
lancement_strategie = False

carte_asserv_active = 0
carte_actionneur_active = 0
carte_RPI_active = 0
carte_batteries_active = 0
etat_bau = 1

demande_nouvelle_strat = False
x_strategie = 2825
y_strategie = 1150
strategie_en_cours = [] 

step = 0

temps_demarage = 0
temps_ecoules = 0
temps_restant = 100
temps_retour = 15 # Temps restant pour revenir au départ en fin de match
temps_max = 100

action_voulu = None
action_en_cours = None
action_precedente = None

verif_mouv = 0
old_verif_mouv = verif_mouv
verif_angle = 0
verif_recalage = 0
verif_action = 0
verif_Noisette_a_bouge_simul = 0
action_est_supprime = False 
mode_attraper = False

pince_a_utilise = -1
noisette_a_manipulee = 0

demande_recalcul_traj = False
Astars_a_fail = False
ordre_mouvement = 0
old_ordre_mouvement = 0

PAMI_debut_match = False
match_demarre = False
old_match_demarre = False
match_id = None
old_match_id = None
step_robot = 0
old_step_robot = step_robot
# ==================================================

TOL_POS_X = 16 
TOL_POS_Y = 16
TOL_POS_A = 5

dernier_envoi_pami = 0
FREQUENCE_ENVOI_PAMI = 0.5  # Envoyer toutes les 200ms (5 fois par seconde)

################## Fonction Threads ##########################################
def comm_robot(stop_event):
    print(f"[Récepteur] Serveur en attente sur le port {PORT_RECEPTION}...")
    
    global x_robot_actuel, y_robot_actuel, angle_robot_actuel, x_ennemi_lidar, y_ennemi_lidar, Batteries, Batteries_alert,Noisettes_stockees_dans_robot, action_voulu, action_precedente,step_robot,match_demarre,match_id,old_match_demarre,temps_demarage,old_match_id,temps_ecoules,temps_restant,temps_retour,temps_max,PAMI_debut_match

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((IP_RECEPTION, PORT_RECEPTION))
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
                
                donnees_recues = json.loads(data.decode())
                
                x_robot_actuel = donnees_recues["x_robot_actuel"]
                y_robot_actuel = donnees_recues["y_robot_actuel"]
                angle_robot_actuel = donnees_recues["angle_robot_actuel"]
                x_ennemi_lidar = donnees_recues["x_ennemi"]
                y_ennemi_lidar = donnees_recues["y_ennemi"]
                Batteries = donnees_recues["Batteries"]
                Batteries_alert = donnees_recues["Batteries_alert"]
                Noisettes_stockees_dans_robot = donnees_recues["Noisettes_stockees_dans_robot"]
                action_voulu = donnees_recues["action_voulu"]
                action_precedente = donnees_recues["action_precedente"]
                step_robot = donnees_recues["step_robot"]
                match_demarre = donnees_recues["match_demarre"]
                match_id = donnees_recues["match_id"]
                
                if match_id != old_match_id:
                    temps_ecoules = 0
                    temps_restant = 100
                    temps_retour = 15 # Temps restant pour revenir au départ en fin de match
                    temps_max = 100
                    PAMI_debut_match = False
                if match_demarre == True and old_match_demarre == False:
                    temps_demarage = time.time()
                    PAMI_debut_match = True
                old_match_demarre = match_demarre
                old_match_id = match_id
            except json.JSONDecodeError:
                print("[Récepteur] Erreur : données JSON invalides")
            
    except KeyboardInterrupt:
        print("[Récepteur] Arrêt.")
    finally:
        server_socket.close()
##############################################################################

################## Fonction ##################################################
def arret_programme(event, stop_event=None):
    print("Bouton STOP pressé — arrêt demandé.")
    if stop_event is not None:
        stop_event.set()
        bring_to_front(fig)

def demarrage_strategie(event):
    """Fonction appelée quand le bouton START est pressé."""
    global lancement_strategie
    lancement_strategie = not lancement_strategie  # Toggle (bascule) de l'état
    
    if lancement_strategie:
        print("✅ Bouton START pressé — Stratégie ACTIVÉE")
    
    bring_to_front(fig)

def on_click(event):
    """
    Callback pour les clics sur la piste.
    
    COMPORTEMENT :
    - Clic GAUCHE (bouton 1) : Ajoute une consigne PRIORITAIRE en début de Liste_actions
    - Clic DROIT (bouton 3) : Affiche juste le point voulu (ancien comportement)
    """
    global x_robot_voulu, y_robot_voulu, Liste_actions, Strategie,x_strategie,y_strategie,demande_nouvelle_strat
    
    if event.inaxes == ax:  # Clic dans la zone du graphique
        x_clic = event.xdata
        y_clic = event.ydata
        
        # Mettre à jour l'affichage du point voulu
        x_robot_voulu = x_clic
        y_robot_voulu = y_clic
        point_voulu_plot.set_data([round(x_robot_voulu, 0)], [round(y_robot_voulu, 0)])
        
        # ⭐ CLIC GAUCHE : Ajouter une consigne PRIORITAIRE ⭐
        if event.button == 1:  # Bouton gauche
            if not Strategie:
                if not Astars:
                    Liste_actions.insert(-1,["Consigne", round(int(x_clic), 0), round(int(y_clic), 0)])
                else :
                    # Vider la liste (modifie la liste globale, pas une copie locale)
                    Liste_actions.clear()
                    # Ajouter la consigne cliquée
                    Liste_actions.append(["Consigne", round(int(x_clic), 0), round(int(y_clic), 0)])
            else :
                x_strategie = round(int(x_clic), 0)
                y_strategie = round(int(y_clic), 0)
                demande_nouvelle_strat = True
        
        plt.draw()
        bring_to_front(fig)


def bouton_attraper_callback(event):
    """
    Callback pour le bouton Attraper.
    """
    global verif_action, action_voulu
    
    # Vérifier qu il y a une action en cours
    if len(Liste_actions) > 0 and action_voulu in ["Attraper","Retourner","Relacher"]:
        verif_action = 1


def update_display(background):
    """Mise à jour optimisée avec blitting"""
    # Restaurer le fond
    fig.canvas.restore_region(background)
    
    # Redessiner uniquement les éléments qui changent
    ax.draw_artist(robot_plot)
    ax.draw_artist(ennemi_plot)
    ax.draw_artist(scat)
    ax.draw_artist(robot_info_text)
    ax.draw_artist(info_alim_rpi)
    ax.draw_artist(chronometre_text)
    ax.draw_artist(match_text)
    ax.draw_artist(noisette_text)
    ax.draw_artist(x_voulu_text)
    ax.draw_artist(y_voulu_text)
    ax.draw_artist(A_voulu_text)
    ax.draw_artist(cercle_robot_patch)
    fig.canvas.blit(ax.bbox)
    fig.canvas.flush_events()

def appliquer_couleur(config: Config, couleur: str) -> Config:
    """
    Configure les variables de tags selon la couleur de l'équipe.
    couleur == 'B' : équipe Bleue
    couleur == 'J' : équipe Jaune
    """
    if couleur == "B":
        config.tag_calibration_Noisette = 51
        config.tag_calibration_robot    = 52
        config.tag_robot                = (1, 2, 3, 4, 5)
        config.tag_ennemi               = (6, 7, 8, 9, 10)
    elif couleur == "J":
        config.tag_calibration_Noisette = 71
        config.tag_calibration_robot    = 72
        config.tag_robot                = (6, 7, 8, 9, 10)
        config.tag_ennemi               = (1, 2, 3, 4, 5)
    else:
        raise ValueError(f"Couleur '{couleur}' invalide. Valeurs acceptées : 'B' ou 'J'.")
    return config

########################################################################

############################### Programme principal ####################

if __name__ == '__main__':

    stop_event = threading.Event()
    
    if WiFi:
        thread_reception = threading.Thread(target=comm_robot, args=(stop_event,), daemon=True)
        thread_reception.start()

    fig, ax, robot_plot, ennemi_plot, consigne_plot, scat, robot_info_text,ax_button_stop,bouton_stop,ax_button_start,bouton_start,point_voulu_plot,x_voulu_text,y_voulu_text,A_voulu_text, robot_angle_line,robot_angle_voulu_line,background,info_alim_rpi,chronometre_text,cercle_robot_patch,noisette_text,match_text = init_affichage(x_robot_depart,y_robot_depart,R_ROBOT)

    cid = fig.canvas.mpl_connect('button_press_event', on_click) # Choix des coordonnées voulues avec la souris
    bouton_stop.on_clicked(partial(arret_programme, stop_event=stop_event))
    bouton_start.on_clicked(demarrage_strategie)  # ⭐ Connexion du bouton START ⭐
    
    #obstacle_scatter, expanded_scatter = afficher_obstacles(ax,obstacle_array,expanded_array,CASE_MM,show_expanded=True,show_obstacles=False)
    zone_ennemi_scatter, cercle_ennemi_patch = afficher_zone_securite_ennemi(ax,x_ennemi, y_ennemi,R_ROBOT, R_ENNEMI,MARGE_ENNEMI,CASE_MM,X_PISTE,Y_PISTE,show_zone=True,show_cercle=False)
    
    # ========== DESSIN DES NOISETTES AVEC COULEURS ==========
    patches_noisettes = dessiner_noisettes(ax, Liste_noisette_xya,longueur=150, largeur=50,alpha=0.7, linewidth=2)
    # ========================================================

    # création du trait, initialement à la position du robot
    ax.add_line(robot_angle_line)
    ax.add_line(robot_angle_voulu_line)
    
    # Bouton ATTRAPER
    ax_attraper_button = plt.axes([0.38, 0.92, 0.12, 0.08])
    bouton_attraper = Button(ax_attraper_button, "Action", color="lightblue", hovercolor="blue")
    bouton_attraper.on_clicked(bouton_attraper_callback)

    changement_noisettes_detecte = detecter_changements_noisettes(
        Liste_noisette_xya, 
        Liste_noisette_xya_precedente
    )
    
    if Camera_active:
        config = Config()
        config = appliquer_couleur(config, couleur)

        if couleur == "B":
            calibration_sequence = [20, 22, 23, 21]
        elif couleur == "J":
            calibration_sequence = [21, 23, 22, 20]
        else:
            calibration_sequence = [20, 21, 22, 23]

        print(f"Équipe configurée : {'Bleue' if couleur == 'B' else 'Jaune'}")
        print(f"  tag_calibration_Noisette : {config.tag_calibration_Noisette}")
        print(f"  tag_calibration_robot    : {config.tag_calibration_robot}")
        print(f"  tag_robot                : {config.tag_robot}")
        print(f"  tag_ennemi               : {config.tag_ennemi}")

        system = ArUcoTrackingSystem(config, 
                             matrice_antidstorsion='calibration_data_HR_vraiecam.npz',
                             calibration_sequence=calibration_sequence)
        # Chargement automatique des deux calibrations au démarrage
        if system.calibration_mode.load_calibration():
            system.homographie.calcul_homographie_elevated(system.calibration_mode.calibration_points)
            system.plan_elevated_calcule = True

        if system.calibration_mode_robot.load_calibration():
            system.homographie.calcul_homographie_robot(system.calibration_mode_robot.calibration_points)
            system.plan_robot_calcule = True

        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print("Erreur: impossible d'ouvrir la caméra")

        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.camera_largeur)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera_longueur)

        # Créer la fenêtre et le gestionnaire de boutons
        window_name = "Systeme de Tracking ArUco"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 720)
        
        button_manager = ButtonManager(window_name)
        
        # Ajouter les boutons (position en bas de l'écran redimensionné)
        btn_y = 800  # Position Y des boutons
        btn_h = 50   # Hauteur des boutons
        btn_spacing = 5
        
        buttons_config = [
            (10, "Noisette", (0, 100, 150)),
            (150, "Robot", (0, 150, 100)),
            (280, "Capturer", (150, 100, 0)),
            (400, "Sauvegarder", (0, 150, 150)),
            (550, "Chargement", (100, 0, 150)),
            (690, "Couleurs", (150, 150, 0)),
            (820, "Debug", (100, 100, 0)),
            (930, "Quitter", (150, 0, 0)),
        ]
        
        for btn_x, btn_text, btn_color in buttons_config:
            button_manager.add_button(Button_A(btn_x, btn_y, 130, btn_h, btn_text, btn_color))
    
    
    try:
        while (not stop_event.is_set()): # Tant que le Flag de Thread n'est pas levé, que la batterie RPI est suffisamment chargées, qu'il y a encore des actions à réaliser, que le BAU n'est pas appuyé
            if temps_demarage != 0:
                temps_ecoules = time.time() - temps_demarage
            temps_restant = temps_max - temps_ecoules

            if Camera_active:
                cv2.namedWindow("Systeme de Tracking ArUco", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Systeme de Tracking ArUco", 1280, 720)

                ret, frame = cap.read()
                if not ret:
                    print("Erreur de lecture de la caméra")
                    break

                annotated, results = system.process_frame(frame)
                Liste_noisette_xya = results['Liste_noisette_xya']
                Liste_robots_xy    = results['Liste_robots_xy']
                for robot in Liste_robots_xy:
                    if robot[4]=='R':
                        x_robot_actuel_cam = robot[1]
                        y_robot_actuel_cam = robot[2]
                        angle_robot_actuel_cam = robot[3]
                    if robot[4]=='E':
                        x_ennemi_cam = robot[1]
                        y_ennemi_cam = robot[2]
                        angle_ennemi_cam = robot[3]


                if hasattr(system, 'show_debug') and system.show_debug:
                    system.color_detector.show_debug_masks(frame)

                if results['homographie_ok'] and not system.plan_reference_calcule:
                    print("Plan de référence calculé et verrouillé.")
                    system.plan_reference_calcule = True

                # Dessiner les boutons sur l'image
                button_manager.draw_all(annotated)
                
                cv2.imshow(window_name, annotated)
                
                # Gérer les clics de boutons
                button_click = button_manager.get_last_click()
                
                if button_click == "Noisette":
                    system.mode_calibration_active = True
                    system.calibration_mode.reset()
                    print("\nMode calibration NOISETTE activé")
                    print(f"Positionner le tag {config.tag_calibration_Noisette} au-dessus du tag "
                        f"{system.calibration_mode.get_current_target()} et appuyer sur CAPTURER")
                
                elif button_click == "Robot":
                    system.mode_calibration_robot_active = True
                    system.calibration_mode_robot.reset()
                    print("\nMode calibration ROBOT activé")
                    print(f"Positionner le tag {config.tag_calibration_robot} au-dessus du tag "
                        f"{system.calibration_mode_robot.get_current_target()} et appuyer sur CAPTURER")
                
                elif button_click == "Capturer":
                    detected_tags_list, _ = system.detecteur.detect(system.undistort_image(frame))
                    if system.mode_calibration_active:
                        system.handle_calibration_capture(detected_tags_list)
                    elif system.mode_calibration_robot_active:
                        system.handle_calibration_capture_robot(detected_tags_list)
                    else:
                        print("Aucune calibration active. Cliquez d'abord sur 'Plan Noisette' ou 'Plan Robot'")
                
                elif button_click == "Sauvegarder":
                    if system.mode_calibration_active and system.calibration_mode.is_complete():
                        system.calibration_mode.save_calibration()
                        system.mode_calibration_active = False
                        system.plan_elevated_calcule = True
                        print("Calibration NOISETTE sauvegardée.")
                    elif system.mode_calibration_robot_active and system.calibration_mode_robot.is_complete():
                        system.calibration_mode_robot.save_calibration()
                        system.mode_calibration_robot_active = False
                        system.plan_robot_calcule = True
                        print("Calibration ROBOT sauvegardée.")
                    else:
                        print("Aucune calibration complète à sauvegarder")
                
                elif button_click == "Chargement":
                    if not system.mode_calibration_active and not system.mode_calibration_robot_active:
                        if system.calibration_mode.load_calibration():
                            system.homographie.calcul_homographie_elevated(system.calibration_mode.calibration_points)
                            system.plan_elevated_calcule = True
                        if system.calibration_mode_robot.load_calibration():
                            system.homographie.calcul_homographie_robot(system.calibration_mode_robot.calibration_points)
                            system.plan_robot_calcule = True
                    else:
                        print("Impossible de charger pendant une calibration active")
                
                elif button_click == "Couleurs":
                    print("\n=== Entrée en mode calibration couleurs ===")
                    system.color_detector.calibrate_interactive(frame)
                
                elif button_click == "Debug":
                    if not hasattr(system, 'show_debug'):
                        system.show_debug = False
                    system.show_debug = not system.show_debug
                    if system.show_debug:
                        print("\nMode debug ACTIVÉ")
                    else:
                        print("\nMode debug DÉSACTIVÉ")
                        for win in ["Masque Jaune", "Masque Bleu", "Masques Combinés (Bleu=Bleu, Jaune=Rouge)"]:
                            cv2.destroyWindow(win)
                
                elif button_click == "Quitter":
                    print("Fermeture de la fenêtre caméra...")
                    Camera_active = False
                    cv2.destroyAllWindows()
                    if cap.isOpened():
                        cap.release()
                
                key = cv2.waitKey(1) & 0xFF

                if key == ord('q'):
                    print("Fermeture de la fenêtre caméra...")
                    Camera_active = False
                    cv2.destroyAllWindows()
                    if cap.isOpened():
                        cap.release()

            if WiFi:
                donnees_pour_robot = {
                    "Liste_noisette_xya_cam": Liste_noisette_xya,
                    "x_robot_actuel_cam": x_robot_actuel_cam,
                    "y_robot_actuel_cam": y_robot_actuel_cam,
                    "angle_robot_actuel_cam": angle_robot_actuel_cam,
                    "x_ennemi_cam": x_ennemi_cam,
                    "y_ennemi_cam": y_ennemi_cam,
                    "angle_ennemi_cam": angle_ennemi_cam,
                }
                try :
                    message = json.dumps(donnees_pour_robot)
                    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    client_socket.settimeout(0.1)
                    client_socket.connect((IP_ROBOT, PORT_ENVOI))
                    client_socket.sendall(message.encode())
                    client_socket.close()
                except (socket.timeout, ConnectionRefusedError, OSError) as e:
                    print(f"WiFi Envoi échoué : {e}")

            step +=1
            print("step :",step)
            
            # ======================== Tri Noisettes ============================================= #
            if Mode_pince :
                groupes_initiaux, adjacence = trouver_groupes_initiaux(Liste_noisette_xya)

                Noisettes_groupees = []
                for groupe in groupes_initiaux:
                    paires = separer_groupe(groupe, Liste_noisette_xya, adjacence)
                    for paire in paires:
                        noisettes_paire = [Liste_noisette_xya[i] for i in paire]
                        Noisettes_groupees.append(noisettes_paire)
                    
            else :
                groupes_initiaux, adjacence = trouver_groupes_initiaux(Liste_noisette_xya)

                Noisettes_groupees = []
                for groupe in groupes_initiaux:
                    groupes_quatre = regrouper_par_quatre(groupe, Liste_noisette_xya)
                    for groupe_quatre in groupes_quatre:
                        noisettes_groupe = [Liste_noisette_xya[i] for i in groupe_quatre]
                        Noisettes_groupees.append(noisettes_groupe)

            changement_noisettes_detecte = detecter_changements_noisettes(
                Liste_noisette_xya, 
                Liste_noisette_xya_precedente
            )


            Liste_Noisettes_dans_GM = []
            for Noisette in Liste_noisette_xya:
                x_centre, y_centre, angle = Noisette[0], Noisette[1], Noisette[2]
    
                # Dimensions
                longueur = 150  # mm (dans la direction de l'angle)
                largeur = 50    # mm (perpendiculaire à l'angle)
                
                # Conversion angle en radians
                angle_rad = math.radians(angle)
                
                # Vecteurs directeurs
                dx_long = (longueur / 2) * math.cos(angle_rad)
                dy_long = (longueur / 2) * math.sin(angle_rad)
                dx_larg = (largeur / 2) * math.sin(angle_rad)  # Perpendiculaire = rotation de 90°
                dy_larg = -(largeur / 2) * math.cos(angle_rad)
                
                # Calcul des 4 coins (sens trigonométrique depuis le centre)
                Noisette_coin_hg = (x_centre - dx_long - dx_larg, y_centre - dy_long - dy_larg)  # Haut-Gauche
                Noisette_coin_hd = (x_centre + dx_long - dx_larg, y_centre + dy_long - dy_larg)  # Haut-Droite
                Noisette_coin_bd = (x_centre + dx_long + dx_larg, y_centre + dy_long + dy_larg)  # Bas-Droite
                Noisette_coin_bg = (x_centre - dx_long + dx_larg, y_centre - dy_long + dy_larg)  # Bas-Gauche
    
                # Vérifier si AU MOINS UN coin est dans une zone GM
                for num_gm in range(len(Liste_zones_gm_coins)):
                    zone = Liste_zones_gm_coins[num_gm]
                    x_min, y_min = zone[0]
                    x_max, y_max = zone[1]
                    
                    # Liste des 4 coins
                    coins = [Noisette_coin_hg, Noisette_coin_hd, Noisette_coin_bd, Noisette_coin_bg]
                    
                    # Vérifier si au moins un coin est dans la zone
                    if any(x_min <= coin[0] <= x_max and y_min <= coin[1] <= y_max for coin in coins):
                        Liste_Noisettes_dans_GM.append(Noisette)
                        break  # Sortir dès qu'une zone est trouvée
            if Debug_Action:
                print("Liste_Noisettes_dans_GM : ",Liste_Noisettes_dans_GM)      
            # ================================================================================================= #
  
            if changement_noisettes_detecte:
                changement_noisettes_detecte = False
                    
                # ⭐ ÉTAPE 5 : Redessiner les noisettes
                for patch in patches_noisettes:
                    patch.remove()
                patches_noisettes = dessiner_noisettes(
                    ax, Liste_noisette_xya, 
                    longueur=150, largeur=50,
                    alpha=0.7, linewidth=2
                )

            # ============== MISE À JOUR AFFICHAGE ==================== #

            # Mettre à jour robot, ennemi et consigne sur affichage
            robot_plot.set_offsets([[x_robot_actuel_cam, y_robot_actuel_cam]])
            cercle_robot_patch.center = (x_robot_actuel_cam, y_robot_actuel_cam)
            
            x_ennemi = x_ennemi_lidar
            y_ennemi = y_ennemi_lidar
            # Mettre à jour la zone de sécurité dynamique de l'ennemi
            zone_ennemi_scatter, cercle_ennemi_patch = mettre_a_jour_zone_ennemi(
                zone_ennemi_scatter, cercle_ennemi_patch,
                x_ennemi, y_ennemi, R_ROBOT, R_ENNEMI,
                MARGE_ENNEMI, CASE_MM,
                X_PISTE, Y_PISTE
            )
            ennemi_plot.set_offsets([[x_ennemi, y_ennemi]])
            
            x0, y0 = x_robot_actuel_cam, y_robot_actuel_cam
            x1 = x0 + longueur_trait * math.cos(math.radians(angle_robot_actuel_cam))
            y1 = y0 + longueur_trait * math.sin(math.radians(angle_robot_actuel_cam))
            robot_angle_line.set_data([x0, x1], [y0, y1])
            
            # Affichage texte Coordonées
            noisette_text.set_text(f"Avant : {Noisettes_stockees_dans_robot[0]} \nArrière : {Noisettes_stockees_dans_robot[1]}")
            chronometre_text.set_text(f"Temps : {int(temps_restant)} s\nAction : {action_voulu}")
            match_text.set_text(f"Id : {match_id} s\nDemarre : {match_demarre}")

            robot_info_text.set_text(f"X = {x_robot_actuel_cam:.1f} Y = {y_robot_actuel_cam:.1f} A = {angle_robot_actuel_cam:.1f}°")

            # ==================================================================== #
            
            
            # ======================= GESTION BATTERIES ========================== #
            if step > 1:
                battery_patches, battery_texts = afficher_batteries(ax, Batteries_alert,Bat_Compet,Batteries,battery_patches, battery_texts,couleurs, seuils,largeur_rect, hauteur_rect, espacement, espacement_salves,y_base, texte_offset_y)
            # ==================================================================== #
            
            if WiFi and Pami:
                # ✅ VÉRIFIER SI ASSEZ DE TEMPS S'EST ÉCOULÉ
                temps_actuel = time.time()
                socket_pami_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

                if temps_actuel - dernier_envoi_pami >= FREQUENCE_ENVOI_PAMI:
                    dernier_envoi_pami = temps_actuel
                    
                    donnees_pour_PAMI = {
                        "couleur": 1 if couleur == "B" else 2,
                        "PAMI_debut_match": PAMI_debut_match,
                        "temps_restant": int(temps_restant),
                    }
                    message_second = json.dumps(donnees_pour_PAMI).encode()
                    socket_pami_udp.sendto(message_second, ("192.168.0.255", PORT_PAMI))

                            
            # MAJ de l'affichage et des Variables de Bouncing
            update_display(background)
            fig.canvas.flush_events()
            x_robot_voulu_last = x_robot_voulu
            y_robot_voulu_last = y_robot_voulu
            x_ennemi_old = x_ennemi
            y_ennemi_old = y_ennemi
            old_verif_mouv = verif_mouv
            Liste_noisette_xya_precedente = [noisette[:] for noisette in Liste_noisette_xya]  # Copie profonde
            print("")

            time.sleep(0.000005)

        time.sleep(2) 
        stop_event.set()

    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur.")
        stop_event.set()  # signal aux threads de s'arrêter
        # Attente que chaque thread termine proprement
        
    except Exception as e:
        print(e)
    finally:
        print("Programme terminé proprement.")
        
        plt.close(fig)
        cap.release()
        cv2.destroyAllWindows()

########################################################################