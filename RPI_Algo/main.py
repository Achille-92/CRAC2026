Reel = False
Wifi = False
Lidar_on = True
affichage = True
Strategie = True
Astars = True
Faire_curseur = False

Simul_mvt = True
Simul_mvt_ennemi = False
Simul_action = True

Maj_Noisette = False

Bat_Compet = True
Recalage = True
lancement_cartes = True
faire_Ninja = False
Strat_agressive = False
Pousser = True
couleur = "N"
TOL_PRECIS = 15
TOL_PASPRECIS = 70

Noisettes_stockees_dans_robot = [["N","N"],["N","N"]]
################## Librairies ##########################################
import matplotlib
matplotlib.use('Qt5Agg')
from pyrplidar import PyRPlidar
from rplidar import RPLidar
import math,time,os,can,struct,random,platform,sys, json, socket,uuid
current_os = platform.system()
if current_os == "Linux":
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BCM)  # Utilisation de la numérotation BCM
    GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Activation de la résistance de pull-up interne
import numpy as np
import threading
import queue
from collections import deque 
import matplotlib.pyplot as plt
from functools import partial
from matplotlib.widgets import Button
import multiprocessing as mp
from scipy.ndimage import binary_dilation
from affichage import init_affichage, bring_to_front,afficher_obstacles,afficher_zone_securite_ennemi, mettre_a_jour_zone_ennemi,afficher_batteries,dessiner_noisettes,fenetre_selection_couleur,fenetre_selection_agression,fenetre_selection_pousser
from calcul_mouv import  calculer_trajectoire_complete,actualiser_zones_jeu,verifier_segments_trajectoire_ennemi
from fonction import Obstacles,associer_noisette_a_emplacement,detecter_changements_noisettes, distance
from fichier_strategie import trouver_groupes_initiaux, separer_groupe,regrouper_par_quatre,remplir_Liste_actions
########################################################################
couleur = fenetre_selection_couleur()
Strat_agressive = fenetre_selection_agression()
Pousser = fenetre_selection_pousser()

# Config CAN 
Liste_ID_recoit = [0x02,0x03,0x04,0x05,0x06,0x008,0x100, 0x101, 0x102,0x103,0x104,0x105,0x106,0x107,0x108,0x10A,0x10B,0x10C,0x10D,0x10E,0x10F,0x110,0x111,0x112] # ID sur lesquels la RPI va recevoir des données
Liste_ID_envoi = [0x01,0x002,0x003,0x004,0x005,0x006,0x200,0x201,0x202,0x204,0x205,0x206,0x207,0x208,0x209,0x20A,0x300,0x301,0x302,0x303,0x500,0x501,0x502,0x503,0x504,0x505,0x506]
Filtre_CAN = [{"can_id": Id, "can_mask": 0x7FF, "extended": False} for Id in Liste_ID_recoit]
if Reel: 
    os.system('sudo ip link set can0 type can bitrate 500000')
    os.system('sudo ifconfig can0 up')
    bus = can.interface.Bus(
        channel='can0',
        bustype='socketcan',
        bitrate=500000,
        can_filters=Filtre_CAN)
dico_envoi = {}
for ID in Liste_ID_envoi:
    dico_envoi[ID]= 0
#################################################

# Perimètre de sécurité
LARGEUR_ROBOT = 250
LONGUEUR_ROBOT = 138
R_ROBOT = int(math.sqrt((LARGEUR_ROBOT/2)**2+(LONGUEUR_ROBOT/2)**2))
R_ROBOT = 170
R_ENNEMI = 250
MARGE_ENNEMI = 100
MARGE_NOISETTE = 0
MARGE_GM = -35
MARGE_TRAJECTOIRE = 20
R_securite = R_ROBOT + R_ENNEMI + MARGE_ENNEMI
TOL_CAM_NOISETTE = 32
TOLERANCE_STRATEGIE_NOISETTE = 20
FREQUENCE_AFFICHAGE = 2

X_PISTE = 3000
Y_PISTE = 2000
MARGE_BORDUREPISTE_X = 120 # Détection Lidar
MARGE_BORDUREPISTE_Y = 80 # Détection Lidar
DISTANCE_MIN_ROBOT = 50  # Distance minimale au robot en mm
############################
if not Strategie:
    if couleur == "B":
        Liste_actions = [
            ["Consigne",2825,int(900+LONGUEUR_ROBOT/2)],
            ["ReculerPrecis",2825,1200],
            ["Avancer",2500,1200],
            ["Consigne",2600,400],
            ["Rotation",90],
            ["Attente"]
        ]
    else:
        Liste_actions = [
            ["Consigne",175,int(900+LONGUEUR_ROBOT/2)],
            ["ReculerPrecis",175,1200],
            ["Avancer",500,1200],
            ["Consigne",400,400],
            ["Rotation",90],
            ["Attente"]
        ]
else:
    Liste_actions = []

Liste_actions_ennemi = []

Liste_zones_Noisette_depart = [
    [[100,1100],[250,1300]],
    [[100,300],[250,500]],
    [[2750,1100],[2900,1300]],
    [[2750,300],[2900,500]],
    [[1050,725],[1250,875]],
    [[1750,725],[1950,875]],
    [[1000,100],[1200,250]],
    [[1800,100],[2000,250]]
]

Liste_noisette_xya = [
    [175,1125,0,"R"],[175,1175,0,"R"],[175,1225,0,"R"],[175,1275,0,"R"],
    [175,325,0,"R"],[175,375,0,"R"],[175,425,0,"R"],[175,475,0,"R"],

    [2825,1125,0,"R"],[2825,1175,0,"R"],[2825,1225,0,"R"],[2825,1275,0,"R"],
    [2825,325,0,"R"],[2825,375,0,"R"],[2825,425,0,"R"],[2825,475,0,"R"],

    [1075,800,90,"R"],[1125,800,90,"R"],[1175,800,90,"R"],[1225,800,90,"R"],
    [1775,800,90,"R"],[1825,800,90,"R"],[1875,800,90,"R"],[1925,800,90,"R"],

    [1025,175,90,"R"],[1075,175,90,"R"],[1125,175,90,"R"],[1175,175,90,"R"],
    [1825,175,90,"R"],[1875,175,90,"R"],[1925,175,90,"R"],[1975,175,90,"R"],

] 
Liste_noisette_xya_precedente = [noisette[:] for noisette in Liste_noisette_xya]  # Copie profonde
Liste_noisette_xya_cam = []
if not Wifi:
    Liste_noisette_xya_cam = [
        [175-20,1125+20,0+2,"B"],[175-20,1175+20,0+2,"J"],[175-20,1225+20,0+2,"B"],[175-20,1275+20,0+2,"J"],
        [175-20,325-20,0-2,"B"],[175-20,375-20,0-2,"B"],[175-20,425-20,0-2,"J"],[175-20,475-20,0-2,"J"],

        [2825+20,1125+20,0-2,"J"],[2825+20,1175+20,0-2,"B"],[2825+20,1275+20,0+2,"B"],[2825+20,1225+20,0+2,"J"],
        [2825+20,325-20,0-2,"B"],[2825+20,375-20,0-2,"J"],[2825+20,425-20,0+2,"J"],[2825+20,475-20,0+2,"B"],

        [1075-20,800-20,90+2,"J"],[1125-20,800-20,90-2,"J"],[1175-20,800-20,90+2,"B"],[1225-20,800-20,90-2,"B"],
        [1775+20,800-20,90+2,"J"],[1825+20,800-20,90-2,"B"],[1875+20,800-20,90+2,"B"],[1925+20,800-20,90-2,"J"],

        [1025-20,175-20,90-2,"B"],[1075-20,175-20,90+2,"B"],[1125-20,175-20,90-2,"J"],[1175-20,175-20,90+2,"J"],
        [1825+20,175-20,90-2,"B"],[1875+20,175-20,90+2,"J"],[1925+20,175-20,90-2,"J"],[1975+20,175-20,90+2,"B"],

    ] 
if not Wifi:
    code_couleur_noisette = ["A","E","C","F","B","E","D","C"]
else:
    code_couleur_noisette = ["N","N","N","N","N","N","N","N"]
Liste_association_Noisette_zone = [[] for i in range(8)]
for Noisette_posconnue in Liste_noisette_xya:
    for num_zonedepart in range(len(Liste_zones_Noisette_depart)):
        # Vérifier si la noisette est dans cette zone
        if (Liste_zones_Noisette_depart[num_zonedepart][0][0] < Noisette_posconnue[0] < Liste_zones_Noisette_depart[num_zonedepart][1][0] and 
            Liste_zones_Noisette_depart[num_zonedepart][0][1] < Noisette_posconnue[1] < Liste_zones_Noisette_depart[num_zonedepart][1][1]):
            Liste_association_Noisette_zone[num_zonedepart].append(Noisette_posconnue)
            
            

Noisette_init = False
Liste_association_Noisette_zone_cam = [[] for i in range(8)]
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

# Lidar
if current_os == "Linux":
    PORT_NAME = '/dev/ttyUSB0'
else:
    PORT_NAME = 'COM14'
BAUDRATE = 1000000
lidar = None

PRECISION = 2 # 0.5° = 2 points par degré
PORT = 12345
BUFFER_SIZE = 2048
NB_VALUES = 360 * PRECISION

# Un tableau de 720 points (x, y) initialisé à 10000
points = 10000*np.ones((NB_VALUES, 2))
encore = True

lock = threading.Lock()
####################################

# Configuration pour la réception
IP_RECEPTION = '0.0.0.0'
PORT_RECEPTION = 5001

# Configuration pour l'envoi vers la BC
IP_BC = "192.168.0.100"  # IP de la BC
PORT_ENVOI = 5000

# Coordonnées et angle de notre robot (coordonnées initiales en haut)
if couleur == "B":
    x_robot_depart = int(2825)
    y_robot_depart = int(1550+LONGUEUR_ROBOT/2+100)
    angle_robot_depart = -90

    x_robot_retour = 3000-LARGEUR_ROBOT/2-100
    y_robot_retour = 1800
    angle_robot_retour = -90
    
    x_fin_curseur = 2300
    y_fin_curseur = 150

    x_ennemi = 275
    y_ennemi = 1650
    
    if Strat_agressive:
        Liste_strategie = [
            [1200,800],
            [1100,800],
            [1550,800],
            [1450,800],
            
            [1050,175],
            [1150,175],
            [1550,100],
            [1450,100],
            
            [1950,175],
            [1850,175],
            [2350,100],
            [2250,100],
            
            [1900,800],
            [1800,800],
            [2250,800],
            [2150,800],

        ]
    else:
        Liste_strategie = [
            [2825,1250],
            [2825,1150],
            [2150,800],
            [2250,800],

            [2825,450],
            [2825,350],
            [2950,850],
            [2950,750],

            ["Recalage_X"],

            [1950,175],
            [1850,175],
            [2350,100],
            [1450,100],

        ]
 
else:
    x_robot_depart = int(175)
    y_robot_depart = int(1550+LONGUEUR_ROBOT/2+100)
    angle_robot_depart = -90
    
    x_robot_retour = LARGEUR_ROBOT/2+100
    y_robot_retour = 1800
    angle_robot_retour = -90

    x_fin_curseur = 700
    y_fin_curseur = 150

    x_ennemi = 2725 
    y_ennemi = 1650    
    
    if Strat_agressive:    
        Liste_strategie = [
            [1800,800],
            [1900,800],
            [1450,800],
            [1550,800],
            
            [1950,175],
            [1850,175],
            [1450,100],
            [1550,100],
            
            [1050,175],
            [1150,175],
            [650,100],
            [750,100],
            
            [1100,800],
            [1200,800],
            [750,800],
            [850,800],

        ]
    else:
        Liste_strategie = [
            [175,1250],
            [175,1150],
            [850,800],
            [750,800],

            [175,450],
            [175,350],
            [50,850],
            [50,750],

            ["Recalage_X"],

            [1050,175],
            [1150,175],
            [650,100],
            [1550,100],

        ]

if Faire_curseur:
    if not Strat_agressive:
        Liste_strategie.insert(9,['Curseur'])

x_robot_actuel = x_robot_depart
y_robot_actuel = y_robot_depart
angle_robot_actuel = angle_robot_depart

x_robot_voulu = x_robot_actuel
y_robot_voulu = y_robot_actuel
angle_robot_voulu = angle_robot_actuel
x_robot_voulu_prochain = x_robot_actuel
y_robot_voulu_prochain = y_robot_actuel

x_ennemi_lidar = x_ennemi
y_ennemi_lidar = y_ennemi

x_ennemi_old = x_ennemi
y_ennemi_old = y_ennemi
############

# Variable pour les Grilles du A*
CASE_MM = 10
width = X_PISTE//CASE_MM
height = Y_PISTE//CASE_MM
rayon_total_case = (R_ROBOT + MARGE_TRAJECTOIRE) // CASE_MM 
####################

# === CRÉATION DES OBSTACLES avec la classe Obstacles === #
obs_manager = Obstacles(X_PISTE,Y_PISTE,R_ROBOT-50,0,CASE_MM)
obs_manager_noisettes = Obstacles(X_PISTE,Y_PISTE,R_ROBOT-30,0,CASE_MM)

for i, noisette_data in enumerate(Liste_noisette_xya, 1):
    if len(noisette_data) >= 3:
        x, y, angle = noisette_data[0], noisette_data[1], noisette_data[2]
        obs_manager_noisettes.ajouter_rectangle_oriente(
            f"Noisette{i}", 
            (x, y),      # Centre
            150,         # Longueur (dans la direction de l'angle)
            50,          # Largeur (perpendiculaire)
            angle,       # Angle en degrés
            actif=True
        )

obs_manager.ajouter_rectangle("grenier", (600, 1550), (2400, 2000), actif=True)
# ======================================================= #

# === Création des Grilles pour A* ====================== #

# Générer les grilles séparément
grid_zones, grid_zones_expanded, obstacle_array_zones, expanded_array_zones = obs_manager.generer_grille()
grid_noisettes, grid_noisettes_expanded, obstacle_array_noisettes, expanded_array_noisettes = obs_manager_noisettes.generer_grille()

# Combiner les deux grilles (union logique OR)
grid = np.logical_or(grid_zones, grid_noisettes).astype(int)
grid_expanded = np.logical_or(grid_zones_expanded, grid_noisettes_expanded).astype(int)

# Combiner les arrays d'obstacles pour l'affichage
obstacle_array = np.vstack([obstacle_array_zones, obstacle_array_noisettes]) if len(obstacle_array_zones) > 0 and len(obstacle_array_noisettes) > 0 else (obstacle_array_zones if len(obstacle_array_zones) > 0 else obstacle_array_noisettes)
expanded_array = np.vstack([expanded_array_zones, expanded_array_noisettes]) if len(expanded_array_zones) > 0 and len(expanded_array_noisettes) > 0 else (expanded_array_zones if len(expanded_array_zones) > 0 else expanded_array_noisettes)
# ======================================================= #

# Variables fonctionnelles des Batteries
Batteries = [random.randint(10, 100),random.randint(10, 100),random.randint(10, 100),random.randint(10, 100)] # V décharge, V charge, V actuel, % de charge
Batteries_interrupteur = [1,1,1]
Batteries_alert = [0,0,0,0]
RPI_decharge = False

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
step = 0

carte_asserv_active = 0
carte_actionneur0_active = 0
carte_actionneur1_active = 0
carte_batteries_active = 0
etat_bau = 0
etat_jack = 1

demande_nouvelle_strat = True
x_strategie = 0
y_strategie = 0
strategie_en_cours = [] 

action_voulu = None
action_precedente = None
aller_Noisette = False
aller_GM = False

temps_demarage = 0
temps_ecoules = 0
temps_restant = 100
temps_retour = 8 # Temps restant pour revenir au départ en fin de match
temps_max = 100
reset_fin = False

verif_mouv = 0
verif_mouv_rpi = 0
old_verif_mouv = verif_mouv
verif_angle = 0
verif_action = 0
verif_action1 = 0
verif_action2 = 0
verif_recalage = 0
verif_recalageX = 0
verif_recalageY = 0
verif_curseur = 0

action_est_supprime = False 
mode_attraper = False

pince_a_utilise = -1
noisette_a_manipulee = 0
Pince_Avant = True # Pas à pas ou herkulex écarter pince marchent pas : pas possibe de retourner
Pince_Av_1 = True # Serrer intérieur , marche pas
Pince_Av_2 = True # Serrer extérieur 
Pince_Arriere = True  # Pas à pas ou herkulex écarter pince marchent pas : pas possibe de retourner
Pince_Ar_1 = True # Serrer intérieur, marche pas
Pince_Ar_2 = True # Serrer extérieur 

demande_recalcul_traj = False
curseur_fait = False
Astars_a_fail = False
calcul_astar = False
calcul_fait = False
ordre_mouvement = 0
old_ordre_mouvement = 0

x_sortie_fixe = None
y_sortie_fixe = None

distance_robot_consigne = 3000
# ==================================================

# ==================== PARAMÈTRES DE L'ALGORITHME ====================

SAFETY_WEIGHT = 2.0  # Poids de sécurité pour A*
MIN_CLEARANCE = 3.0
SMOOTHNESS = 1.0
DISTANCE_AJUSTABLE = 5  # Distance seuil pour pénalité sécurité (en cases)

# Expansion des obstacles
from scipy.ndimage import binary_dilation
structure = np.zeros((2*rayon_total_case+1, 2*rayon_total_case+1))
y_grid, x_grid = np.ogrid[-rayon_total_case:rayon_total_case+1, -rayon_total_case:rayon_total_case+1]
mask = x_grid*x_grid + y_grid*y_grid <= rayon_total_case*rayon_total_case
structure[mask] = 1
grid_expanded = binary_dilation(grid, structure=structure)

# Carte de distance aux obstacles (pour A* pondéré)
from scipy.ndimage import distance_transform_edt
distance_map = distance_transform_edt(~grid_expanded)

x_robot_actuel_cam = x_robot_depart
y_robot_actuel_cam = y_robot_depart
y_robot_actuel_cam_corr = y_robot_depart
angle_robot_actuel_cam = angle_robot_depart

x_ennemi_cam = x_ennemi
y_ennemi_cam = y_ennemi
angle_ennemi_cam = 0

# Identifiant unique de match (généré au démarrage)
match_id = str(uuid.uuid4())[:8]  # Ex: "a3f2b891"
match_demarre = False
dernier_envoi_debut_match = 0
INTERVALLE_ENVOI_DEBUT = 0.5  # Envoyer signal toutes les 0.5s pendant 5s
print(f"🎲 Match ID généré : {match_id}")

queue_demande_astar = queue.Queue()  # Pour envoyer des demandes
queue_resultat_astar = queue.Queue()  # Pour recevoir les résultats

Liste_Noisette_temps_cam = [[] for i in range(temps_max)]
################## Fonction Threads ##########################################
def lidar_udp(stop_event):

    global  x_robot_actuel, y_robot_actuel, angle_robot_actuel,x_ennemi_lidar, y_ennemi_lidar,x_robot_depart, y_robot_depart, angle_robot_depart, Reel
    
    # 1. Création du socket UDP
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(1.0)  # 1 seconde

    # 2. Liaison à une adresse et un port
    sock.bind(("0.0.0.0", PORT))  # écoute sur toutes les interfaces

    print(f"Serveur UDP en attente sur le port {PORT}...")

    # Conversion de l'angle en radians pour les calculs

    while not stop_event.is_set():
        
        if stop_event.is_set():
            break

        # 3. Réception du datagramme
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
        except socket.timeout:
            data = bytes(NB_VALUES * 2)  # Remplir de zéros en cas aucune données reçues

        if len(data) != NB_VALUES * 2:  # Chaque valeur est un uint16 (2 bytes)
            data = bytes(NB_VALUES * 2)  # Remplir de zéros en cas de données incorrectes

        # '720H' = 720 unsigned short (uint16)
        tableau = np.frombuffer(data, dtype='<u2', count=NB_VALUES)

        for i in range(len(tableau)):
            if tableau[i] == 0: #mettre le point loin
                points[i,0] = 10000
            else:
                # Calcul dans le repère du LIDAR
                angle_robot_rad = np.deg2rad(angle_robot_actuel)
                cos_angle = np.cos(angle_robot_rad)
                sin_angle = np.sin(angle_robot_rad)
                angle = -(i+0.5)*((np.pi/180.0)/PRECISION) #angle en radians
                x_lidar = tableau[i]*np.cos(angle)
                y_lidar = tableau[i]*np.sin(angle)
                
                # Transformation vers le repère global
                # Rotation selon l'orientation du robot
                x_rotated = x_lidar * cos_angle - y_lidar * sin_angle
                y_rotated = x_lidar * sin_angle + y_lidar * cos_angle
                
                # Translation selon la position du robot
                x_point = x_rotated + x_robot_actuel
                y_point = y_rotated + y_robot_actuel
                
                # Clamping des coordonnées
                x_point = max(0, min(X_PISTE, int(x_point)))
                y_point = max(0, min(Y_PISTE, int(y_point)))
                
                # Calcul de la distance entre le robot et le point
                distance_robot_point = np.sqrt((x_robot_actuel - x_point)**2 + (y_robot_actuel - y_point)**2)
                
                # Vérification des conditions de validité
                
                if MARGE_BORDUREPISTE_X <= x_point <= X_PISTE-MARGE_BORDUREPISTE_X and \
                   MARGE_BORDUREPISTE_Y <= y_point <= Y_PISTE-MARGE_BORDUREPISTE_Y and \
                   distance_robot_point > DISTANCE_MIN_ROBOT and not (600<x_point<2400 and 1550<y_point<2000):
                    # Point valide
                    points[i,0] = x_point
                    points[i,1] = y_point
                else:
                    # Point ignoré (trop proche des bords ou du robot)
                    points[i,0] = 10000
                    points[i,1] = 10000
        
        # Calcul de la position moyenne de l'ennemi (moyenne des points valides)
        with lock:
            points_valides = points[points[:, 0] != 10000]

            if len(points_valides) > 0:
                distances = np.sqrt((points_valides[:, 0] - x_robot_actuel)**2 + 
                           (points_valides[:, 1] - y_robot_actuel)**2)
                
                index_min = np.argmin(distances)
                x_ennemi_lidar = points_valides[index_min, 0]
                y_ennemi_lidar = points_valides[index_min, 1]
                
            else:
                # Aucun point valide détecté - placer hors du graphique
                x_ennemi_lidar = -1000
                y_ennemi_lidar = -1000


def LectureCAN(stop_event):
    """
    Argument : flag "stop_event"
    Modification : variables globales x_robot, y_robot, angle_robot, Batteries

    Utilisation :
    Lis le Bus CAN
    Si l'ID du message n'est pas dans la Liste_ID, saute
    Sinon, met à jour les coordonées et angle du robot, valeurs des batteries
    """
    global x_robot_actuel, y_robot_actuel, angle_robot_actuel, Liste_ID_recoit, Batteries, bus, verif_mouv, verif_angle,verif_action1,verif_action2,Batteries_alert, carte_actionneur0_active, carte_actionneur1_active, carte_asserv_active, carte_batteries_active, etat_bau,verif_recalage,verif_curseur, verif_recalageX, verif_recalageY
    while not stop_event.is_set():
        msg = bus.recv(0.05)
        if msg is None:
            continue  # pas de message, on repart

        # Vérifie qu'on a bien reçu 4 octets avant de décoder
        if msg.arbitration_id not in Liste_ID_recoit:
            continue  # on saute les autres trames

        if msg.arbitration_id == 0x100:
            x_robot_actuel = struct.unpack('f', bytes(msg.data))[0]
        elif msg.arbitration_id == 0x101:
            y_robot_actuel = struct.unpack('f', bytes(msg.data))[0]
        elif msg.arbitration_id == 0x102:
            angle_robot_actuel = struct.unpack('f', bytes(msg.data))[0]

        # Batteries
        elif msg.arbitration_id == 0x103:
            Batteries[0] = struct.unpack('<H', bytes(msg.data[:2]))[0]
        elif msg.arbitration_id == 0x104:
            Batteries[1] = struct.unpack('<H', bytes(msg.data[:2]))[0]
        elif msg.arbitration_id == 0x105:
            Batteries[2] = struct.unpack('<H', bytes(msg.data[:2]))[0]
        elif msg.arbitration_id == 0x10A:
            Batteries[3] = struct.unpack('<H', bytes(msg.data[:2]))[0]

        elif msg.arbitration_id == 0x106:
            Batteries_alert[0] = struct.unpack('<H', bytes(msg.data[:2]))[0]
        elif msg.arbitration_id == 0x107:
            Batteries_alert[1] = struct.unpack('<H', bytes(msg.data[:2]))[0]
        elif msg.arbitration_id == 0x108:
            Batteries_alert[2] = struct.unpack('<H', bytes(msg.data[:2]))[0]
        elif msg.arbitration_id == 0x10B:
            Batteries_alert[3] = struct.unpack('<H', bytes(msg.data[:2]))[0]
            
        elif msg.arbitration_id == 0x10C:
            if msg.dlc >= 1:
                verif_action1 = msg.data[0]

        elif msg.arbitration_id == 0x10D:
            if msg.dlc >= 1:
                verif_action2 = msg.data[0]

        elif msg.arbitration_id == 0x10E:
            verif_mouv = struct.unpack('f', bytes(msg.data))[0]
        elif msg.arbitration_id == 0x10F:
            verif_angle = struct.unpack('f', bytes(msg.data))[0]
            
        elif msg.arbitration_id == 0x002:
            carte_asserv_active = struct.unpack('f', bytes(msg.data))[0]
        elif msg.arbitration_id == 0x003:
            if msg.dlc >= 1:
                carte_actionneur0_active = msg.data[0]
        elif msg.arbitration_id == 0x004:
            if msg.dlc >= 1:
                carte_actionneur1_active = msg.data[0]

        elif msg.arbitration_id == 0x005:
            carte_batteries_active = struct.unpack('<H', bytes(msg.data[:2]))[0]
        elif msg.arbitration_id == 0x06:
            if msg.dlc >= 2:
                etat_bau = struct.unpack('<H', bytes(msg.data[:2]))[0]
            else:
                etat_bau = msg.data[0]
                
        elif msg.arbitration_id == 0x110:
            verif_recalage = struct.unpack('f', bytes(msg.data))[0]

        elif msg.arbitration_id == 0x008:
            if msg.dlc >= 1:
                verif_curseur = msg.data[0]
                
        elif msg.arbitration_id == 0x111:
            verif_recalageX = struct.unpack('f', bytes(msg.data))[0]
        elif msg.arbitration_id == 0x112:
            verif_recalageY = struct.unpack('f', bytes(msg.data))[0]
    
# Fonction pour recevoir des données de la RPI
def comm_bc(stop_event):
    print(f"[Récepteur] Serveur en attente sur le port {PORT_RECEPTION}...")
    global Liste_noisette_xya_cam,x_robot_actuel_cam,y_robot_actuel_cam,angle_robot_actuel_cam,x_ennemi_cam,y_ennemi_cam,angle_ennemi_cam,code_couleur_noisette

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
                donnees_de_bc = json.loads(data.decode())
                
                Liste_noisette_xya_cam = donnees_de_bc["Liste_noisette_xya_cam"]
                x_robot_actuel_cam = donnees_de_bc["x_robot_actuel_cam"]
                y_robot_actuel_cam = donnees_de_bc["y_robot_actuel_cam"]
                angle_robot_actuel_cam = donnees_de_bc["angle_robot_actuel_cam"]
                x_ennemi_cam = donnees_de_bc["x_ennemi_cam"]
                y_ennemi_cam = donnees_de_bc["y_ennemi_cam"]
                angle_ennemi_cam  = donnees_de_bc["angle_ennemi_cam"]
                code_couleur_noisette = donnees_de_bc["code_couleur_noisette"] 
                
            except json.JSONDecodeError:
                print("[Récepteur] Erreur : données JSON invalides")
            
    except KeyboardInterrupt:
        print("[Récepteur] Arrêt.")
    finally:
        server_socket.close()


def calcul_traj(stop_event):
    """Thread qui attend des demandes et calcule les trajectoires"""
    global obs_manager, obs_manager_noisettes, x_ennemi, y_ennemi, ax, CASE_MM, X_PISTE, Y_PISTE, SAFETY_WEIGHT, MIN_CLEARANCE, SMOOTHNESS, DISTANCE_AJUSTABLE
    while not stop_event.is_set():
        try:
            # Attendre une demande (bloquant avec timeout)
            demande = queue_demande_astar.get(timeout=0.5)
            
            if demande is None:  # Signal d'arrêt
                break
            
            x_depart, y_depart, x_cible, y_cible = demande
            
            # ⭐ Calcul A* (longue opération)
            points_bruts = calculer_trajectoire_complete(
                x_depart, y_depart, x_cible, y_cible,
                obs_manager, obs_manager_noisettes,
                x_ennemi, y_ennemi, R_securite,
                CASE_MM, X_PISTE, Y_PISTE,
                SAFETY_WEIGHT, MIN_CLEARANCE,
                SMOOTHNESS, DISTANCE_AJUSTABLE,
                affichage_ax=ax
            )
            
            # Envoyer le résultat
            queue_resultat_astar.put(points_bruts)
            
            queue_demande_astar.task_done()
            
        except queue.Empty:
            continue  # Timeout, on reboucle pour vérifier stop_event
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
    global verif_action1,verif_action2,verif_action, action_voulu,verif_curseur,verif_recalageX,verif_recalageY
    
    # Vérifier qu il y a une action en cours
    if len(Liste_actions) > 0 and action_voulu in ["Attraper","Retourner","Relacher"]:
        verif_action1 = 1
        verif_action2 = 1
    if len(Liste_actions) > 0 and action_voulu in ["Attente_test"]:
        verif_action = 1
    if len(Liste_actions) > 0 and action_voulu in ["Curseur_Bleu","Curseur_Jaune"]:
        verif_curseur = 1
    if len(Liste_actions) > 0 and action_voulu in ["Recalage_X"]:
        verif_recalageX = 1
    if len(Liste_actions) > 0 and action_voulu in ["Recalage_Y"]:
        verif_recalageY = 1


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




def trouver_case_libre_proche(x_robot, y_robot, grid_expanded, 
                               CASE_MM, width, height, rayon_max_mm=500):
    """
    Cherche la case libre la plus proche du robot dans grid_expanded.
    Utilise une recherche en cercles concentriques case par case.
    Retourne (x_mm, y_mm) ou (None, None) si rien trouvé.
    """
    x_case = max(0, min(width - 1,  int(x_robot // CASE_MM)))
    y_case = max(0, min(height - 1, int(y_robot // CASE_MM)))
    
    rayon_max_cases = rayon_max_mm // CASE_MM

    for rayon in range(1, rayon_max_cases + 1):
        # Parcourir le périmètre du carré de rayon `rayon`
        for dx in range(-rayon, rayon + 1):
            for dy in range(-rayon, rayon + 1):
                # Ne tester que le bord du carré (périmètre)
                if abs(dx) != rayon and abs(dy) != rayon:
                    continue
                nx = x_case + dx
                ny = y_case + dy
                if 0 <= nx < width and 0 <= ny < height:
                    if not grid_expanded[nx, ny]:
                        # Convertir en mm (centre de la case)
                        return (nx * CASE_MM + CASE_MM // 2,
                                ny * CASE_MM + CASE_MM // 2)
    return None, None

def comm_PAMI(stop_event):
    global match_demarre,couleur, temps_restant
    old_match_demarre = match_demarre
    decompte = False
    int_temps_restant_old = 100
    while not stop_event.is_set():
        
        if stop_event.is_set():
            break
        
        if not match_demarre:
            if couleur == "B":
                sendPamis(1, 255, "192.168.0.255")
            if couleur == "J":
                sendPamis(2, 255, "192.168.0.255")
        else:
            if match_demarre != old_match_demarre:
                decompte = True
            if decompte:
                int_temps_restant = int(temps_restant)
                if int_temps_restant != int_temps_restant_old:
                    if couleur == "B":
                        sendPamis(1, int_temps_restant, "192.168.0.255")
                    if couleur == "J":
                        sendPamis(2, int_temps_restant, "192.168.0.255")

                int_temps_restant_old = int_temps_restant
        old_match_demarre = match_demarre
        time.sleep(0.1)  # Petite pause pour éviter de saturer le CPU



def sendPamis(couleur, temps = 255, UDP_IP = "192.168.0.255"):
    UDP_PORT = 5002
    # message de 2 octets
    MESSAGE = bytes([couleur, temps])
    # print("UDP target IP: %s" % UDP_IP)
    # print("UDP target port: %s" % UDP_PORT)
    # print("message: %s" % MESSAGE)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Enable broadcast
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

########################################################################

############################### Programme principal ####################

if __name__ == '__main__':
    try:
        mp.set_start_method('spawn')
    except RuntimeError:
        pass

    stop_event = threading.Event()

    if Reel :
        tache_LectureCAN = threading.Thread(
            target=LectureCAN, 
            args=(stop_event,),
            daemon=True
        )
        tache_LectureCAN.start()

    if Wifi:
        tache_Wifi = threading.Thread(target=comm_bc, args=(stop_event,),daemon=True)
        tache_Wifi.start()

        tache_PAMI = threading.Thread(target=comm_PAMI, args=(stop_event,),daemon=True)
        tache_PAMI.start()

    fig, ax, robot_plot, ennemi_plot, consigne_plot, scat, robot_info_text,ax_button_stop,bouton_stop,ax_button_start,bouton_start,point_voulu_plot,x_voulu_text,y_voulu_text,A_voulu_text, robot_angle_line,robot_angle_voulu_line,background,info_alim_rpi,chronometre_text,cercle_robot_patch,noisette_text,match_text = init_affichage(x_robot_depart,y_robot_depart,R_ROBOT)
    
    cid = fig.canvas.mpl_connect('button_press_event', on_click) # Choix des coordonnées voulues avec la souris
    bouton_stop.on_clicked(partial(arret_programme, stop_event=stop_event))
    bouton_start.on_clicked(demarrage_strategie)  # ⭐ Connexion du bouton START ⭐
    
    obstacle_scatter, expanded_scatter = afficher_obstacles(ax,obstacle_array,expanded_array,CASE_MM,show_expanded=True,show_obstacles=False)
    zone_ennemi_scatter, cercle_ennemi_patch = afficher_zone_securite_ennemi(ax,x_ennemi, y_ennemi,R_ROBOT, R_ENNEMI,MARGE_ENNEMI,CASE_MM,X_PISTE,Y_PISTE,show_zone=True,show_cercle=False)
    
    # ========== DESSIN DES NOISETTES AVEC COULEURS ==========
    patches_noisettes = dessiner_noisettes(ax, Liste_noisette_xya,longueur=150, largeur=50,alpha=0.7, linewidth=2)
    # ========================================================

    grid, grid_expanded, obstacle_array, expanded_array, \
    obs_manager, obs_manager_noisettes, \
    obstacle_scatter, expanded_scatter, distance_map, \
    ax, width, height, CASE_MM = actualiser_zones_jeu(
        grid, grid_expanded, obstacle_array, expanded_array,
        obs_manager, obs_manager_noisettes,
        Liste_noisette_xya,  # ⭐ NOUVEAU
        obstacle_scatter, expanded_scatter, distance_map,
        ax, width, height, CASE_MM
    )
    # création du trait, initialement à la position du robot
    ax.add_line(robot_angle_line)
    ax.add_line(robot_angle_voulu_line)
    
    # Bouton ATTRAPER
    ax_attraper_button = plt.axes([0.38, 0.92, 0.12, 0.08])
    bouton_attraper = Button(ax_attraper_button, "Action", color="lightblue", hovercolor="blue")
    bouton_attraper.on_clicked(bouton_attraper_callback)
    
    #carte_tolerance = afficher_profils_tolerance(ax)
    groupes_initiaux, adjacence = trouver_groupes_initiaux(Liste_noisette_xya)
    Noisettes_groupees = []
    for groupe in groupes_initiaux:
        paires = separer_groupe(groupe, Liste_noisette_xya, adjacence)
        for paire in paires:
            noisettes_paire = [Liste_noisette_xya[i] for i in paire]
            Noisettes_groupees.append(noisettes_paire)
            

    changement_noisettes_detecte = detecter_changements_noisettes(
        Liste_noisette_xya, 
        Liste_noisette_xya_precedente
    )

    if changement_noisettes_detecte:
        print("🔄 Changement détecté dans Liste_noisette_xya - Mise à jour des grilles")
        
        grid, grid_expanded, obstacle_array, expanded_array, \
        obs_manager, obs_manager_noisettes, \
        obstacle_scatter, expanded_scatter, distance_map, \
        ax, width, height, CASE_MM = actualiser_zones_jeu(
            grid, grid_expanded, obstacle_array, expanded_array,
            obs_manager, obs_manager_noisettes,
            Liste_noisette_xya,  # ⭐ NOUVEAU
            obstacle_scatter, expanded_scatter, distance_map,
            ax, width, height, CASE_MM
        )
        
        # Forcer le recalcul de trajectoire
        demande_recalcul_traj = True

    
    if Reel and lancement_cartes:
        while(carte_asserv_active == 0 or carte_actionneur0_active == 0 or carte_actionneur1_active == 0 or carte_batteries_active == 0):
            if carte_asserv_active == 0:
                print(f"Carte Asserv Manquante")
            if carte_actionneur0_active == 0:
                print(f"Carte Actionneur 0 Manquante")
            if carte_actionneur1_active == 0:
                print(f"Carte Actionneur 1 Manquante")
            if carte_batteries_active == 0:
                print(f"Carte Batteries Manquante")
            time.sleep(0.1)
            
    if Reel:
        while(etat_bau == 1):
            print(f"Attente BAU")
            time.sleep(0.1)
        
        if couleur == "B":
            dico_envoi[0x01]=1
            dico_envoi[0x200]=x_robot_depart
            dico_envoi[0x201]=y_robot_depart
            dico_envoi[0x202]=angle_robot_depart
            bus.send(can.Message(arbitration_id=0x01, data=struct.pack('<i',dico_envoi[0x01]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x200, data=struct.pack('<f',dico_envoi[0x200]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x201, data=struct.pack('<f',dico_envoi[0x201]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x202, data=struct.pack('<f',dico_envoi[0x202]), is_extended_id=False))
            dico_envoi[0x200]=0
            dico_envoi[0x201]=0
            dico_envoi[0x202]=0
            time.sleep(0.1)
            dico_envoi[0x01]=1
            dico_envoi[0x200]=x_robot_depart
            dico_envoi[0x201]=y_robot_depart
            dico_envoi[0x202]=angle_robot_depart
            bus.send(can.Message(arbitration_id=0x01, data=struct.pack('<i',dico_envoi[0x01]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x200, data=struct.pack('<f',dico_envoi[0x200]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x201, data=struct.pack('<f',dico_envoi[0x201]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x202, data=struct.pack('<f',dico_envoi[0x202]), is_extended_id=False))
            dico_envoi[0x200]=0
            dico_envoi[0x201]=0
            dico_envoi[0x202]=0

        if couleur == "J":
            dico_envoi[0x01]=1
            dico_envoi[0x200]=x_robot_depart
            dico_envoi[0x201]=y_robot_depart
            dico_envoi[0x202]=90
            bus.send(can.Message(arbitration_id=0x01, data=struct.pack('<i',dico_envoi[0x01]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x200, data=struct.pack('<f',dico_envoi[0x200]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x201, data=struct.pack('<f',dico_envoi[0x201]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x202, data=struct.pack('<f',dico_envoi[0x202]), is_extended_id=False))
            dico_envoi[0x200]=0
            dico_envoi[0x201]=0
            dico_envoi[0x202]=0
            time.sleep(0.1)
            dico_envoi[0x01]=1
            dico_envoi[0x200]=x_robot_depart
            dico_envoi[0x201]=y_robot_depart
            dico_envoi[0x202]=90
            bus.send(can.Message(arbitration_id=0x01, data=struct.pack('<i',dico_envoi[0x01]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x200, data=struct.pack('<f',dico_envoi[0x200]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x201, data=struct.pack('<f',dico_envoi[0x201]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x202, data=struct.pack('<f',dico_envoi[0x202]), is_extended_id=False))
            dico_envoi[0x200]=0
            dico_envoi[0x201]=0
            dico_envoi[0x202]=0
        time.sleep(2)

        if Recalage: 
            recalage_depart = False
            
            while(recalage_depart == False and not stop_event.is_set()):
                if etat_bau == 1:
                    stop_event.set()
                print("Recalage de départ en cours")
                print("X : ",x_robot_actuel," Y : ",y_robot_actuel," A : ",angle_robot_actuel)
                bus.send(can.Message(arbitration_id=0x01, data=struct.pack('<i',1), is_extended_id=False))
                if couleur == "B":
                    bus.send(can.Message(arbitration_id=0x206, data=struct.pack('<i',7), is_extended_id=False))
                if couleur == "J":
                    bus.send(can.Message(arbitration_id=0x206, data=struct.pack('<i',8), is_extended_id=False))

                if verif_recalage == 1:
                    recalage_depart = True
                    bus.send(can.Message(arbitration_id=0x209, data=struct.pack('<i',2), is_extended_id=False))
                else :
                    bus.send(can.Message(arbitration_id=0x209, data=struct.pack('<i',1), is_extended_id=False))

                time.sleep(0.1)

            pos_depart = False
            ordre_mouvement=5
            while(pos_depart == False and not stop_event.is_set()):
                if etat_bau == 1:
                    stop_event.set()
                print("X et Y de départ en cours")
                print("X : ",x_robot_actuel," Y : ",y_robot_actuel," A : ",angle_robot_actuel)
                bus.send(can.Message(arbitration_id=0x203, data=struct.pack('<f',x_robot_depart), is_extended_id=False))
                bus.send(can.Message(arbitration_id=0x204, data=struct.pack('<f',y_robot_depart), is_extended_id=False))
                bus.send(can.Message(arbitration_id=0x209, data=struct.pack('<f',x_robot_depart), is_extended_id=False))
                bus.send(can.Message(arbitration_id=0x20A, data=struct.pack('<f',y_robot_depart), is_extended_id=False))
                bus.send(can.Message(arbitration_id=0x206, data=struct.pack('<i',5), is_extended_id=False))

                if verif_mouv == 1:
                    bus.send(can.Message(arbitration_id=0x207, data=struct.pack('<i',2), is_extended_id=False))
                    pos_depart = True
                else :
                    bus.send(can.Message(arbitration_id=0x207, data=struct.pack('<i',1), is_extended_id=False))
                    
                time.sleep(0.1)

            angle_depart = False
            while(angle_depart == False and not stop_event.is_set()):
                if etat_bau == 1:
                    stop_event.set()
                print("Angle de départ en cours")
                print("X : ",x_robot_actuel," Y : ",y_robot_actuel," A : ",angle_robot_actuel)
                bus.send(can.Message(arbitration_id=0x205, data=struct.pack('<f',angle_robot_depart+360), is_extended_id=False))
                bus.send(can.Message(arbitration_id=0x206, data=struct.pack('<i',4), is_extended_id=False))

                if verif_angle == 1:
                    bus.send(can.Message(arbitration_id=0x208, data=struct.pack('<i',2), is_extended_id=False))
                    angle_depart = True
                else :
                    bus.send(can.Message(arbitration_id=0x208, data=struct.pack('<i',1), is_extended_id=False))
                    
                time.sleep(0.1)

    while(lancement_strategie==False and etat_jack == 1 and not stop_event.is_set()):
        if current_os == "Linux":
            etat_jack = GPIO.input(26)
        dico_envoi[0x01]=1
        if not Bat_Compet:                      # Si on est en mode Test
            if Batteries_alert[0]==0:           #   Si il n'y a pas de message d'alerte pour la batterie
                Batteries_interrupteur[0]=2     #     On met l'interrupteur à 1
            else :                              #   Sinon   
                Batteries_interrupteur[0]=1     #     On met l'interrupteur à 2

            if Batteries_alert[1]==0:
                Batteries_interrupteur[1]=2
            else :
                Batteries_interrupteur[1]=1
                
            if Batteries_alert[2]==0:
                Batteries_interrupteur[2]=2
            else :
                Batteries_interrupteur[2]=1
        else :                                  # Sinon
            Batteries_interrupteur[0]=2         #  On met tous les interrupteurs à 1
            Batteries_interrupteur[1]=2
            Batteries_interrupteur[2]=2  

        if Batteries_alert[3]==1 and Bat_Compet == False:
            RPI_decharge = True
        else :
            RPI_decharge = False
            
        dico_envoi[0x300]=Batteries_interrupteur[0]
        dico_envoi[0x301]=Batteries_interrupteur[1]
        dico_envoi[0x302]=Batteries_interrupteur[2]

        if Bat_Compet:
            dico_envoi[0x303]=1
        else:
            dico_envoi[0x303]=2

        if Reel :
            bus.send(can.Message(arbitration_id=0x001, data=struct.pack('<i',dico_envoi[0x001]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x300, data=struct.pack('<i',dico_envoi[0x300]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x301, data=struct.pack('<i',dico_envoi[0x301]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x302, data=struct.pack('<i',dico_envoi[0x302]), is_extended_id=False))
            bus.send(can.Message(arbitration_id=0x303, data=struct.pack('<i',dico_envoi[0x303]), is_extended_id=False))

            if etat_bau == 1:
                stop_event.set()
        print("Attente du Jack")
        if Wifi:
            donnees_vers_bc = {
                "match_id": match_id,
                "match_demarre": match_demarre,  # ⚠️ Match pas encore lancé
                "x_robot_actuel": x_robot_actuel,
                "y_robot_actuel": y_robot_actuel,
                "angle_robot_actuel": angle_robot_actuel,
                "x_ennemi": x_ennemi,
                "y_ennemi": y_ennemi,
                "Batteries": Batteries,
                "Batteries_alert": Batteries_alert,
                "Noisettes_stockees_dans_robot": Noisettes_stockees_dans_robot,
                "action_voulu" : action_voulu,
                "action_precedente" : action_precedente,
                "step_robot" : step,
            }
            try:
                message = json.dumps(donnees_vers_bc)
                client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client_socket.settimeout(0.1)
                client_socket.connect((IP_BC, PORT_ENVOI))
                client_socket.sendall(message.encode())
                client_socket.close()
            except (socket.timeout, ConnectionRefusedError, OSError) as e:
                print(f"WiFi Envoi échoué : {e}")
            
        plt.pause(0.1)
    
    temps_demarage = time.time()
    match_demarre = True

    if not affichage:
        plt.close(fig)

    try:
        if Lidar_on:
            tache_Lidar = threading.Thread(
                target=lidar_udp, 
                args=(stop_event,),
                daemon=True
            )
            tache_Lidar.start()

        if Astars:
            tache_Astar = threading.Thread(target=calcul_traj, args=(stop_event,), daemon=False)
            tache_Astar.start()

        n = random.randint(20, 60)
        Liste_actions_ennemi = [[300,800],[1500,1000]]
        if not Strat_agressive:
            Liste_actions_ennemi.append([650,200])
        for i in range(n):
            Liste_actions_ennemi.append([random.randint(100, 2900),random.randint(100, 1900)])
        n_init = len(Liste_actions_ennemi)

        while (not stop_event.is_set() and temps_restant >=0 and not RPI_decharge and not etat_bau): # Tant que le Flag de Thread n'est pas levé, que la batterie RPI est suffisamment chargées, qu'il y a encore des actions à réaliser, que le BAU n'est pas appuyé
            dico_envoi[0x01]=1

            if Wifi:
                print(f"Recu du WiFi : x={x_robot_actuel_cam}, y={y_robot_actuel_cam}, angle={angle_robot_actuel_cam}")    

                if 1000 <= y_robot_actuel_cam <= 2000:
                    # Formule d'erreur : erreur_y = (y / 1500) × 50
                    erreur_y = (y_robot_actuel_cam / 1500.0) * 40.0
                    
                    # Corriger en soustrayant l'erreur
                    y_corrige = y_robot_actuel_cam + erreur_y
                    y_robot_actuel_cam_corr = int(round(y_corrige))
                    print(f"Après : x={x_robot_actuel_cam}, y={y_robot_actuel_cam_corr}, angle={angle_robot_actuel_cam}") 
                else:
                    y_robot_actuel_cam_corr = y_robot_actuel_cam
                    print(f"Après : x={x_robot_actuel_cam}, y={y_robot_actuel_cam_corr}, angle={angle_robot_actuel_cam}")

            temps_ecoules = time.time() - temps_demarage
            temps_restant = temps_max - temps_ecoules

            step +=1
            print("step :",step)
            
            # =============== Association Couleur CAM à Noisette Aveugle ==================== #
            if not Noisette_init:
                for i in range(8):
                    print(f"Code de la zone N°{i} : {code_couleur_noisette[i]}")
                    n1 = Liste_noisette_xya[4*i]
                    n2 = Liste_noisette_xya[4*i+1]
                    n3 = Liste_noisette_xya[4*i+2]
                    n4 = Liste_noisette_xya[4*i+3]
                    if code_couleur_noisette[i] != "N" and n1[3]=="R":
                        if 0<=i<=3:
                            if code_couleur_noisette[i] == "A":
                                n1[3]="J"
                                n2[3]="J"
                                n3[3]="B"
                                n4[3]="B"
                            if code_couleur_noisette[i] == "B":
                                n1[3]="B"
                                n2[3]="B"
                                n3[3]="J"
                                n4[3]="J"
                            if code_couleur_noisette[i] == "C":
                                n1[3]="B"
                                n2[3]="J"
                                n3[3]="J"
                                n4[3]="B"
                            if code_couleur_noisette[i] == "D":
                                n1[3]="J"
                                n2[3]="B"
                                n3[3]="B"
                                n4[3]="J"
                            if code_couleur_noisette[i] == "E":
                                n1[3]="J"
                                n2[3]="B"
                                n3[3]="J"
                                n4[3]="B"
                            if code_couleur_noisette[i] == "F":
                                n1[3]="B"
                                n2[3]="J"
                                n3[3]="B"
                                n4[3]="J"
                        else:
                            if code_couleur_noisette[i] == "A":
                                n1[3]="B"
                                n2[3]="B"
                                n3[3]="J"
                                n4[3]="J"
                            if code_couleur_noisette[i] == "B":
                                n1[3]="J"
                                n2[3]="J"
                                n3[3]="B"
                                n4[3]="B"
                            if code_couleur_noisette[i] == "C":
                                n1[3]="B"
                                n2[3]="J"
                                n3[3]="J"
                                n4[3]="B"
                            if code_couleur_noisette[i] == "D":
                                n1[3]="J"
                                n2[3]="B"
                                n3[3]="B"
                                n4[3]="J"
                            if code_couleur_noisette[i] == "E":
                                n1[3]="B"
                                n2[3]="J"
                                n3[3]="B"
                                n4[3]="J"
                            if code_couleur_noisette[i] == "F":
                                n1[3]="J"
                                n2[3]="B"
                                n3[3]="J"
                                n4[3]="B"
                Noisette_restantes = [n for n in Liste_noisette_xya if n[3]=="R"]

                lettres = ["A","B","C","D","E","F"]
                if temps_ecoules > 1.5:
                    for Nois in Noisette_restantes:
                        indice = None
                        for num_zonedepart in range(len(Liste_zones_Noisette_depart)):
                            # Vérifier si la noisette est dans cette zone
                            if (Liste_zones_Noisette_depart[num_zonedepart][0][0] < Nois[0] < Liste_zones_Noisette_depart[num_zonedepart][1][0] and 
                                Liste_zones_Noisette_depart[num_zonedepart][0][1] < Nois[1] < Liste_zones_Noisette_depart[num_zonedepart][1][1]):
                                indice = num_zonedepart
                                break
                        code_couleur_noisette[indice] = random.choice(lettres)
                        

                if len(Noisette_restantes)==0:
                    Noisette_init = True
            #else:
            if Maj_Noisette:
                print("Mettre à jour les positions des noisettes dans Liste_noisette_xya à partir de la CAM")
                if Liste_Noisette_temps_cam[int(temps_ecoules)] == []:
                    Liste_Noisette_temps_cam[int(temps_ecoules)] = Liste_noisette_xya_cam
                    print("Liste_Noisette_temps_cam[[int(temps_ecoules)]] : ",Liste_Noisette_temps_cam[int(temps_ecoules)])

                if temps_ecoules > 3:
                    for Noisette_save in Liste_noisette_xya:
                        est_supprimee = True
                        for index in range(3):
                            for Noisette_presente in Liste_Noisette_temps_cam[int(temps_ecoules)-index]:
                                distance_NN = math.sqrt(
                                    (Noisette_save[0] - Noisette_presente[0])**2 + 
                                    (Noisette_save[1] - Noisette_presente[1])**2
                                )
                                if distance_NN <= TOL_CAM_NOISETTE:
                                    est_supprimee = False

                        if est_supprimee:
                            distance_NN_ennemi = math.sqrt(
                                (Noisette_save[0] - x_ennemi)**2 + 
                                (Noisette_save[1] - y_ennemi)**2
                            )
                            if distance_NN_ennemi < R_securite:
                                print("Noisette supprimée : ",Noisette_save)
                                Liste_noisette_xya.remove(Noisette_save)

                    for index in range(5):   
                        for Noisette_presente in Liste_Noisette_temps_cam[int(temps_ecoules)-index]: 
                            est_nouvelle = True
                            for Noisette_save in Liste_noisette_xya:
                                distance_NN = math.sqrt(
                                    (Noisette_save[0] - Noisette_presente[0])**2 + 
                                    (Noisette_save[1] - Noisette_presente[1])**2
                                )
                                if distance_NN <= TOL_CAM_NOISETTE:
                                    est_nouvelle = False

                            if est_nouvelle:
                                ajoutee = False
                                for num_gm in range(len(Liste_zones_gm_coins)):
                                    zone = Liste_zones_gm_coins[num_gm]
                                    x_min, y_min = zone[0]
                                    x_max, y_max = zone[1]
                                    # Vérifier si au moins un coin est dans la zone
                                    if (x_min <= Noisette_presente[0] <= x_max and y_min <= Noisette_presente[1] <= y_max):
                                        Liste_noisette_xya.append(Noisette_presente)
                                        ajoutee = True
                                """if not ajoutee:
                                    for index_zone_depart in range(len(Liste_zones_Noisette_depart)):
                                        zone_depart = Liste_zones_Noisette_depart[index_zone_depart]
                                        x_min, y_min = zone_depart[0]
                                        x_max, y_max = zone_depart[1]
                                        if (x_min <= Noisette_presente[0] <= x_max and y_min <= Noisette_presente[1] <= y_max):
                                            Liste_noisette_xya.append(Noisette_presente)
                                            ajoutee = True"""
            # ============================================================================ #

            
            # Simu déplacement robot ennemi
            if not Reel and Simul_mvt_ennemi:
                    
                x_ennemi_voulu = int(Liste_actions_ennemi[0][0])
                y_ennemi_voulu = int(Liste_actions_ennemi[0][1])

                angle_ennemi_consigne = np.degrees(math.atan2(y_ennemi_voulu - y_ennemi, x_ennemi_voulu - x_ennemi))

                x_ennemi += round(50*np.cos(math.radians(angle_ennemi_consigne)),0)
                y_ennemi += round(50*np.sin(math.radians(angle_ennemi_consigne)),0)
            
            if Wifi and not Lidar_on:
                x_ennemi = x_ennemi_cam
                y_ennemi = y_ennemi_cam
                angle_ennemi = angle_ennemi_cam
            elif not Wifi and Lidar_on:
                with lock:
                    x_ennemi = x_ennemi_lidar  
                    y_ennemi = y_ennemi_lidar
            elif Wifi and Lidar_on:
                with lock:
                    x_ennemi = x_ennemi_lidar  
                    y_ennemi = y_ennemi_lidar
            ###
            


            # ======================== Tri Noisettes ============================================= #
            groupes_initiaux, adjacence = trouver_groupes_initiaux(Liste_noisette_xya)
            Noisettes_groupees = []
            for groupe in groupes_initiaux:
                paires = separer_groupe(groupe, Liste_noisette_xya, adjacence)
                for paire in paires:
                    noisettes_paire = [Liste_noisette_xya[i] for i in paire]
                    Noisettes_groupees.append(noisettes_paire)
                    

            # Détecte si la Liste de Noisette a changé, puis met à jour la grille
            changement_noisettes_detecte = detecter_changements_noisettes(
                Liste_noisette_xya, 
                Liste_noisette_xya_precedente
            )

            if changement_noisettes_detecte:
                print("🔄 Changement détecté dans Liste_noisette_xya - Mise à jour des grilles")
                
                grid, grid_expanded, obstacle_array, expanded_array, \
                obs_manager, obs_manager_noisettes, \
                obstacle_scatter, expanded_scatter, distance_map, \
                ax, width, height, CASE_MM = actualiser_zones_jeu(
                    grid, grid_expanded, obstacle_array, expanded_array,
                    obs_manager, obs_manager_noisettes,
                    Liste_noisette_xya,  # ⭐ NOUVEAU
                    obstacle_scatter, expanded_scatter, distance_map,
                    ax, width, height, CASE_MM
                )

                # ⭐ ÉTAPE 5 : Redessiner les noisettes
                for patch in patches_noisettes:
                    patch.remove()
                patches_noisettes = dessiner_noisettes(
                    ax, Liste_noisette_xya, 
                    longueur=150, largeur=50,
                    alpha=0.7, linewidth=2
                )
                
                # Forcer le recalcul de trajectoire
                demande_recalcul_traj = True
            ### 

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
                  
            # ================================================================================================= #

            # ============ Prise de décision ========== #
            print("Liste_strategie : ",Liste_strategie)

            if (0<=x_ennemi<=500 and 0<=y_ennemi<=1000)or(2500<x_ennemi<3000 and 0<y_ennemi<1000):
                if Liste_actions[0][0] in ["Curseur_Jaune","Curseur_Bleu"]:
                    demande_nouvelle_strat = True

            # === Retour au Nid au bout d'un certains temps === #
            print("reset_fin : ",reset_fin)
            if temps_restant < 2:
                dico_envoi[0x506]=1
            if not Strategie and not reset_fin:
                if temps_restant <= temps_retour:
                    Liste_actions.clear() 
                    if couleur == "B":
                        Liste_actions = [["Avancer",int(2700),int(1300)],["Avancer",int(x_robot_retour),int(y_robot_retour)],["Attente_test"]]
                    if couleur == "J":
                        Liste_actions = [["Avancer",int(300),int(1300)],["Avancer",int(x_robot_retour),int(y_robot_retour)],["Attente_test"]]
                    
                    
                    reset_fin = True
            elif not reset_fin and Strategie:
                if temps_restant <= temps_retour:
                    Liste_actions.clear() 
                    
                    Liste_actions = [["Consigne",int(x_robot_retour),int(y_robot_retour-100)],["Consigne",int(x_robot_retour),int(y_robot_retour)],["Attente_test"]]
                    """if couleur == "J":
                        Liste_actions.insert(0,["ReculerPrecis",1100,300])
                    if couleur == "B":
                        Liste_actions.insert(0,["ReculerPrecis",1900,300])"""

                    if faire_Ninja:
                        if couleur == "J":
                            Liste_actions.insert(0, ["ReculerPrecis",int(600+LONGUEUR_ROBOT/2),1400])
                            Liste_actions.insert(0, ["Consigne",int(1150-LARGEUR_ROBOT/2),1400])
                            Liste_actions.insert(0, ["Consigne",int(600+LONGUEUR_ROBOT/2),1400])
                            Liste_actions.insert(0, ["Consigne",int(600+LONGUEUR_ROBOT/2),1200])
                        if couleur == "B":
                            Liste_actions.insert(0, ["ReculerPrecis",int(2400-LONGUEUR_ROBOT/2),1400])
                            Liste_actions.insert(0, ["Consigne",int(1850+LARGEUR_ROBOT/2),1400])
                            Liste_actions.insert(0, ["Consigne",int(2400-LONGUEUR_ROBOT/2),1400])
                            Liste_actions.insert(0, ["Consigne",int(600+LONGUEUR_ROBOT/2),1200])

                    reset_fin = True
                else:
                    if (demande_nouvelle_strat):
                        demande_nouvelle_strat = False
                        if len(Liste_strategie[0])==2:
                            if x_strategie == Liste_strategie[0][0] and y_strategie == Liste_strategie[0][1]:
                                print("Changement de stratégie")
                                couple_strat = Liste_strategie.pop(0)   
                                Liste_strategie.append(couple_strat)
                                Astars_a_fail = True
                        elif len(Liste_strategie[0])==1:
                            if Liste_actions[0][0] in ['Curseur_Bleu','Curseur_Jaune']:
                                print("Changement de stratégie")
                                couple_strat = Liste_strategie.pop(0)

                            if Liste_actions[0][0]in ["Recalage_X","Recalage_Y"]:
                                pass 

                        if len(Liste_strategie[0])==2:
                            x_strategie = Liste_strategie[0][0]
                            y_strategie = Liste_strategie[0][1]
                            Liste_actions,demande_nouvelle_strat = remplir_Liste_actions(LARGEUR_ROBOT,x_strategie,y_strategie,Noisettes_groupees,strategie_en_cours,demande_nouvelle_strat,Liste_actions,couleur,Liste_zones_gm_coins,TOLERANCE_STRATEGIE_NOISETTE,Noisettes_stockees_dans_robot,MARGE_NOISETTE,MARGE_GM,LONGUEUR_ROBOT,Liste_noisette_xya,x_robot_actuel,y_robot_actuel,Pince_Avant, Pince_Av_1, Pince_Av_2,Pince_Arriere, Pince_Ar_1, Pince_Ar_2,width,height,CASE_MM,grid_expanded,Pousser,angle_robot_actuel)
                            demande_recalcul_traj = True

                        else:
                            if Liste_strategie[0][0] in ["Curseur"]:
                                if couleur == "B":
                                    Liste_actions = [["Curseur_Bleu"],["Attente"],["Attente"]]
                                if couleur == "J":
                                    Liste_actions = [["Curseur_Jaune"],["Attente"],["Attente"]]
                            else:
                                if Liste_strategie[0][0] in ["Recalage_X"]:
                                    Liste_actions = [["Recalage_X"],["Attente"],["Attente"]]
                                if Liste_strategie[0][0] in ["Recalage_Y"]:
                                    if couleur == "B":
                                        Liste_actions = [["ReculerPrecis",2700,175],["Recalage_Y"],["Attente"],["Attente"]]
                                    if couleur == "J":
                                        Liste_actions = [["ReculerPrecis",300,175],["Recalage_Y"],["Attente"],["Attente"]]
                            
            # ================================================= #
            

            # ========== Lire l'action courante =============== #
            if type(Liste_actions[0]) == list and len(Liste_actions[0])==3 and Liste_actions[0][0] in ["Consigne","Avancer","Reculer","ReculerPrecis"]:
                action_voulu = Liste_actions[0][0]
                x_robot_voulu = Liste_actions[0][1] 
                y_robot_voulu = Liste_actions[0][2]
                angle_robot_voulu = -181

            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==1 and Liste_actions[0][0] in ["Attente","Attente_test","Curseur_Bleu","Curseur_Jaune","Recalage_X","Recalage_Y"]:
                action_voulu = Liste_actions[0][0]
                
            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==2:
                action_voulu = Liste_actions[0][0]
                angle_robot_voulu = round(Liste_actions[0][1],0)
                
            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==3 and Liste_actions[0][0] in ["Attraper","Retourner","Relacher"]:
                action_voulu = Liste_actions[0][0]
                pince_a_utilise = Liste_actions[0][1]
                noisette_a_manipulee = Liste_actions[0][2]

            # ====== Bouger si Robot dans Zone interdite pour Attraper et Relacher === #
            if len(Liste_actions)>2 and not reset_fin:
                if (action_voulu in ["Attraper","Retourner","Relacher"]) or (action_voulu in ["Rotation"] and Liste_actions[1][0] in ["Attraper","Relacher","Retourner"]) or (action_voulu in ["Consigne","ReculerPrecis"] and Liste_actions[1][0] in ["Rotation"] and Liste_actions[2][0] in ["Attraper","Retourner"] and Liste_actions[3][0] not in ["Relacher"] )  or (action_voulu in ["Rotation"] and Liste_actions[1][0] in ["Consigne","ReculerPrecis"] and Liste_actions[2][0] in ["Rotation"] and Liste_actions[3][0] in ["Attraper","Relacher","Retourner"]) or (action_precedente in ["Relacher"] and action_voulu in ["Consigne","ReculerPrecis","Reculer","Avancer"]):
                    mode_attraper = True
                else : 
                    mode_attraper = False
            else :
                mode_attraper = False
                if (action_precedente in ["Relacher"] and action_voulu in ["Consigne","ReculerPrecis","Reculer","Avancer"] and not reset_fin) or action_voulu in ["Retourner","Attraper"]:
                    mode_attraper = True
                    
            print("mode_attraper : ",mode_attraper)

            # ======================================================================== #
            
            distance_robot_ennemi = math.sqrt((x_ennemi - x_robot_actuel)**2 + (y_ennemi - y_robot_actuel)**2)
            angle_ennemi = np.degrees(math.atan2(y_ennemi-y_ennemi_old,x_ennemi-x_ennemi_old))
            mouvement_ennemi = math.sqrt((x_ennemi - x_ennemi_old)**2 + (y_ennemi - y_ennemi_old)**2)
            
            distance_consigne_ennemi = 100000
            for action in Liste_actions:
                if action[0] in ["Consigne","ReculerPrecis"]:
                    distance_consigne_ennemi = math.sqrt((action[1] - x_ennemi)**2 + (action[2] - y_ennemi)**2)
                    break
               
            
            # ======================== CALCUL DE LA TRAJECTOIRE A* =================== #
            
            x_case_robot = max(0, min(width - 1, int(x_robot_actuel // CASE_MM)))
            y_case_robot = max(0, min(height - 1, int(y_robot_actuel // CASE_MM)))
            
            if distance_robot_ennemi <= R_securite:
                
                print("Ennemi trop proche du robot")

                if action_voulu in ["Consigne","Avancer","Reculer","ReculerPrecis","Rotation","Recalage_X","Recalage_Y"]:
                    demande_recalcul_traj = True
                    Astars_a_fail = True
                    
                else:
                    Astars_a_fail = False
                    
            elif Astars and grid_expanded[x_case_robot, y_case_robot] and not mode_attraper:
                print("⚠️ Robot dans zone interdite — recherche case libre proche")
                x_libre, y_libre = trouver_case_libre_proche(
                    x_robot_actuel, y_robot_actuel,
                    grid_expanded, CASE_MM, width, height,
                    rayon_max_mm=500
                )
                if x_libre is not None and y_libre is not None:
                    x_prochain = Liste_actions[0][1]
                    y_prochain = Liste_actions[0][2]
                    distance_sortie = math.sqrt((x_prochain - x_libre)**2 + (y_prochain - y_libre)**2)
                    print(f"✅ Case libre trouvée : ({x_libre}, {y_libre})")
                    # Insérer un Avancer prioritaire vers ce point
                    if distance_sortie > 50:
                        angle_vers_libre = math.atan2(y_libre - y_robot_actuel, x_libre - x_robot_actuel)
                        x_plusloin = x_libre + 70 * math.cos(angle_vers_libre)
                        y_plusloin = y_libre + 70 * math.sin(angle_vers_libre)
                        if action_voulu in ["Consigne", "Avancer"]:
                            Liste_actions.insert(0, ["ReculerPrecis", int(x_plusloin), int(y_plusloin)])
                        elif action_voulu in ["ReculerPrecis", "Reculer"]:
                            Liste_actions.insert(0, ["Consigne", int(x_plusloin), int(y_plusloin)])
                        Astars_a_fail = False

                """elif distance_robot_ennemi <= R_securite+30:
                Astars_a_fail = False
                print("Robot un peu proche du robot adverse")
                if action_voulu in ["Consigne","Avancer","Reculer","ReculerPrecis"]:
                    angle_robot_ennemi = math.atan2(y_ennemi - y_robot_actuel, x_ennemi - x_robot_actuel)
                    x_futur = x_robot_actuel + 100*math.cos(math.pi+angle_robot_ennemi)
                    y_futur = y_robot_actuel + 100*math.sin(math.pi+angle_robot_ennemi)
                    x_case_futur = max(0, min(width - 1, int(x_futur // CASE_MM)))
                    y_case_futur = max(0, min(height - 1, int(y_futur // CASE_MM)))
                    if math.sqrt((Liste_actions[0][1] - x_futur)**2 + (Liste_actions[0][2] - y_futur)**2)>50 and not grid_expanded[x_case_futur, y_case_futur]:
                        if action_voulu in ["Consigne", "Avancer"]:
                            Liste_actions.insert(0, ["ReculerPrecis", int(x_futur), int(y_futur)])
                        elif action_voulu in ["ReculerPrecis", "Reculer"]:
                            Liste_actions.insert(0, ["Consigne", int(x_futur), int(y_futur)])"""
            else:
                Astars_a_fail = False
                print("Ennemi assez loin du robot")

                if Astars and (action_voulu in ["Consigne","ReculerPrecis"] or demande_recalcul_traj == True) and not mode_attraper:
                    Astars_a_fail = False
                    demande_recalcul_traj = False
                    grid, grid_expanded, obstacle_array, expanded_array,obs_manager, obs_manager_noisettes,obstacle_scatter, expanded_scatter, distance_map,ax, width, height, CASE_MM = actualiser_zones_jeu(grid, grid_expanded, obstacle_array, expanded_array, obs_manager, obs_manager_noisettes,Liste_noisette_xya,obstacle_scatter, expanded_scatter, distance_map, ax, width, height, CASE_MM)
                    
                    
                    # ⭐ ÉTAPE 1 : VIDER LA QUEUE DE RÉSULTATS (supprimer anciens résultats)
                    while not queue_resultat_astar.empty():
                        try:
                            old_result = queue_resultat_astar.get_nowait()
                            print(f"⚠️ Suppression ancien résultat : {len(old_result) if old_result else 0} points")
                        except queue.Empty:
                            break
                    
                    # ⭐ ÉTAPE 2 : ENVOYER LA NOUVELLE DEMANDE
                    queue_demande_astar.put((
                        x_robot_actuel, y_robot_actuel,
                        x_robot_voulu, y_robot_voulu
                    ))
                    
                    # ⭐ ÉTAPE 3 : ATTENDRE LE RÉSULTAT AVEC TIMEOUT
                    points_bruts = None
                    timeout_calcul = 0.6  # Timeout de 0.5 secondes
                    temps_debut_attente = time.time()
                    
                    while points_bruts is None and (time.time() - temps_debut_attente) < timeout_calcul:
                        try:
                            # Attendre résultat avec timeout court
                            points_bruts = queue_resultat_astar.get(timeout=0.1)
                            print(f"✅ Résultat reçu : {len(points_bruts) if points_bruts else 0} points")
                        except queue.Empty:
                            # Pas encore de résultat, continuer à attendre
                            continue

                    # ⭐ ÉTAPE 4 : TRAITER LE RÉSULTAT
                    if points_bruts is None:
                        # Timeout dépassé
                        Astars_a_fail = True
                        print("⏱️ Timeout calcul A* - pas de résultat reçu")
                        if distance_consigne_ennemi <= R_securite+10:
                            demande_nouvelle_strat = True

                    else:

                        if points_bruts is not None and len(points_bruts) > 0:
                            # 10. AFFICHER (OPTIONNEL)
                            if ax is not None:
                                # Supprimer anciennes trajectoires
                                for line in ax.lines[:]:
                                    if line.get_label() in ['Chemin A*', 'A* brut']:
                                        line.remove()
                                
                                # Tracer nouvelle trajectoire
                                x_plot = [p[0] for p in points_bruts]
                                y_plot = [p[1] for p in points_bruts]
                                ax.plot(x_plot, y_plot, 'g-', linewidth=2, label='Chemin A*', 
                                                marker='o', markersize=4, zorder=10)
                        
                            print(f"✅ Trajectoire calculée : {len(points_bruts)} points")

                            # ⭐ NETTOYAGE : Supprimer anciens "Avancer"/"Reculer"
                            Liste_actions = [action for action in Liste_actions if not (isinstance(action, list) 
                                                and len(action) >= 2 and action[0] in ["Avancer","Reculer"])]
                            
                            # ⭐ AJOUT DES POINTS DANS Liste_actions
                            verif_mouv = 0
                            verif_mouv_rpi = 0
                            verif_angle = 0
                            
                            if len(points_bruts) > 2:
                                for i in range(len(points_bruts)-1, 0, -1):
                                    x_cible, y_cible = points_bruts[i]
                                    distance_ab = math.sqrt((x_cible - x_robot_voulu)**2 + (y_cible - y_robot_voulu)**2)
                                    if distance_ab > 5:
                                        if action_voulu in ["Consigne"]:
                                            Liste_actions.insert(0, ["Avancer", x_cible, y_cible])
                                        elif action_voulu in ["ReculerPrecis"]:
                                            Liste_actions.insert(0, ["Reculer", x_cible, y_cible])
                            else:
                                # Trajectoire trop courte, supprimer les Avancer/Reculer
                                Liste_actions = [action for action in Liste_actions if not (isinstance(action, list) 
                                                and len(action) >= 2 and action[0] in ["Avancer","Reculer"])]
                            
                            Astars_a_fail = False

            print("demande_recalcul_traj : ",demande_recalcul_traj)
            # ======================================================================== #
            
                                          
            """else:
                    if not mode_attraper:
                        print("Il faut sortir du rayon ennemi")  
                        grid, grid_expanded, obstacle_array, expanded_array,obs_manager, obs_manager_noisettes,obstacle_scatter, expanded_scatter, distance_map,ax, width, height, CASE_MM = actualiser_zones_jeu(grid, grid_expanded, obstacle_array, expanded_array, obs_manager, obs_manager_noisettes,Liste_noisette_xya,obstacle_scatter, expanded_scatter, distance_map, ax, width, height, CASE_MM)
                        Astars_a_fail = True
                        # Reculer selon la dernière trajectoire, si possible alors le faire, sinon prendre l'algorithme de fuite
                        angle_consigne_robot = math.atan2(y_robot_voulu - y_robot_actuel, x_robot_voulu - x_robot_actuel)
                        if angle_consigne_robot >math.pi:
                            angle_consigne_robot -=2*math.pi

                        angle_ennemi_robot = math.atan2(y_ennemi - y_robot_actuel, x_ennemi - x_robot_actuel)
                        if angle_ennemi_robot >math.pi:
                            angle_ennemi_robot -=2*math.pi

                        x_test = x_robot_actuel + (R_securite - distance_robot_ennemi + 50) * math.cos(math.pi+angle_ennemi_robot)
                        y_test = y_robot_actuel + (R_securite - distance_robot_ennemi + 50) * math.sin(math.pi+angle_ennemi_robot)

                        angle_robot_test = math.atan2(y_test - y_robot_actuel, x_test - x_robot_actuel)
                        if angle_robot_test > math.pi:
                            angle_robot_test -= 2*math.pi

                        distance_prochain_test = math.sqrt((x_test - x_robot_voulu)**2 + (y_test - y_robot_voulu)**2)
                        x_case_test = max(0, min(width - 1, int(x_test // CASE_MM)))
                        y_case_test = max(0, min(height - 1, int(y_test // CASE_MM)))

                        if grid_expanded[x_case_test, y_case_test]:
                            print("⚠️ Robot dans zone interdite — recherche case libre proche")
                            x_libre, y_libre = trouver_case_libre_proche(
                                x_robot_actuel, y_robot_actuel,
                                grid_expanded, CASE_MM, width, height,
                                rayon_max_mm=500
                            )
                            if x_libre is not None and y_libre is not None:
                                x_prochain = Liste_actions[0][1]
                                y_prochain = Liste_actions[0][2]
                                distance_sortie = math.sqrt((x_prochain - x_libre)**2 + (y_prochain - y_libre)**2)
                                print(f"✅ Case libre trouvée : ({x_libre}, {y_libre})")
                                # Insérer un Avancer prioritaire vers ce point
                                if distance_sortie >50:  # Seuil de proximité pour décider de se diriger vers la case libre
                                    angle_vers_libre = np.degrees(math.atan2(y_libre - y_robot_actuel, x_libre - x_robot_actuel))
                                    x_plusloin = x_libre + 30 * math.cos(math.radians(angle_vers_libre))
                                    y_plusloin = y_libre + 30 * math.sin(math.radians(angle_vers_libre))
                                    if action_voulu in ["Consigne", "Avancer"]:
                                        Liste_actions.insert(0, ["ReculerPrecis", int(x_plusloin), int(y_plusloin)])
                                    elif action_voulu in ["ReculerPrecis", "Reculer"]:
                                        Liste_actions.insert(0, ["Consigne", int(x_plusloin), int(y_plusloin)])
                            else:
                                print("❌ Aucune case libre trouvée dans le rayon de recherche")
                        else:

                            if distance_prochain_test > 60:
                                if action_voulu in ["Consigne","ReculerPrecis"]:
                                    Liste_actions.pop(0)
                                print("angle_ennemi_robot : ", np.degrees(angle_ennemi_robot))
                                print("angle_consigne_robot : ", np.degrees(angle_consigne_robot))
                                print("angle_consigne_robot - 90°: ", np.degrees(angle_consigne_robot - math.pi/2))
                                print("angle_consigne_robot + 90°: ", np.degrees(angle_consigne_robot + math.pi/2))

                                print("angle_robot_test : ", np.degrees(angle_robot_test))
                                print("angle_robot_actuel : ", angle_robot_actuel)
                                print("angle_robot_actuel-90 : ", angle_robot_actuel-90)
                                print("angle_robot_actuel+90 : ", angle_robot_actuel+90)
                                if angle_robot_actuel-90<=np.degrees(angle_robot_test)<angle_robot_actuel+90:
                                    Liste_actions.insert(0, ["Avancer", int(x_test), int(y_test)])
                                else:
                                    Liste_actions.insert(0, ["Reculer", int(x_test), int(y_test)])

                                if np.degrees(angle_consigne_robot - math.pi/2)<np.degrees(angle_ennemi_robot)<np.degrees(angle_consigne_robot + math.pi/2):
                                    print("Ennemi dans cadran, Reculer pour s'éloigner")
                                    if angle_robot_actuel-90<=np.degrees(angle_robot_test)<angle_robot_actuel+90:
                                        Liste_actions.insert(0, ["Consigne", int(x_test), int(y_test)])
                                    else:
                                        Liste_actions.insert(0, ["ReculerPrecis", int(x_test), int(y_test)])
                                else:
                                    print("Ennemi dans cadran opposé, Avancer vers Consigne")
                                    if angle_robot_actuel-90<=np.degrees(angle_robot_test)<angle_robot_actuel+90:
                                        Liste_actions.insert(0, ["ReculerPrecis", int(x_robot_voulu), int(y_robot_voulu)])
                                    else:
                                        Liste_actions.insert(0, ["Consigne", int(x_robot_voulu), int(y_robot_voulu)])"""

                            
            # ======================================================================== #
            
            # ========== Lire l'action courante =============== #
            if type(Liste_actions[0]) == list and len(Liste_actions[0])==3 and Liste_actions[0][0] in ["Consigne","Avancer","Reculer","ReculerPrecis"]:
                action_voulu = Liste_actions[0][0]
                x_robot_voulu = Liste_actions[0][1] 
                y_robot_voulu = Liste_actions[0][2]
                if Liste_actions[1][0] in ["Consigne","Avancer","Reculer","ReculerPrecis"]:
                    x_robot_voulu_prochain = Liste_actions[1][1]
                    y_robot_voulu_prochain = Liste_actions[1][2]
                else:
                    x_robot_voulu_prochain = x_robot_voulu
                    y_robot_voulu_prochain = y_robot_voulu
                angle_robot_voulu = -181

            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==1 and Liste_actions[0][0] in ["Attente","Attente_test","Curseur_Bleu","Curseur_Jaune","Recalage_X","Recalage_Y"]:
                action_voulu = Liste_actions[0][0]
                
            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==2:
                action_voulu = Liste_actions[0][0]
                angle_robot_voulu = round(Liste_actions[0][1],0)
                
            elif type(Liste_actions[0]) == list and len(Liste_actions[0])==3 and Liste_actions[0][0] in ["Attraper","Retourner","Relacher"]:
                action_voulu = Liste_actions[0][0]
                pince_a_utilise = Liste_actions[0][1]
                noisette_a_manipulee = Liste_actions[0][2]
                print("Appeler Pince N°",pince_a_utilise," pour ",action_voulu," les Noisettes ",noisette_a_manipulee)
            # ================================================= #
            
            # ===== Verif coordonnées souhaitées dans la table ============== #
            for action in Liste_actions:
                if action[0] in ["Consigne","Avancer","Reculer","ReculerPrecis"]:
                    x = action[1]
                    y = action[2]
                    if not (100 <= x <= 2900 and 100 <= y <= 1900):
                        demande_nouvelle_strat = True
            # ================================================= #

            # ====== Traduction de l'action en ordre de mouvement pour la carte Asserv ==== #
            if Astars_a_fail:
                ordre_mouvement=3
            elif action_voulu in ["Avancer"]:
                ordre_mouvement=1
            elif action_voulu in ["Reculer"]:
                ordre_mouvement=2
            elif action_voulu in ["Attraper","Retourner","Relacher","Attente","Attente_test"]:
                ordre_mouvement=3
            elif action_voulu in ["Rotation"]:
                ordre_mouvement=4
            elif action_voulu in ["Consigne"]:
                ordre_mouvement=5
            elif action_voulu in ["ReculerPrecis"]:
                ordre_mouvement = 6
            elif action_voulu in ["Curseur_Bleu"]:
                ordre_mouvement = 9
            elif action_voulu in ["Curseur_Jaune"]:
                ordre_mouvement = 10
            elif action_voulu in ["Recalage_X"]:
                ordre_mouvement = 11
            elif action_voulu in ["Recalage_Y"]:
                ordre_mouvement = 12
            else :
                ordre_mouvement=100
            # ======================================================== #
        
            print("Ordre Mouvement : ",ordre_mouvement) 
            print(f"X_actuel = {x_robot_actuel} Y_actuel = {y_robot_actuel} Angle_actuel = {angle_robot_actuel}°")
            print(f"X_voulu = {x_robot_voulu} Y_voulu = {y_robot_voulu} Angle_voulu = {angle_robot_voulu}°")
            print("Liste_actions : ", Liste_actions)

            # ==================================================================== #


            # ==== Envoi des Ordres de Consigne de Rotation à la Carte Asserv ==== #
            if angle_robot_voulu != -181:
                if angle_robot_voulu <= -180:
                    angle_robot_voulu +=360
                elif angle_robot_voulu > 180:
                    angle_robot_voulu -= 360
            dico_envoi[0x203] = x_robot_voulu_prochain
            dico_envoi[0x204] = y_robot_voulu_prochain
            dico_envoi[0x209] = x_robot_voulu
            dico_envoi[0x20A] = y_robot_voulu
            dico_envoi[0x205] = angle_robot_voulu+360
            dico_envoi[0x206] = ordre_mouvement
            # ==================================================================== #

            # Envoi des Ordres de Manipulation des Noisettes à la Carte Actionneur #
            print(Noisettes_stockees_dans_robot)
            if action_voulu in ["Attraper","Retourner","Relacher"]:
                dico_envoi[0x502+pince_a_utilise]=noisette_a_manipulee
                if action_voulu in ["Attraper"]:
                    dico_envoi[0x500+pince_a_utilise]=1
                elif action_voulu in ["Retourner"]:
                    dico_envoi[0x500+pince_a_utilise]=2
                elif action_voulu in ["Relacher"]:
                    dico_envoi[0x500+pince_a_utilise]=3
            # ==================================================================== #
            
            # ================== Simulation Mouvement Robot ====================== #

            if not Reel and not Wifi: 
                if Simul_mvt:
                    if ordre_mouvement!=3:
                        if(action_voulu in ["Rotation"]):
                            diff_angle = angle_robot_actuel % 5
                            angle_robot_actuel -= diff_angle
                            
                            # ⭐ Calculer la différence angulaire la plus courte
                            angle_diff = angle_robot_voulu - angle_robot_actuel
                            
                            # Normaliser entre -180 et 180
                            while angle_diff > 180:
                                angle_diff -= 360
                            while angle_diff < -180:
                                angle_diff += 360
                            
                            # Tourner dans le bon sens selon le signe de angle_diff
                            if angle_diff > 0:
                                angle_robot_actuel += 5  # Sens anti-horaire
                            elif angle_diff < 0:
                                angle_robot_actuel -= 5  # Sens horaire
                            
                            # Normaliser l'angle résultant entre -180 et 180
                            if angle_robot_actuel > 180:
                                angle_robot_actuel -= 360
                            elif angle_robot_actuel <= -180:
                                angle_robot_actuel += 360

                        if(action_voulu in ["Consigne","Avancer","Reculer","ReculerPrecis"]):
                            angle_robot_consigne = math.atan2(y_robot_voulu-y_robot_actuel,x_robot_voulu-x_robot_actuel)
                            if(action_voulu in ["Consigne","Avancer"]):
                                angle_robot_actuel = np.degrees(angle_robot_consigne)
                            else:
                                angle_robot_actuel = np.degrees(angle_robot_consigne)+180
                            x_robot_actuel += round(15*np.cos(angle_robot_consigne),0)
                            y_robot_actuel += round(15*np.sin(angle_robot_consigne),0)

            """elif not Reel and Wifi:
                x_robot_actuel = x_robot_actuel_cam
                y_robot_actuel = y_robot_actuel_cam_corr
                angle_robot_actuel = angle_robot_actuel_cam"""         

            # ============== MISE À JOUR AFFICHAGE ==================== #

            # Mettre à jour robot, ennemi et consigne sur affichage
            robot_plot.set_offsets([[x_robot_actuel, y_robot_actuel]])
            cercle_robot_patch.center = (x_robot_actuel, y_robot_actuel)
            
            # Mettre à jour la zone de sécurité dynamique de l'ennemi
            zone_ennemi_scatter, cercle_ennemi_patch = mettre_a_jour_zone_ennemi(
                zone_ennemi_scatter, cercle_ennemi_patch,
                x_ennemi, y_ennemi, R_ROBOT, R_ENNEMI,
                MARGE_ENNEMI, CASE_MM,
                X_PISTE, Y_PISTE
            )
            ennemi_plot.set_offsets([[x_ennemi, y_ennemi]])

            if Astars:
                if (action_voulu in ["Consigne","Avancer","Reculer","ReculerPrecis"]):
                    consigne_plot.set_offsets([[x_robot_voulu, y_robot_voulu]])
                else :
                    consigne_plot.set_offsets([[-20, -20]])
            else :
                # Récupérer tous les points "Consigne" dans Liste_actions
                points_consigne = [
                    (action[1], action[2])
                    for action in Liste_actions
                    if isinstance(action, list) and len(action) >= 3 and action[0] in ["Consigne", "Avancer","Reculer","ReculerPrecis"]
                ]

                if points_consigne:
                    # Afficher tous les points "Consigne" simultanément
                    consigne_plot.set_offsets(points_consigne)
                else:
                    # Si aucun point "Consigne", masquer l'affichage
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

            
            # Affichage texte Coordonées
            noisette_text.set_text(f"Avant : {Noisettes_stockees_dans_robot[0]} \nArrière : {Noisettes_stockees_dans_robot[1]}")
            chronometre_text.set_text(f"Temps : {int(temps_restant)} \nAction : {action_voulu}")
            match_text.set_text(f"Id : {match_id} s\nDemarre : {match_demarre}")
            robot_info_text.set_text(f"X = {x_robot_actuel:.1f} Y = {y_robot_actuel:.1f} A = {angle_robot_actuel:.1f}°")
            x_voulu_text.set_text(f"X = {x_strategie:.1f}")
            y_voulu_text.set_text(f"Y = {y_strategie:.1f}")
            A_voulu_text.set_text(f"Strat = {demande_nouvelle_strat}")
            
            # ==================================================================== #
            
            
            # ======================= GESTION BATTERIES ========================== #
            
            if step > 1:
                battery_patches, battery_texts = afficher_batteries(ax, Batteries_alert,Bat_Compet,Batteries,battery_patches, battery_texts,couleurs, seuils,largeur_rect, hauteur_rect, espacement, espacement_salves,y_base, texte_offset_y)
            
            if not Bat_Compet:                      # Si on est en mode Test
                if Batteries_alert[0]==0:           #   Si il n'y a pas de message d'alerte pour la batterie
                    Batteries_interrupteur[0]=2     #     On met l'interrupteur à 1
                else :                              #   Sinon   
                    Batteries_interrupteur[0]=1     #     On met l'interrupteur à 2

                if Batteries_alert[1]==0:
                    Batteries_interrupteur[1]=2
                else :
                    Batteries_interrupteur[1]=1
                    
                if Batteries_alert[2]==0:
                    Batteries_interrupteur[2]=2
                else :
                    Batteries_interrupteur[2]=1
            else :                                  # Sinon
                Batteries_interrupteur[0]=2         #  On met tous les interrupteurs à 1
                Batteries_interrupteur[1]=2
                Batteries_interrupteur[2]=2  

            if Batteries_alert[3]==1 and Bat_Compet == False:
                RPI_decharge = True
            else :
                RPI_decharge = False
                
            dico_envoi[0x300]=Batteries_interrupteur[0]
            dico_envoi[0x301]=Batteries_interrupteur[1]
            dico_envoi[0x302]=Batteries_interrupteur[2]

            if Bat_Compet:
                dico_envoi[0x303]=1
            else:
                dico_envoi[0x303]=2
            # ==================================================================== #
            

            # Si robot est à la position de consigne 
            distance_robot_consigne = math.sqrt((x_robot_actuel - x_robot_voulu)**2 + (y_robot_actuel - y_robot_voulu)**2)
            if((distance_robot_consigne<TOL_PRECIS and action_voulu in ["Consigne","ReculerPrecis"]) or (distance_robot_consigne<TOL_PASPRECIS and action_voulu in ["Avancer","Reculer"])):
                print("Bonne position")
                verif_mouv_rpi = 1
                    
            if not Reel :
                if(abs(angle_robot_actuel-angle_robot_voulu)<5) and action_voulu in ["Rotation"]:
                    print("Bon Angle")
                    verif_angle = 1 
                    
            if not Reel:
                if Simul_mvt_ennemi:
                    if(abs(x_ennemi-x_ennemi_voulu)<31 and abs(y_ennemi-y_ennemi_voulu)<31):
                        if(len(Liste_actions_ennemi)!=1):
                            Liste_actions_ennemi.pop(0)
            
            # ================= Envoi des accusés de réception ======================== #
            if verif_mouv_rpi == 1:
                dico_envoi[0x207]=2
            else :
                dico_envoi[0x207]=1

            if verif_angle == 1:
                dico_envoi[0x208]=2
            else :
                dico_envoi[0x208]=1

            
            if verif_recalageX == 1:
                dico_envoi[0x20B]=2
            else :
                dico_envoi[0x20B]=1

            if verif_recalageY == 1:
                dico_envoi[0x20C]=2
            else :
                dico_envoi[0x20C]=1

            if (verif_action1 == 1 and pince_a_utilise == 0)or(verif_action2 == 1 and pince_a_utilise == 1):
                dico_envoi[0x504+pince_a_utilise]=2
            else :
                dico_envoi[0x504+pince_a_utilise]=1

            if verif_curseur == 1:
                dico_envoi[0x009]=2
            else :
                dico_envoi[0x009]=1
            # ==================================================================== #
            
            print("action_precedente : ",action_precedente)


            if not action_est_supprime:
                if (action_voulu in ["Consigne","Avancer","Reculer","ReculerPrecis"] and verif_mouv_rpi == 1) or \
                (action_voulu in ["Rotation"] and verif_angle == 1) or \
                (action_voulu in ["Attente_test"] and verif_action == 1) or \
                (action_voulu in ["Curseur_Bleu","Curseur_Jaune"] and verif_curseur == 1) or \
                (action_voulu in ["Recalage_X"] and verif_recalageX == 1) or \
                (action_voulu in ["Recalage_Y"] and verif_recalageY == 1) or \
                (action_voulu in ["Attraper","Retourner","Relacher"] and ((verif_action1 == 1 and pince_a_utilise == 0)or(verif_action2 == 1 and pince_a_utilise == 1))):
                    if not Reel:
                        if action_voulu in ["Curseur_Bleu","Curseur_Jaune"]:
                            x_robot_actuel = x_fin_curseur
                            y_robot_actuel = y_fin_curseur
                    
                    if Simul_action:
                        if (action_voulu in ["Attraper"]):
                            if(Noisettes_stockees_dans_robot[pince_a_utilise][0] in ["J","B"] and Noisettes_stockees_dans_robot[pince_a_utilise][1] in ["J","B"] and noisette_a_manipulee == 12):
                                demande_nouvelle_strat = True
                                
                            if(Noisettes_stockees_dans_robot[pince_a_utilise][0] in ["J","B"] and (noisette_a_manipulee == 1 or noisette_a_manipulee == 12)):
                                demande_nouvelle_strat = True

                            if((noisette_a_manipulee == 2 or noisette_a_manipulee == 12) and Noisettes_stockees_dans_robot[pince_a_utilise][1]in ["J","B"]):
                                demande_nouvelle_strat = True

                            if(Noisettes_stockees_dans_robot[pince_a_utilise][0]=='N' and Noisettes_stockees_dans_robot[pince_a_utilise][1]=='N' and noisette_a_manipulee == 12):
                                print("Action possible")
                                for coupleNoisette in Noisettes_groupees:
                                    if len(coupleNoisette)==2:
                                        distance_R_N1 = distance((x_robot_actuel,y_robot_actuel), (coupleNoisette[0][0],coupleNoisette[0][1]))
                                        distance_R_N2 = distance((x_robot_actuel,y_robot_actuel), (coupleNoisette[1][0],coupleNoisette[1][1]))
                                        if distance_R_N1<=75+LONGUEUR_ROBOT/2 or distance_R_N2<=75+LONGUEUR_ROBOT/2:
                                            print("Supprimer : ", coupleNoisette)
                                            Noisettes_groupees.remove(coupleNoisette)
                                            Liste_noisette_xya.remove(coupleNoisette[0])
                                            Liste_noisette_xya.remove(coupleNoisette[1])
                                            if distance_R_N1<distance_R_N2:
                                                Noisettes_stockees_dans_robot[pince_a_utilise] = [coupleNoisette[0][3],coupleNoisette[1][3]]
                                            else :
                                                Noisettes_stockees_dans_robot[pince_a_utilise] = [coupleNoisette[1][3],coupleNoisette[0][3]]
                                            changement_noisettes_detecte = True
                                    else:
                                        distance_R_N = distance((x_robot_actuel,y_robot_actuel), (coupleNoisette[0][0],coupleNoisette[0][1]))
                                        if distance_R_N<=50+LONGUEUR_ROBOT/2:          
                                            print("Supprimer : ", coupleNoisette)
                                            Noisettes_groupees.remove(coupleNoisette)
                                            Liste_noisette_xya.remove(coupleNoisette[0])
                                            Noisettes_stockees_dans_robot[pince_a_utilise][0] = coupleNoisette[0][3]
                                            changement_noisettes_detecte = True

                            if (Noisettes_stockees_dans_robot[pince_a_utilise][0]=='N' and noisette_a_manipulee == 1):
                                print("Action possible")
                                for coupleNoisette in Noisettes_groupees:
                                    if len(coupleNoisette)==2:
                                        distance_R_N1 = distance((x_robot_actuel,y_robot_actuel), (coupleNoisette[0][0],coupleNoisette[0][1]))
                                        distance_R_N2 = distance((x_robot_actuel,y_robot_actuel), (coupleNoisette[1][0],coupleNoisette[1][1]))
                                        if distance_R_N1<=65+LONGUEUR_ROBOT/2 or distance_R_N2<=65+LONGUEUR_ROBOT/2:
                                            if distance_R_N1 < distance_R_N2:
                                                print("Supprimer : ", coupleNoisette[0])
                                            else :
                                                print("Supprimer : ", coupleNoisette[1])

                                            if distance_R_N1 < distance_R_N2:
                                                Liste_noisette_xya.remove(coupleNoisette[0])
                                            else :
                                                Liste_noisette_xya.remove(coupleNoisette[1])

                                            if distance_R_N1<distance_R_N2:
                                                Noisettes_stockees_dans_robot[pince_a_utilise][0] = coupleNoisette[0][3]
                                            else :
                                                Noisettes_stockees_dans_robot[pince_a_utilise][0] = coupleNoisette[1][3]
                                            changement_noisettes_detecte = True
                                    else:
                                        distance_R_N = distance((x_robot_actuel,y_robot_actuel), (coupleNoisette[0][0],coupleNoisette[0][1]))
                                        if distance_R_N<=100+LONGUEUR_ROBOT/2:
                                            
                                            Noisettes_groupees.remove(coupleNoisette)
                                            Liste_noisette_xya.remove(coupleNoisette[0])
                                            Noisettes_stockees_dans_robot[pince_a_utilise][0] = coupleNoisette[0][3]
                                            changement_noisettes_detecte = True

                            if (Noisettes_stockees_dans_robot[pince_a_utilise][1]=='N' and noisette_a_manipulee == 2):
                                print("Action possible")
                                for coupleNoisette in Noisettes_groupees:
                                    if len(coupleNoisette)==2:
                                        distance_R_N1 = distance((x_robot_actuel,y_robot_actuel), (coupleNoisette[0][0],coupleNoisette[0][1]))
                                        distance_R_N2 = distance((x_robot_actuel,y_robot_actuel), (coupleNoisette[1][0],coupleNoisette[1][1]))
                                        if distance_R_N1<=65+LONGUEUR_ROBOT/2 or distance_R_N2<=50+65+LONGUEUR_ROBOT/2:
                                            if distance_R_N1 > distance_R_N2:
                                                print("Supprimer : ", coupleNoisette[1], " distance : ", distance_R_N1)
                                            else :
                                                print("Supprimer : ", coupleNoisette[0], " distance : ", distance_R_N2)

                                            if distance_R_N1 > distance_R_N2:
                                                Liste_noisette_xya.remove(coupleNoisette[1])
                                            else :
                                                Liste_noisette_xya.remove(coupleNoisette[0])

                                            if distance_R_N1>distance_R_N2:
                                                Noisettes_stockees_dans_robot[pince_a_utilise][1] = coupleNoisette[1][3]
                                            else :
                                                Noisettes_stockees_dans_robot[pince_a_utilise][1] = coupleNoisette[0][3]
                                            changement_noisettes_detecte = True
                                    else:
                                        distance_R_N = distance((x_robot_actuel,y_robot_actuel), (coupleNoisette[0][0],coupleNoisette[0][1]))
                                        if distance_R_N<=125+LONGUEUR_ROBOT/2:
                                            Noisettes_groupees.remove(coupleNoisette)
                                            Liste_noisette_xya.remove(coupleNoisette[0])
                                            Noisettes_stockees_dans_robot[pince_a_utilise][1] = coupleNoisette[0][3]
                                            changement_noisettes_detecte = True


                        if (action_voulu in ["Retourner"]):
                            if(Noisettes_stockees_dans_robot[pince_a_utilise][0] == "N" and Noisettes_stockees_dans_robot[pince_a_utilise][1] == "N" ):
                                demande_nouvelle_strat = True
                            else :
                                if noisette_a_manipulee == 1 and Noisettes_stockees_dans_robot[pince_a_utilise][0] != "N":
                                    if Noisettes_stockees_dans_robot[pince_a_utilise][0] == "J":
                                        Noisettes_stockees_dans_robot[pince_a_utilise][0] = "B"
                                    else :
                                        Noisettes_stockees_dans_robot[pince_a_utilise][0] = "J"
                                elif  noisette_a_manipulee == 1 and Noisettes_stockees_dans_robot[pince_a_utilise][0] == "N":
                                    demande_nouvelle_strat = True

                                if noisette_a_manipulee == 2 and Noisettes_stockees_dans_robot[pince_a_utilise][1] != "N":
                                    if Noisettes_stockees_dans_robot[pince_a_utilise][1] == "J":
                                        Noisettes_stockees_dans_robot[pince_a_utilise][1] = "B"
                                    else :
                                        Noisettes_stockees_dans_robot[pince_a_utilise][1] = "J"
                                elif  noisette_a_manipulee == 2 and Noisettes_stockees_dans_robot[pince_a_utilise][1] == "N":
                                    demande_nouvelle_strat = True

                                if noisette_a_manipulee == 12 and Noisettes_stockees_dans_robot[pince_a_utilise][0] != "N" and Noisettes_stockees_dans_robot[pince_a_utilise][1] != "N":
                                    if Noisettes_stockees_dans_robot[pince_a_utilise][0] == "J":
                                        Noisettes_stockees_dans_robot[pince_a_utilise][0] = "B"
                                    else :
                                        Noisettes_stockees_dans_robot[pince_a_utilise][0] = "J"
                                    if Noisettes_stockees_dans_robot[pince_a_utilise][1] == "J":
                                        Noisettes_stockees_dans_robot[pince_a_utilise][1] = "B"
                                    else :
                                        Noisettes_stockees_dans_robot[pince_a_utilise][1] = "J"
                        
                        if (action_voulu in ["Relacher"]):
                            if noisette_a_manipulee == 1:
                                if Noisettes_stockees_dans_robot[pince_a_utilise][0]=="N":
                                    demande_nouvelle_strat = True
                                else:
                                    if pince_a_utilise == 0:
                                        x_noisette = x_robot_actuel + (25+LONGUEUR_ROBOT/2)*math.cos(math.radians(angle_robot_actuel))
                                        y_noisette = y_robot_actuel + (25+LONGUEUR_ROBOT/2)*math.sin(math.radians(angle_robot_actuel))
                                    else :
                                        x_noisette = x_robot_actuel + (25+LONGUEUR_ROBOT/2)*math.cos(math.radians(180+angle_robot_actuel))
                                        y_noisette = y_robot_actuel + (25+LONGUEUR_ROBOT/2)*math.sin(math.radians(180+angle_robot_actuel))
                                    angle_noisette = 90+angle_robot_actuel
                                    Liste_noisette_xya.append([int(x_noisette),int(y_noisette),int(angle_noisette),Noisettes_stockees_dans_robot[pince_a_utilise][0]])
                                    Noisettes_stockees_dans_robot[pince_a_utilise][0]="N"
                                    changement_noisettes_detecte = True
                            
                            if noisette_a_manipulee == 2:
                                if Noisettes_stockees_dans_robot[pince_a_utilise][1]=="N":
                                    demande_nouvelle_strat = True
                                else:
                                    if pince_a_utilise == 0:
                                        x_noisette = x_robot_actuel + (50+25+LONGUEUR_ROBOT/2)*math.cos(math.radians(angle_robot_actuel))
                                        y_noisette = y_robot_actuel + (50+25+LONGUEUR_ROBOT/2)*math.sin(math.radians(angle_robot_actuel))
                                    else :
                                        x_noisette = x_robot_actuel + (50+25+LONGUEUR_ROBOT/2)*math.cos(math.radians(180+angle_robot_actuel))
                                        y_noisette = y_robot_actuel + (50+25+LONGUEUR_ROBOT/2)*math.sin(math.radians(180+angle_robot_actuel))
                                    angle_noisette = 90+angle_robot_actuel
                                    Liste_noisette_xya.append([int(x_noisette),int(y_noisette),int(angle_noisette),Noisettes_stockees_dans_robot[pince_a_utilise][1]])
                                    Noisettes_stockees_dans_robot[pince_a_utilise][1]="N"
                                    changement_noisettes_detecte = True
                            
                            if noisette_a_manipulee == 12:
                                if Noisettes_stockees_dans_robot[pince_a_utilise][0]=="N" or Noisettes_stockees_dans_robot[pince_a_utilise][1]=="N":
                                    demande_nouvelle_strat = True
                                else:
                                    if pince_a_utilise == 0:
                                        x_noisette_1 = x_robot_actuel + (25+LONGUEUR_ROBOT/2)*math.cos(math.radians(angle_robot_actuel))
                                        y_noisette_1 = y_robot_actuel + (25+LONGUEUR_ROBOT/2)*math.sin(math.radians(angle_robot_actuel))
                                        x_noisette_2 = x_robot_actuel + (50+25+LONGUEUR_ROBOT/2)*math.cos(math.radians(angle_robot_actuel))
                                        y_noisette_2 = y_robot_actuel + (50+25+LONGUEUR_ROBOT/2)*math.sin(math.radians(angle_robot_actuel))
                                    else :
                                        x_noisette_1 = x_robot_actuel + (25+LONGUEUR_ROBOT/2)*math.cos(math.radians(180+angle_robot_actuel))
                                        y_noisette_1 = y_robot_actuel + (25+LONGUEUR_ROBOT/2)*math.sin(math.radians(180+angle_robot_actuel))
                                        x_noisette_2 = x_robot_actuel + (50+25+LONGUEUR_ROBOT/2)*math.cos(math.radians(180+angle_robot_actuel))
                                        y_noisette_2 = y_robot_actuel + (50+25+LONGUEUR_ROBOT/2)*math.sin(math.radians(180+angle_robot_actuel))

                                    angle_noisette_1 = 90+angle_robot_actuel
                                    angle_noisette_2 = 90+angle_robot_actuel
                                    
                                    Liste_noisette_xya.append([int(x_noisette_1),int(y_noisette_1),int(angle_noisette_1),Noisettes_stockees_dans_robot[pince_a_utilise][0]])
                                    Liste_noisette_xya.append([int(x_noisette_2),int(y_noisette_2),int(angle_noisette_2),Noisettes_stockees_dans_robot[pince_a_utilise][1]])
                                    Noisettes_stockees_dans_robot[pince_a_utilise][0]="N"
                                    Noisettes_stockees_dans_robot[pince_a_utilise][1]="N"
                                    changement_noisettes_detecte = True
                                            
                    
                    # ==================================================================== #

                    if changement_noisettes_detecte:
                        changement_noisettes_detecte = False
                        print("🔄 Changement détecté dans Liste_noisette_xya - Mise à jour des grilles")
                        grid, grid_expanded, obstacle_array, expanded_array, \
                        obs_manager, obs_manager_noisettes, \
                        obstacle_scatter, expanded_scatter, distance_map, \
                        ax, width, height, CASE_MM = actualiser_zones_jeu(
                            grid, grid_expanded, obstacle_array, expanded_array,
                            obs_manager, obs_manager_noisettes,
                            Liste_noisette_xya,  # ⭐ NOUVEAU
                            obstacle_scatter, expanded_scatter, distance_map,
                            ax, width, height, CASE_MM
                        )
                        # ⭐ ÉTAPE 5 : Redessiner les noisettes
                        for patch in patches_noisettes:
                            patch.remove()
                        patches_noisettes = dessiner_noisettes(
                            ax, Liste_noisette_xya, 
                            longueur=150, largeur=50,
                            alpha=0.7, linewidth=2
                        )
                        
                    # Retirer l'action de la liste 
                    Liste_actions.pop(0) 
                    verif_mouv=0
                    verif_mouv_rpi=0
                    verif_angle = 0
                    verif_recalageX = 0
                    verif_recalageY = 0
                    angle_robot_voulu = -181
                    verif_action = 0
                    verif_action1 = 0
                    verif_action2 = 0
                    dico_envoi[0x500+pince_a_utilise]=0
                    dico_envoi[0x502+pince_a_utilise]=0
                    action_est_supprime = True
                    action_precedente = action_voulu
                    x_sortie_fixe = None
                    y_sortie_fixe = None
                    if Strategie:
                        if Liste_actions[0][0] in ["Attente"] and len(Liste_strategie)>1:
                            Liste_strategie.pop(0)
                            demande_nouvelle_strat = True
                            demande_recalcul_traj = True
            else :
                action_est_supprime = False
            
            """for couple in dico_envoi.items():
                if couple[1] != 0:
                    print(hex(couple[0])," : ",couple[1])"""
            
            if Reel :
                for key, value in dico_envoi.items() :
                    if key in [0x500,0x501]:
                        format_value = struct.pack('<i',dico_envoi[key])
                        msg = can.Message(arbitration_id=key, data=format_value, is_extended_id=False)
                        bus.send(msg)
                        dico_envoi[key]=0
                        time.sleep(0.0006)
                    else:
                        if value != 0:
                            if key in [0x01,0x206,0x207,0x208,0x300,0x301,0x302,0x303,0x500,0x501,0x502,0x503,0x504,0x505,0x506]:
                                format_value = struct.pack('<i',dico_envoi[key])
                            else:
                                format_value = struct.pack('<f',dico_envoi[key])
                            msg = can.Message(arbitration_id=key, data=format_value, is_extended_id=False)
                            bus.send(msg)
                            dico_envoi[key]=0
                            time.sleep(0.0006)
            

            # MAJ de l'affichage et des Variables de Bouncing
            if step % FREQUENCE_AFFICHAGE == 0:
                update_display(background)
                fig.canvas.flush_events()
            x_robot_voulu_last = x_robot_voulu
            y_robot_voulu_last = y_robot_voulu
            x_ennemi_old = x_ennemi
            y_ennemi_old = y_ennemi
            old_verif_mouv = verif_mouv
            old_ordre_mouvement = ordre_mouvement
            Liste_noisette_xya_precedente = [noisette[:] for noisette in Liste_noisette_xya]  # Copie profonde
            Liste_actions_precedente = Liste_actions.copy()

            if Wifi:
                donnees_vers_bc = {
                    "match_id": match_id,
                    "match_demarre": match_demarre,
                    "x_robot_actuel": x_robot_actuel,
                    "y_robot_actuel": y_robot_actuel,
                    "angle_robot_actuel": angle_robot_actuel,
                    "x_ennemi": x_ennemi,
                    "y_ennemi": y_ennemi,
                    "Batteries": Batteries,
                    "Batteries_alert": Batteries_alert,
                    "Noisettes_stockees_dans_robot": Noisettes_stockees_dans_robot,
                    "action_voulu" : action_voulu,
                    "action_precedente" : action_precedente,
                    "step_robot" : step,
                }
                try:
                    message = json.dumps(donnees_vers_bc)
                    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    client_socket.settimeout(0.1)
                    client_socket.connect((IP_BC, PORT_ENVOI))
                    client_socket.sendall(message.encode())
                    client_socket.close()
                except (socket.timeout, ConnectionRefusedError, OSError) as e:
                    print(f"WiFi Envoi échoué : {e}")
            print("")
            time.sleep(0.00001)


        if etat_bau == 1:
            print("BAU enfoncé")

        if RPI_decharge:
            print("Batterie RPI trop faible")

        if Batteries_alert[0]==1:
            print("Batterie 1 Trop faible")
        if Batteries_alert[1]==1:
            print("Batterie 2 Trop faible")
        if Batteries_alert[2]==1:
            print("Batterie 3 Trop faible")
            
        stop_event.set()

    except KeyboardInterrupt:
        print("Arrêt demandé par l'utilisateur.")
        stop_event.set()  # signal aux threads de s'arrêter
        # Attente que chaque thread termine proprement
        
        if Lidar_on:
            tache_Lidar.join()

        if Reel :
            tache_LectureCAN.join()
            os.system("sudo ifconfig can0 down")
        if Astars:
            queue_demande_astar.put(None)  # Signal d'arrêt au thread
            tache_Astar.join(timeout=2)
        if Wifi:
            tache_Wifi.join()
            tache_PAMI.join()
            
        
    except Exception as e:
        print(e)
    finally:
        print("Programme terminé proprement.")
    
        if Lidar_on:
            tache_Lidar.join()
        
        if Reel :
            tache_LectureCAN.join()
            etat_RPI = 2
            data_etat_RPI = struct.pack('<i',etat_RPI)
            bus.send(can.Message(arbitration_id=0x01, data=data_etat_RPI, is_extended_id=False))
        
        if Astars:
            queue_demande_astar.put(None)  # Signal d'arrêt au thread
            tache_Astar.join(timeout=2)
        if Wifi:
            tache_Wifi.join()
            tache_PAMI.join()
            
        plt.close(fig)

########################################################################