# affichage.py - VERSION COMPLETE AVEC ZONE ENNEMI
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as patches
import numpy as np
from matplotlib.widgets import Button
from matplotlib.lines import Line2D
from fonction import creer_zone_securite_ennemi
import tkinter as tk
from tkinter import font as tkfont

def afficher_obstacles(ax, obstacle_array, expanded_array, case_mm, 
                       show_expanded=True, show_obstacles=False):
    """
    Affiche les obstacles sur les axes matplotlib.
    
    Args:
        ax: axes matplotlib
        obstacle_array: array des positions d'obstacles (en cases)
        expanded_array: array des zones de securite (en cases), optionnel
        case_mm: taille d'une case en mm
        show_expanded: afficher ou non la zone etendue
        show_obstacles: afficher ou non les obstacles originaux
        
    Returns:
        tuple: (obstacle_scatter, expanded_scatter)
    """
    obstacle_scatter = None
    expanded_scatter = None
    
    # Obstacles principaux
    if show_obstacles and obstacle_array is not None and len(obstacle_array) > 0:
        obstacle_scatter = ax.scatter(
            obstacle_array[:, 0] * case_mm,
            obstacle_array[:, 1] * case_mm,
            c='darkgray',
            s=4,
            alpha=0.9,
            label='Obstacles',
            zorder=1
        )
    
    # Zone de securite etendue
    if show_expanded and expanded_array is not None and len(expanded_array) > 0:
        expanded_scatter = ax.scatter(
            expanded_array[:, 0] * case_mm,
            expanded_array[:, 1] * case_mm,
            c='lightcoral',
            s=1,
            alpha=0.3,
            label='Zone interdite',
            zorder=0
        )
    
    return obstacle_scatter, expanded_scatter


def afficher_zone_securite_ennemi(ax, x_ennemi, y_ennemi, r_robot, r_ennemi,
                                   marge_securite, case_mm,
                                   terrain_w_mm, terrain_h_mm,
                                   show_zone=True, show_cercle=True):
    """
    Affiche la zone de securite autour de l'ennemi.
    
    Args:
        ax: Axes matplotlib
        x_ennemi: Position x de l'ennemi en mm
        y_ennemi: Position y de l'ennemi en mm
        r_robot: Rayon du robot en mm
        r_ennemi: Rayon de l'ennemi en mm
        marge_securite: Marge de securite en mm
        case_mm: Taille d'une case en mm
        terrain_w_mm: Largeur du terrain en mm
        terrain_h_mm: Hauteur du terrain en mm
        show_zone: Afficher la zone remplie (scatter)
        show_cercle: Afficher le cercle de perimetre
    
    Returns:
        tuple: (zone_scatter, cercle_patch)
            - zone_scatter: Objet scatter de la zone (ou None)
            - cercle_patch: Objet Circle du perimetre (ou None)
    """
    zone_scatter = None
    cercle_patch = None
    
    # Creer la zone de securite
    zone_ennemi_array, R_securite = creer_zone_securite_ennemi(
        x_ennemi, y_ennemi, r_robot, r_ennemi, marge_securite,
        case_mm, terrain_w_mm, terrain_h_mm
    )
    
    # Afficher la zone remplie
    if show_zone and len(zone_ennemi_array) > 0:
        zone_scatter = ax.scatter(
            zone_ennemi_array[:, 0],
            zone_ennemi_array[:, 1],
            c='orange',
            s=2,
            alpha=0.2,
            label='Zone securite ennemi',
            zorder=2
        )
    
    # Afficher le cercle de perimetre
    if show_cercle:
        cercle_patch = plt.Circle(
            (x_ennemi, y_ennemi),
            R_securite,
            color='orange',
            fill=False,
            linestyle='--',
            linewidth=2,
            alpha=0.8,
            zorder=3
        )
        ax.add_patch(cercle_patch)
    
    return zone_scatter, cercle_patch


def mettre_a_jour_zone_ennemi(zone_scatter, cercle_patch, 
                               x_ennemi, y_ennemi, r_robot, r_ennemi,
                               marge_securite, case_mm,
                               terrain_w_mm, terrain_h_mm):
    """
    Met a jour la position de la zone de securite de l'ennemi.
    
    Args:
        zone_scatter: Objet scatter existant de la zone
        cercle_patch: Objet Circle existant du perimetre
        x_ennemi: Nouvelle position x de l'ennemi en mm
        y_ennemi: Nouvelle position y de l'ennemi en mm
        r_robot: Rayon du robot en mm
        r_ennemi: Rayon de l'ennemi en mm
        marge_securite: Marge de securite en mm
        case_mm: Taille d'une case en mm
        terrain_w_mm: Largeur du terrain en mm
        terrain_h_mm: Hauteur du terrain en mm
    
    Returns:
        tuple: (zone_scatter, cercle_patch) mis a jour
    """
    # Recalculer la zone
    zone_ennemi_array, R_securite = creer_zone_securite_ennemi(
        x_ennemi, y_ennemi, r_robot, r_ennemi, marge_securite,
        case_mm, terrain_w_mm, terrain_h_mm
    )
    
    # Mettre a jour le scatter
    if zone_scatter is not None and len(zone_ennemi_array) > 0:
        zone_scatter.set_offsets(zone_ennemi_array)
    
    # Mettre a jour le cercle
    if cercle_patch is not None:
        cercle_patch.center = (x_ennemi, y_ennemi)
        cercle_patch.radius = R_securite
    
    return zone_scatter, cercle_patch

# ============================================================================
# FONCTION UTILITAIRE
# ============================================================================

def bring_to_front(fig):
    """Force la fenetre matplotlib a repasser au premier plan."""
    try:
        manager = plt.get_current_fig_manager()
        if hasattr(manager, 'window'):
            window = manager.window
            window.attributes('-topmost', 1)
            window.attributes('-topmost', 0)
    except Exception:
        pass


# ============================================================================
# INITIALISATION DE L'AFFICHAGE
# ============================================================================

def init_affichage(x_robot_depart,y_robot_depart,R_ROBOT):
    """
    Initialise la fenetre graphique pour le robot et l'ennemi.
    Retourne les objets utiles : figure, axes, plots et boutons.
    """
    # Chargement et configuration de l'image
    img = mpimg.imread("C:/Users/achil/Desktop/Achille/IUT GEII/Crac2026/ProgGithub/CRAC2026/RPI_Algo/Piste.png")
    img = np.rot90(img, 2)  # rotation de 180°

    fig, ax = plt.subplots(figsize=(600/100, 400/100), dpi=100)
    plt.ion()
    plt.show()

    # Objets graphiques
    robot_plot = ax.scatter([], [], s=50, c='blue', marker='o', label="Robot")
    # Cercle de sécurité du robot
    cercle_robot_patch = plt.Circle(
        (x_robot_depart, y_robot_depart),
        R_ROBOT,
        color='lightgreen',
        fill=False,
        linestyle='--',
        linewidth=2,
        alpha=0.8,
        zorder=3
    )

    ax.add_patch(cercle_robot_patch)
    ennemi_plot = ax.scatter([], [], s=50, c='red', marker='o', label="Ennemi")
    consigne_plot = ax.scatter([], [], s=50, c='green', marker='x', label="Consigne")
    scat = ax.scatter([], [], s=5, c='blue', alpha=0.5, label="Points Lidar")

    ax.imshow(img, extent=[0, 3000, 0, 2000], origin='lower')
    ax.set_xlim(0, 3000)
    ax.set_ylim(0, 2250)
    ax.set_aspect('equal', adjustable='box')

    # Textes d'info
    noisette_text = ax.text(2900, 2500, "", color='black', fontsize=8, ha='left', va='top')
    chronometre_text = ax.text(2150, 2500, "", color='black', fontsize=8, ha='left', va='top')
    match_text = ax.text(1500, 2500, "", color='black', fontsize=8, ha='left', va='top')
    info_alim_rpi = ax.text(1500, 2500, "", color='black', fontsize=8, ha='left', va='top')

    robot_info_text = ax.text(1500, 2200, "", color='black', fontsize=8, ha='left', va='top')
    x_voulu_text = ax.text(1500, 2100, "", color='black', fontsize=8, ha='left', va='top')
    y_voulu_text = ax.text(2000, 2100, "", color='black', fontsize=8, ha='left', va='top')
    A_voulu_text = ax.text(2500, 2100, "", color='black', fontsize=8, ha='left', va='top')
    
    # Bouton STOP
    ax_button_stop = plt.axes([0.02, 0.93, 0.15, 0.05])
    bouton_stop = Button(ax_button_stop, 'STOP', color='red', hovercolor='orange')
    bouton_stop.label.set_fontsize(11)
    bouton_stop.label.set_color('white')

    # ⭐ NOUVEAU : Bouton START ⭐
    ax_button_start = plt.axes([0.19, 0.93, 0.15, 0.05])  # Position à droite du bouton STOP
    bouton_start = Button(ax_button_start, 'START', color='green', hovercolor='lightgreen')
    bouton_start.label.set_fontsize(11)
    bouton_start.label.set_color('white')

    # Point voulu
    point_voulu_plot, = ax.plot([], [], 'bx', markersize=10, label="Point voulu")
    # Lignes d'angle
    robot_angle_line = Line2D([0, 0], [0, 0], color='blue', linewidth=2)
    robot_angle_voulu_line = Line2D([0, 0], [0, 0], color='green', linewidth=2)
    
    fig.canvas.draw()
    background = fig.canvas.copy_from_bbox(ax.bbox)

    return (fig, ax, robot_plot, ennemi_plot, consigne_plot, scat, robot_info_text,
            ax_button_stop, bouton_stop, ax_button_start, bouton_start,
            point_voulu_plot, x_voulu_text, y_voulu_text, 
            A_voulu_text, robot_angle_line, robot_angle_voulu_line,background,info_alim_rpi,chronometre_text,cercle_robot_patch,noisette_text,match_text)


def afficher_batteries(ax, Batteries_alert,Bat_Compet,Batteries, battery_patches, battery_texts,
                      couleurs=['red', 'orange', 'yellow', 'lime', 'green'],
                      seuils=[1, 20, 50, 75, 90],
                      largeur_rect=50, hauteur_rect=150, 
                      espacement=0, espacement_salves=200,
                      y_base=2050, texte_offset_y=80,
                      x_depart_base=100):
    """
    Affiche l'etat de la batterie active sous forme de barres colorees.
     
    Args:
        ax: Axes matplotlib ou afficher
        Batteries: Liste des etats des batteries
        battery_patches: Liste des rectangles (sera videe puis remplie)
        battery_texts: Liste des textes (sera videe puis remplie)
        couleurs: Liste des couleurs pour chaque niveau de charge
        seuils: Liste des seuils de % pour chaque couleur
        largeur_rect: Largeur d'un rectangle en mm
        hauteur_rect: Hauteur d'un rectangle en mm
        espacement: Espace entre rectangles d'une meme batterie
        espacement_salves: Espace entre batteries differentes
        y_base: Position verticale de base
        texte_offset_y: Decalage vertical du texte de pourcentage
        x_depart_base: Position x de depart pour la premiere batterie
    
    Returns:
        tuple: (battery_patches, battery_texts) mis a jour
    """
    # Nettoyer les anciens patches et textes
    for patch in battery_patches:
        if hasattr(patch, 'remove'):  # Vérifier que c'est bien un objet matplotlib
            try:
                patch.remove()
            except:
                pass
    
    for text in battery_texts:
        if hasattr(text, 'remove'):  # Vérifier que c'est bien un objet matplotlib
            try:
                text.remove()
            except:
                pass
    
    # Réinitialiser les listes
    battery_patches.clear()
    battery_texts.clear()
    
    # Determiner quelle batterie afficher et sa position x
    battery_index = None
    x_depart = x_depart_base
    for num_batt in range(len(Batteries)):
        if Batteries[num_batt]!=0 and Batteries[num_batt]!=255:
            battery_index = num_batt
            x_depart = x_depart_base + num_batt * (5 * (largeur_rect + espacement) + espacement_salves)
            # Si une batterie est active, l'afficher
            if battery_index is not None:
                # Dessiner les 5 rectangles de niveau de charge
                for j in range(5):
                    if Batteries[battery_index] >= seuils[j]:
                        x = x_depart + j * (largeur_rect + espacement)
                        rect = patches.Rectangle(
                            (x, y_base), largeur_rect, hauteur_rect,
                            linewidth=0, edgecolor='none', 
                            facecolor=couleurs[j], alpha=0.9
                        )
                        ax.add_patch(rect)
                        battery_patches.append(rect)
                
                # Ajouter le texte de pourcentage au centre
                largeur_totale = 5 * largeur_rect + 4 * espacement
                x_centre_salve = x_depart + largeur_totale / 2
                if Batteries[battery_index] !=0 and Batteries[battery_index] !=255:
                    if Batteries_alert[battery_index]==0:
                        txt = ax.text(
                            x_centre_salve, 
                            y_base + hauteur_rect + texte_offset_y,
                            f"{Batteries[battery_index]}%",
                            color='black', fontsize=8, ha='center', va='bottom'
                        )
                    else :
                        txt = ax.text(
                            x_centre_salve, 
                            y_base + hauteur_rect + texte_offset_y,
                            f"{Batteries[battery_index]}%",
                            color='red', fontsize=8, ha='center', va='bottom'
                        )
                else :
                    txt = ax.text(
                        x_centre_salve, 
                        y_base + hauteur_rect + texte_offset_y,
                        f"",
                        color='black', fontsize=8, ha='center', va='bottom'
                    )
                battery_texts.append(txt)
    
    return battery_patches, battery_texts


# ============================================================================
# FIN DU FICHIER
# ============================================================================
def dessiner_noisettes(ax, Liste_noisette_xya, longueur=150, largeur=50, 
                       alpha=0.6, linewidth=2):
    """
    Dessine des rectangles orientés représentant les noisettes avec couleurs.
    
    Args:
        ax: Axes matplotlib
        Liste_noisette_xya: Liste de quadruplets [x_centre, y_centre, angle_degres, couleur]
                           où couleur peut être "J" (jaune), "B" (bleu), ou une couleur matplotlib
        longueur: Longueur du rectangle en mm (dans la direction de l'angle)
        largeur: Largeur du rectangle en mm (perpendiculaire à l'angle)
        alpha: Transparence (0-1)
        linewidth: Épaisseur du contour
        
    Returns:
        list: Liste des patches créés
    """
    import matplotlib.patches as patches_module
    import matplotlib.transforms as transforms
    
    # Dictionnaire de conversion des codes couleurs
    couleurs_map = {
        'J': 'gold',        # Jaune
        'B': 'dodgerblue',  # Bleu
        'R': 'red',         # Rouge (si besoin)
        'V': 'green',       # Vert (si besoin)
    }
    
    patches_noisettes = []
    
    for i, noisette_data in enumerate(Liste_noisette_xya):
        # Gérer les formats : [x, y, angle] ou [x, y, angle, couleur]
        if len(noisette_data) == 3:
            x_centre, y_centre, angle_deg = noisette_data
            couleur_code = 'brown'  # Couleur par défaut
        elif len(noisette_data) == 4:
            x_centre, y_centre, angle_deg, couleur_code = noisette_data
        else:
            print(f"⚠️ Format invalide pour noisette {i+1}: {noisette_data}")
            continue
        
        # Convertir le code couleur en couleur matplotlib
        if couleur_code in couleurs_map:
            couleur = couleurs_map[couleur_code]
        else:
            couleur = couleur_code  # Utiliser directement si c'est déjà une couleur matplotlib
        
        # Créer un rectangle orienté
        # matplotlib.patches.Rectangle utilise le coin inférieur gauche
        # On doit calculer ce coin à partir du centre
        
        # Angle en radians
        angle_rad = np.deg2rad(angle_deg)
        
        # Coin inférieur gauche du rectangle (avant rotation)
        # Le rectangle est centré, donc décalage de -longueur/2, -largeur/2
        coin_x = -longueur / 2
        coin_y = -largeur / 2
        
        # Créer le rectangle
        rect = patches_module.Rectangle(
            (coin_x, coin_y),  # Position du coin (avant transformation)
            longueur,          # Longueur
            largeur,           # Largeur
            linewidth=linewidth,
            edgecolor='black',
            facecolor=couleur,
            alpha=alpha,
            zorder=5
        )
        
        # Appliquer la rotation et la translation
        t = transforms.Affine2D().rotate(angle_rad).translate(x_centre, y_centre) + ax.transData
        rect.set_transform(t)
        
        # Ajouter à l'axe
        ax.add_patch(rect)
        patches_noisettes.append(rect)
        
        # Ajouter un numéro avec la couleur du code
        """text_color = 'white' if couleur_code in ['B', 'R'] else 'black'
        ax.text(x_centre, y_centre, str(i+1), 
                ha='center', va='center', 
                fontsize=8, color=text_color, 
                fontweight='bold', zorder=6)"""
    
    return patches_noisettes

def fenetre_selection_couleur():
    """
    Crée une fenêtre modale pour choisir la couleur (Jaune ou Bleu).
    Retourne 'J' ou 'B' selon le choix de l'utilisateur.
    """
    couleur_selectionnee = [None]  # Liste pour stocker la valeur (closure)
    
    # Créer la fenêtre
    root = tk.Tk()
    root.title("Sélection de la couleur")
    root.geometry("400x250")
    root.configure(bg='white')
    
    # Centrer la fenêtre
    root.update_idletasks()
    x = (root.winfo_screenwidth() // 2) - (400 // 2)
    y = (root.winfo_screenheight() // 2) - (250 // 2)
    root.geometry(f'400x250+{x}+{y}')
    
    # Police personnalisée
    title_font = tkfont.Font(family="Arial", size=18, weight="bold")
    button_font = tkfont.Font(family="Arial", size=14, weight="bold")
    
    # Titre
    label = tk.Label(
        root, 
        text="Choisissez votre couleur :", 
        font=title_font,
        bg='white',
        fg='black'
    )
    label.pack(pady=30)
    
    # Frame pour les boutons
    button_frame = tk.Frame(root, bg='white')
    button_frame.pack(pady=20)
    
    def choisir_jaune():
        couleur_selectionnee[0] = "J"
        root.destroy()
    
    def choisir_bleu():
        couleur_selectionnee[0] = "B"
        root.destroy()
    
    # Bouton Jaune
    btn_jaune = tk.Button(
        button_frame,
        text="JAUNE",
        command=choisir_jaune,
        font=button_font,
        bg='#FFD700',  # Or/Jaune
        fg='black',
        width=12,
        height=2,
        relief='raised',
        bd=3,
        cursor='hand2'
    )
    btn_jaune.pack(side=tk.LEFT, padx=15)
    
    # Bouton Bleu
    btn_bleu = tk.Button(
        button_frame,
        text="BLEU",
        command=choisir_bleu,
        font=button_font,
        bg='#4169E1',  # Bleu royal
        fg='white',
        width=12,
        height=2,
        relief='raised',
        bd=3,
        cursor='hand2'
    )
    btn_bleu.pack(side=tk.LEFT, padx=15)
    
    # Empêcher la fermeture de la fenêtre sans choix
    root.protocol("WM_DELETE_WINDOW", lambda: None)
    
    # Lancer la boucle principale
    root.mainloop()
    
    # Retourner la couleur sélectionnée (ou 'B' par défaut si aucun choix)
    return couleur_selectionnee[0] if couleur_selectionnee[0] is not None else "B"