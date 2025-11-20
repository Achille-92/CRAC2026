# affichage.py - VERSION COMPLETE AVEC ZONE ENNEMI
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import matplotlib.patches as patches
import numpy as np
from matplotlib.widgets import Button
from matplotlib.lines import Line2D
from fonction import creer_zone_securite_ennemi

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

def init_affichage():
    """
    Initialise la fenetre graphique pour le robot et l'ennemi.
    Retourne les objets utiles : figure, axes, plots et boutons.
    """
    # Chargement et configuration de l'image
    img = mpimg.imread("Piste.png")
    img = np.rot90(img, 2)  # rotation de 180°

    fig, ax = plt.subplots(figsize=(600/100, 400/100), dpi=100)
    plt.ion()
    plt.show()

    # Objets graphiques
    robot_plot = ax.scatter([], [], s=50, c='blue', marker='o', label="Robot")
    ennemi_plot = ax.scatter([], [], s=50, c='red', marker='o', label="Ennemi")
    consigne_plot = ax.scatter([], [], s=50, c='green', marker='x', label="Consigne")
    scat = ax.scatter([], [], s=5, c='blue', alpha=0.5, label="Points Lidar")

    ax.imshow(img, extent=[0, 3000, 0, 2000], origin='lower')
    ax.set_xlim(0, 3000)
    ax.set_ylim(0, 2250)
    ax.set_aspect('equal', adjustable='box')

    # Textes d'info
    robot_info_text = ax.text(1500, 2200, "", color='black', fontsize=8, ha='left', va='top')
    x_voulu_text = ax.text(1500, 2100, "", color='black', fontsize=8, ha='left', va='top')
    y_voulu_text = ax.text(2000, 2100, "", color='black', fontsize=8, ha='left', va='top')
    A_voulu_text = ax.text(2500, 2100, "", color='black', fontsize=8, ha='left', va='top')
    
    # Bouton STOP
    ax_button = plt.axes([0.07, 0.93, 0.15, 0.05])
    bouton_stop = Button(ax_button, 'STOP', color='red', hovercolor='orange')
    bouton_stop.label.set_fontsize(11)
    bouton_stop.label.set_color('white')

    # Point voulu
    point_voulu_plot, = ax.plot([], [], 'bx', markersize=10, label="Point voulu")
    # Lignes d'angle
    robot_angle_line = Line2D([0, 0], [0, 0], color='blue', linewidth=2)
    robot_angle_voulu_line = Line2D([0, 0], [0, 0], color='green', linewidth=2)
    
    fig.canvas.draw()
    background = fig.canvas.copy_from_bbox(ax.bbox)

    return (fig, ax, robot_plot, ennemi_plot, consigne_plot, scat, robot_info_text,
            ax_button, bouton_stop, point_voulu_plot, x_voulu_text, y_voulu_text, 
            A_voulu_text, robot_angle_line, robot_angle_voulu_line,background)


def afficher_batteries(ax, Batteries, Ordre_Batteries, battery_patches, battery_texts,
                      couleurs=['red', 'orange', 'yellow', 'lime', 'green'],
                      seuils=[1, 20, 50, 75, 90],
                      largeur_rect=50, hauteur_rect=150, 
                      espacement=0, espacement_salves=200,
                      y_base=2050, texte_offset_y=80,
                      x_depart_base=200):
    """
    Affiche l'etat de la batterie active sous forme de barres colorees.
    
    Args:
        ax: Axes matplotlib ou afficher
        Batteries: Liste des etats des batteries
        Ordre_Batteries: [1,0,0] ou [0,1,0] ou [0,0,1] (batterie active)
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
    for rect in battery_patches:
        rect.remove()
    battery_patches.clear()
    
    for txt in battery_texts:
        txt.remove()
    battery_texts.clear()
    
    # Determiner quelle batterie afficher et sa position x
    battery_index = None
    x_depart = x_depart_base
    
    if Ordre_Batteries == [1, 0, 0]:
        battery_index = 0
        x_depart = x_depart_base
    elif Ordre_Batteries == [0, 1, 0]:
        battery_index = 1
        x_depart = x_depart_base + 1 * (5 * (largeur_rect + espacement) + espacement_salves)
    elif Ordre_Batteries == [0, 0, 1]:
        battery_index = 2
        x_depart = x_depart_base + 2 * (5 * (largeur_rect + espacement) + espacement_salves)
    
    # Si une batterie est active, l'afficher
    if battery_index is not None:
        # Dessiner les 5 rectangles de niveau de charge
        for j in range(5):
            if Batteries[battery_index][3] >= seuils[j]:
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
        txt = ax.text(
            x_centre_salve, 
            y_base + hauteur_rect + texte_offset_y,
            f"{Batteries[battery_index][3]}%",
            color='black', fontsize=8, ha='center', va='bottom'
        )
        battery_texts.append(txt)
    
    return battery_patches, battery_texts


# ============================================================================
# FIN DU FICHIER
# ============================================================================
