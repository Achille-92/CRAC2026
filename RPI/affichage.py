# affichage.py
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from matplotlib.widgets import Button

import matplotlib
import tkinter as tk

from matplotlib.lines import Line2D

def bring_to_front(fig):
    """Force la fenêtre matplotlib à repasser au premier plan."""
    try:
        manager = plt.get_current_fig_manager()
        if hasattr(manager, 'window'):
            # Récupère la fenêtre native Tkinter, Qt ou autre
            window = manager.window
            window.attributes('-topmost', 1)
            window.attributes('-topmost', 0)
    except Exception:
        pass

def init_affichage(R_securite):
    """
    Initialise la fenêtre graphique pour le robot et l'ennemi.
    Retourne les objets utiles : figure, axes, plots et boutons.
    """
    # --- Chargement et configuration de l’image ---
    #img = mpimg.imread("/home/youssef/Desktop/CRAC2026/Piste.png")  # chemin vers ton image
    img = mpimg.imread("Piste.png")  # chemin vers ton image
    img = np.rot90(img, 2)  # rotation de 180°

    fig, ax = plt.subplots(figsize=(600/100, 400/100), dpi=100)
    plt.ion()
    plt.show()

    # --- Objets graphiques ---
    robot_plot = ax.scatter([], [], s=50, c='blue', marker='o', label="Robot")
    ennemi_plot = ax.scatter([], [], s=50, c='red', marker='o', label="Ennemi")
    consigne_plot = ax.scatter([], [], s=50, c='green', marker='x', label="Consigne")
    scat = ax.scatter([], [], s=5, c='blue', alpha=0.5, label="Points Lidar")

    ax.imshow(img, extent=[0, 3000, 0, 2000], origin='lower')
    ax.set_xlim(0, 3000)
    ax.set_ylim(0, 2250)
    ax.set_aspect('equal', adjustable='box')

    # --- Texte d’info du robot ---
    robot_info_text = ax.text(
        1500, 2200,
        "",
        color='black',
        fontsize=8,
        ha='left',
        va='top'
    )
    x_voulu_text = ax.text(
        1500, 2100,
        "",
        color='black',
        fontsize=8,
        ha='left',
        va='top'
    )
    y_voulu_text = ax.text(
        2000, 2100,
        "",
        color='black',
        fontsize=8,
        ha='left',
        va='top'
    )
    A_voulu_text = ax.text(
        2500, 2100,
        "",
        color='black',
        fontsize=8,
        ha='left',
        va='top'
    )
    # --- Bouton STOP ---
    ax_button = plt.axes([0.07, 0.93, 0.15, 0.05])
    bouton_stop = Button(ax_button, 'STOP', color='red', hovercolor='orange')
    bouton_stop.label.set_fontsize(11)
    bouton_stop.label.set_color('white')

    # --- Point voulu ---
    point_voulu_plot, = ax.plot([], [], 'bx', markersize=10, label="Point voulu")

    robot_angle_line = Line2D(
        [0, 0 + 0 * np.cos(np.radians(0))],
        [0, 0 + 0 * np.sin(np.radians(0))],
        color='blue', linewidth=2
    )

    
    robot_angle_voulu_line = Line2D(
        [0, 0 + 0 * np.cos(np.radians(0))],
        [0, 0 + 0 * np.sin(np.radians(0))],
        color='green', linewidth=2
    )
    
    cercle_ennemi = plt.Circle((0, 0), R_securite, color='orange', fill=False, linestyle='--')

    return fig, ax, robot_plot, ennemi_plot, consigne_plot, scat, robot_info_text,ax_button,bouton_stop,point_voulu_plot,x_voulu_text,y_voulu_text,A_voulu_text, robot_angle_line,robot_angle_voulu_line,cercle_ennemi

