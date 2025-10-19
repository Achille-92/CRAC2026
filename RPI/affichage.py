# affichage.py
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from matplotlib.widgets import Button

import matplotlib
import tkinter as tk

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

def init_affichage():
    """
    Initialise la fenêtre graphique pour le robot et l'ennemi.
    Retourne les objets utiles : figure, axes, plots et boutons.
    """
    # --- Chargement et configuration de l’image ---
    img = mpimg.imread("/home/youssef/Desktop/CRAC2026/piste.png")  # chemin vers ton image
    img = np.rot90(img, 2)  # rotation de 180°

    fig, ax = plt.subplots(figsize=(600/100, 400/100), dpi=100)
    plt.ion()
    plt.show()

    # --- Objets graphiques ---
    robot_plot = ax.scatter([], [], s=50, c='red', marker='x', label="Robot")
    ennemi_plot = ax.scatter([], [], s=50, c='green', marker='o', label="Ennemi")
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
    coordonnee_voulues_text = ax.text(
        1500, 2100,
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

    return fig, ax, robot_plot, ennemi_plot, scat, robot_info_text,ax_button,bouton_stop,point_voulu_plot,coordonnee_voulues_text

