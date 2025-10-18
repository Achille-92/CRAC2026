# graphique.py
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

def init_affichage():
    """
    Initialise la fenêtre graphique pour le robot et l'ennemi.
    
    Args:
        x_robot_depart (float): Position x initiale du robot
        y_robot_depart (float): Position y initiale du robot
        image_path (str): Chemin vers l'image de la piste

    Returns:
        tuple: fig, ax, robot_plot, ennemi_plot, scat
    """
    # Objets et variables pour la fenêtre graphique
    img = mpimg.imread("piste.png")  # ou le chemin complet vers ton image
    img = np.rot90(img, 2)  # rotation de 180° (2 x 90°)

    fig, ax = plt.subplots(figsize=(600/100, 400/100), dpi=100)
    plt.ion()
    plt.show()

    # Scatter pour les objets
    robot_plot = ax.scatter([], [], s=50, c='red', marker='x')
    ennemi_plot = ax.scatter([], [], s=50, c='green', marker='o')
    scat = ax.scatter([], [], s=5, c='blue', alpha=0.5)

    ax.imshow(img, extent=[0, 3000, 0, 2000], origin='lower')

    # Fixer les limites du repère
    ax.set_xlim(0, 3000)
    ax.set_ylim(0, 2250)  # un peu plus haut que la piste
    ax.set_aspect('equal', adjustable='box')

    robot_info_text = ax.text(
        1500, 2100,  # position sur la figure (x, y)
        "",        # texte initial vide
        color='black',
        fontsize=8,
        ha='left',
        va='top'
    )

    return fig, ax, robot_plot, ennemi_plot, scat, robot_info_text
