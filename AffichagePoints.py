import matplotlib.pyplot as plt

def afficher_points(liste_points):
    xs = [p[0] for p in liste_points]
    ys = [p[1] for p in liste_points]
    
    plt.clf()                          # Efface le graphique précédent
    plt.scatter(xs, ys, s=5, c='blue') # Affiche les points
    plt.xlim(0, 3000)                  # Limites du repère
    plt.ylim(0, 2000)
    plt.gca().set_aspect('equal')      # Même échelle en X et Y
    plt.pause(0.01)                    # Met à jour le graphique sans bloquer
