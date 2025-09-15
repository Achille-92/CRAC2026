import matplotlib.pyplot as plt

# Exemple de liste de points
points = [(1, 2), (2, 3), (3, 1), (4, 4)]
# Point r
x_r, y_r = 2.5, 2.5

# Séparation des coordonnées pour matplotlib
x_points, y_points = zip(*points)

# Création de la figure
plt.figure(figsize=(6,6), facecolor="white")

# Affichage des points en bleu
plt.scatter(x_points, y_points, color="blue", label="Points")

# Affichage du point r en rouge
plt.scatter([x_r], [y_r], color="red", label="Point r")

# Optionnel : ajouter une grille et une légende
plt.grid(True, linestyle="--", alpha=0.5)
plt.legend()

# Afficher la fenêtre
plt.show()
