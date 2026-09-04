import heapq
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev

# === PARAMÈTRES DU TERRAIN ===
case_mm = 10
terrain_w_mm, terrain_h_mm = 3000, 2000
width, height = terrain_w_mm // case_mm, terrain_h_mm // case_mm

# NOUVEAU : Marges de sécurité plus importantes
rayon_robot_mm = 100
marge_securite_mm = 50  # Augmenté de 20 à 50 mm
rayon_total_case = (rayon_robot_mm + marge_securite_mm) // case_mm  # ~15 cases

start = (15, 15)
goal = (285, 185)

# === DÉFINITION DES ZONES INTERDITES ===
zones_centres_mm = [
    (1250, 1450), (1750, 1450),
    (100, 800), (800, 800), (1500, 800), (2200, 800), (2900, 800),
    (700, 100), (1500, 100), (2300, 100),
]
zones_cote_mm = 200
zones_cote_case = zones_cote_mm // case_mm // 2

# === CRÉATION DES OBSTACLES ===
obstacles = set()
for (cx_mm, cy_mm) in zones_centres_mm:
    cx, cy = cx_mm // case_mm, cy_mm // case_mm
    for x in range(cx - zones_cote_case, cx + zones_cote_case):
        for y in range(cy - zones_cote_case, cy + zones_cote_case):
            if 0 <= x < width and 0 <= y < height:
                obstacles.add((x, y))

# Zone rectangulaire
x_start_case, x_end_case = 60, 240
y_start_case, y_end_case = 155, 200
for x in range(x_start_case, x_end_case):
    for y in range(y_start_case, y_end_case):
        obstacles.add((x, y))

# Grille numpy
grid = np.zeros((width, height), dtype=bool)
for (x, y) in obstacles:
    grid[x, y] = True

# Expansion des obstacles
from scipy.ndimage import binary_dilation
structure = np.zeros((2*rayon_total_case+1, 2*rayon_total_case+1))
y_grid, x_grid = np.ogrid[-rayon_total_case:rayon_total_case+1, -rayon_total_case:rayon_total_case+1]
mask = x_grid*x_grid + y_grid*y_grid <= rayon_total_case*rayon_total_case
structure[mask] = 1
grid_expanded = binary_dilation(grid, structure=structure)

# NOUVEAU : Carte de distance aux obstacles (pour A* pondéré)
from scipy.ndimage import distance_transform_edt
distance_map = distance_transform_edt(~grid_expanded)

# === FONCTIONS UTILITAIRES ===
def euclidienne(a, b):
    dx, dy = a[0] - b[0], a[1] - b[1]
    return math.sqrt(dx*dx + dy*dy)

DIRECTIONS = [
    ((1, 0), 1.0), ((-1, 0), 1.0), ((0, 1), 1.0), ((0, -1), 1.0),
    ((1, 1), 1.414), ((-1, -1), 1.414), ((1, -1), 1.414), ((-1, 1), 1.414)
]

def voisins_rapide(x, y):
    result = []
    for (dx, dy), cost in DIRECTIONS:
        nx, ny = x + dx, y + dy
        if 0 <= nx < width and 0 <= ny < height and not grid_expanded[nx, ny]:
            result.append(((nx, ny), cost))
    return result

# === A* AVEC PONDÉRATION PAR DISTANCE AUX OBSTACLES ===
def astar_safe(start, goal, safety_weight=2.0):
    """A* qui favorise les chemins éloignés des obstacles"""
    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}
    closed_set = set()
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current in closed_set:
            continue
        closed_set.add(current)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        current_g = g_score[current]
        x, y = int(current[0]), int(current[1])  # CORRECTION : conversion en int
        
        for (neighbor, cost) in voisins_rapide(x, y):
            if neighbor in closed_set:
                continue
            
            nx, ny = int(neighbor[0]), int(neighbor[1])  # CORRECTION : conversion en int
            
            # NOUVEAU : Pénalité inversement proportionnelle à la distance aux obstacles
            # Plus on est proche d'un obstacle, plus le coût augmente
            dist_to_obstacle = distance_map[nx, ny]
            if dist_to_obstacle < 5:  # Très proche d'un obstacle
                safety_penalty = safety_weight * (5 - dist_to_obstacle)
            else:
                safety_penalty = 0
            
            tentative_g = current_g + cost + safety_penalty
            
            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + euclidienne(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
    
    return None

# === EXÉCUTION ===
import time
t0 = time.time()
path = astar_safe(start, goal, safety_weight=3.0)  # Poids de sécurité élevé

# === SIMPLIFICATION SÉCURISÉE DU CHEMIN ===
def simplify_path_safe(path, min_clearance=3):
    """Simplifie le chemin en gardant une distance minimale aux obstacles"""
    if len(path) <= 2:
        return path
    
    simplified = [path[0]]
    i = 0
    
    while i < len(path) - 1:
        j = len(path) - 1
        found = False
        
        while j > i + 1:
            if is_line_clear_safe(path[i], path[j], min_clearance):
                simplified.append(path[j])
                i = j
                found = True
                break
            j -= 1
        
        if not found:
            i += 1
            if i < len(path):
                simplified.append(path[i])
    
    return simplified

def is_line_clear_safe(p1, p2, min_clearance=3):
    """Vérifie si la ligne est libre ET à distance minimale des obstacles"""
    x0, y0 = p1
    x1, y1 = p2
    
    # Algorithme de Bresenham
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    
    if dx > dy:
        error = dx / 2
        while x != x1:
            points.append((x, y))
            error -= dy
            if error < 0:
                y += y_inc
                error += dx
            x += x_inc
    else:
        error = dy / 2
        while y != y1:
            points.append((x, y))
            error -= dx
            if error < 0:
                x += x_inc
                error += dy
            y += y_inc
    points.append((x1, y1))
    
    # Vérifier que tous les points sont libres ET à distance minimale
    for x, y in points:
        if x < 0 or x >= width or y < 0 or y >= height:
            return False
        if grid_expanded[x, y]:
            return False
        if distance_map[x, y] < min_clearance:
            return False
    
    return True

# Simplification avec distance de sécurité
if path:
    path_simplified = simplify_path_safe(path, min_clearance=3)
    print(f"📉 Chemin simplifié : {len(path)} → {len(path_simplified)} points")
else:
    path_simplified = None

# === LISSAGE SÉCURISÉ ===
def smooth_path_safe(path, smoothness=3.0):
    """Lisse le chemin en vérifiant la sécurité"""
    if not path or len(path) < 4:
        return None, None
    
    x, y = zip(*path)
    
    # Essayer différents niveaux de lissage
    for s in [smoothness, smoothness*2, smoothness*4]:
        try:
            tck, u = splprep([x, y], s=s, k=min(3, len(path)-1))
            u_fine = np.linspace(0, 1, len(path) * 4)
            x_smooth, y_smooth = splev(u_fine, tck)
            
            # Vérifier que la spline est sûre
            safe = True
            for i in range(len(x_smooth)):
                xi, yi = int(round(x_smooth[i])), int(round(y_smooth[i]))
                if xi < 0 or xi >= width or yi < 0 or yi >= height:
                    safe = False
                    break
                if grid_expanded[xi, yi] or distance_map[xi, yi] < 2:
                    safe = False
                    break
            
            if safe:
                return x_smooth, y_smooth
        except:
            continue
    
    # Si aucun lissage ne fonctionne, retourner le chemin simplifié
    return x, y

x_smooth, y_smooth = smooth_path_safe(path_simplified, smoothness=2.0)

# === MÉTRIQUES ===
if path:
    path_length = sum(euclidienne(path[i], path[i+1]) for i in range(len(path)-1))
    path_length_mm = path_length * case_mm
    print(f"📏 Longueur : {path_length_mm:.0f} mm")
    
    if path_simplified:
        simplified_length = sum(euclidienne(path_simplified[i], path_simplified[i+1]) 
                               for i in range(len(path_simplified)-1))
        print(f"📏 Longueur simplifiée : {simplified_length * case_mm:.0f} mm")

t1 = time.time()
print(f"⏱️  Calcul A* : {(t1-t0)*1000:.1f} ms")

# === VISUALISATION AMÉLIORÉE ===
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

for ax in [ax1, ax2]:
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linestyle=':', alpha=0.3)

try:
    background_image = plt.imread('Piste.png')  # Changez le nom du fichier
    for ax in [ax1, ax2]:
        ax.imshow(background_image, 
                  extent=[0, width, 0, height],  # Correspond à vos dimensions
                  aspect='auto', 
                  alpha=0.4,  # Transparence (0=invisible, 1=opaque)
                  zorder=0)   # Arrière-plan
        
except FileNotFoundError:
    print("⚠️ Image de fond non trouvée, affichage sans fond")

# Graphique 1 : Vue complète
ax1.set_title(f"Vue complète - Rayon {rayon_robot_mm}mm + marge {marge_securite_mm}mm")
obstacle_array = np.argwhere(grid)
ax1.scatter(obstacle_array[:, 0], obstacle_array[:, 1], c='darkgray', s=2, alpha=0.8, label='Obstacles')
expanded_array = np.argwhere(grid_expanded & ~grid)
ax1.scatter(expanded_array[:, 0], expanded_array[:, 1], c='lightcoral', s=1, alpha=0.4, label='Zone interdite')

if path:
    px, py = zip(*path)
    ax1.plot(px, py, 'b-', linewidth=1, alpha=0.3, label='A* brut')
if path_simplified:
    px, py = zip(*path_simplified)
    ax1.plot(px, py, 'g-', linewidth=2, label='Simplifié', marker='o', markersize=5)
if x_smooth is not None:
    ax1.plot(x_smooth, y_smooth, 'orange', linewidth=3, label='Trajectoire finale', zorder=4)

ax1.scatter(start[0], start[1], color='green', s=250, label='Départ', zorder=5, edgecolors='black', linewidths=2)
ax1.scatter(goal[0], goal[1], color='red', s=250, label='Arrivée', zorder=5, edgecolors='black', linewidths=2)
ax1.legend(loc='upper left')

# Graphique 2 : Carte de distance aux obstacles
ax2.set_title("Carte de distance aux obstacles")
im = ax2.imshow(distance_map.T, origin='lower', cmap='viridis', alpha=0.6, extent=[0, width, 0, height])
plt.colorbar(im, ax=ax2, label='Distance (cases)')

if x_smooth is not None:
    ax2.plot(x_smooth, y_smooth, 'orange', linewidth=3, label='Trajectoire finale')
if path_simplified:
    px, py = zip(*path_simplified)
    ax2.plot(px, py, 'r--', linewidth=1.5, alpha=0.7, label='Points clés')
    ax2.scatter(px, py, color='red', s=30, zorder=5)

ax2.scatter(start[0], start[1], color='green', s=250, zorder=6, edgecolors='black', linewidths=2)
ax2.scatter(goal[0], goal[1], color='red', s=250, zorder=6, edgecolors='black', linewidths=2)
ax2.legend()

plt.tight_layout()
plt.show()
