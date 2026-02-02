"""
calcul_mouv.py
Adaptation du programme A* isolé au programme principal
SEULE différence : génération des obstacles via obs_manager
"""

import heapq
import math
import numpy as np
from scipy.interpolate import splprep, splev
from scipy.ndimage import binary_dilation, distance_transform_edt


# === CONSTANTES ===
DIRECTIONS = [
    ((1, 0), 1.0), ((-1, 0), 1.0), ((0, 1), 1.0), ((0, -1), 1.0),
    ((1, 1), 1.414), ((-1, -1), 1.414), ((1, -1), 1.414), ((-1, 1), 1.414)
]


# === FONCTIONS UTILITAIRES ===

def euclidienne(a, b):
    """Calcule la distance euclidienne entre deux points"""
    dx, dy = a[0] - b[0], a[1] - b[1]
    return math.sqrt(dx*dx + dy*dy)


def voisins_rapide(x, y, grid_expanded, width, height):
    """Retourne les voisins accessibles d'une case"""
    result = []
    for (dx, dy), cost in DIRECTIONS:
        nx, ny = x + dx, y + dy
        if 0 <= nx < width and 0 <= ny < height and not grid_expanded[nx, ny]:
            result.append(((nx, ny), cost))
    return result


# === A* AVEC PONDÉRATION PAR DISTANCE AUX OBSTACLES ===

def astar_safe(start, goal, grid_expanded, distance_map, width, height, safety_weight, DISTANCE_AJUSTABLE):
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
            # Reconstruire le chemin
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path
        
        current_g = g_score[current]
        x, y = int(current[0]), int(current[1])
        
        for (neighbor, cost) in voisins_rapide(x, y, grid_expanded, width, height):
            if neighbor in closed_set:
                continue
            
            nx, ny = int(neighbor[0]), int(neighbor[1])
            
            # Pénalité inversement proportionnelle à la distance aux obstacles
            dist_to_obstacle = distance_map[nx, ny]
            if dist_to_obstacle < DISTANCE_AJUSTABLE:
                safety_penalty = safety_weight * (DISTANCE_AJUSTABLE - dist_to_obstacle)
            else:
                safety_penalty = 0
            
            tentative_g = current_g + cost + safety_penalty
            
            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + euclidienne(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))
    
    return None


# === SIMPLIFICATION SÉCURISÉE DU CHEMIN ===

def simplify_path_safe(path, grid_expanded, distance_map, width, height, min_clearance):
    """Simplifie le chemin en gardant une distance minimale aux obstacles"""
    if len(path) <= 2:
        return path
    
    simplified = [path[0]]
    i = 0
    
    while i < len(path) - 1:
        j = len(path) - 1
        found = False
        
        while j > i + 1:
            if is_line_clear_safe(path[i], path[j], grid_expanded, distance_map, width, height, min_clearance):
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


def is_line_clear_safe(p1, p2, grid_expanded, distance_map, width, height, min_clearance):
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


# === LISSAGE SÉCURISÉ ===

def smooth_path_safe(path, grid_expanded, distance_map, width, height, smoothness):
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


# === FONCTION PRINCIPALE ===

def calculer_trajectoire_complete(x_robot_actuel, y_robot_actuel, 
                                   x_robot_voulu, y_robot_voulu,
                                   obs_manager, obs_manager_noisettes,
                                   x_ennemi, y_ennemi, R_securite,
                                   CASE_MM, terrain_w_mm, terrain_h_mm,
                                   safety_weight, MIN_CLEARANCE, SMOOTHNESS, DISTANCE_AJUSTABLE,
                                   affichage_ax=None):
    """
    Calcule une trajectoire complète de A à B.
    Basé sur le programme A* isolé qui fonctionne.
    Inclut le robot ennemi comme zone interdite.
    """
    
    print("\n" + "="*70)
    print("🚀 CALCUL DE TRAJECTOIRE A*")
    print("="*70)
    
    # 1. DIMENSIONS
    width = terrain_w_mm // CASE_MM
    height = terrain_h_mm // CASE_MM
    
    # 2. GÉNÉRER LES GRILLES (via obs_manager au lieu de les créer manuellement)
    grid_zones, grid_zones_expanded, _, _ = obs_manager.generer_grille()
    grid_noisettes, grid_noisettes_expanded, _, _ = obs_manager_noisettes.generer_grille()
    
    # 2b. CRÉER LA GRILLE POUR LE ROBOT ENNEMI
    from fonction import Obstacles
    # Créer un cercle de rayon R_securite (déjà la zone de sécurité complète)
    # On met MARGE=0 car on ne veut pas d'expansion supplémentaire
    obs_manager_ennemi = Obstacles(terrain_w_mm, terrain_h_mm, 0, 0, CASE_MM)
    obs_manager_ennemi.ajouter_cercle("ennemi", (int(x_ennemi), int(y_ennemi)), int(R_securite), actif=True)
    grid_ennemi, _, _, _ = obs_manager_ennemi.generer_grille()
    # Utiliser grid_ennemi pour les deux (grid et grid_expanded) car pas d'expansion supplémentaire
    grid_ennemi_expanded = grid_ennemi
    
    # Combiner les grilles (zones + noisettes + ennemi)
    grid = np.logical_or(np.logical_or(grid_zones, grid_noisettes), grid_ennemi).astype(bool)
    grid_expanded = np.logical_or(np.logical_or(grid_zones_expanded, grid_noisettes_expanded), grid_ennemi_expanded).astype(bool)
    
    print(f"✅ Grilles générées : {width}×{height} cases")
    
    # 3. CALCULER distance_map DEPUIS grid_expanded (comme dans le programme isolé)
    distance_map = distance_transform_edt(~grid_expanded)
    
    print(f"✅ distance_map calculée (max: {distance_map.max():.1f} cases)")
    
    # 4. CONVERTIR POSITIONS EN CASES
    start = (int(x_robot_actuel // CASE_MM), int(y_robot_actuel // CASE_MM))
    goal = (int(x_robot_voulu // CASE_MM), int(y_robot_voulu // CASE_MM))
    
    print(f"📍 Départ : {start} ({x_robot_actuel}, {y_robot_actuel} mm)")
    print(f"🎯 Arrivée : {goal} ({x_robot_voulu}, {y_robot_voulu} mm)")
    
    # 5. CALCULER LE CHEMIN AVEC A*
    import time
    t0 = time.time()
    
    path = astar_safe(start, goal, grid_expanded, distance_map, 
                     width, height, safety_weight, DISTANCE_AJUSTABLE)
    
    if not path:
        print("❌ AUCUN CHEMIN TROUVÉ")
        return None
    
    t1 = time.time()
    print(f"✅ A* terminé en {(t1-t0)*1000:.1f} ms : {len(path)} points")
    
    # 6. SIMPLIFIER LE CHEMIN
    path_simplified = simplify_path_safe(path, grid_expanded, distance_map, 
                                        width, height, MIN_CLEARANCE)
    
    print(f"📉 Chemin simplifié : {len(path)} → {len(path_simplified)} points")
    
    # 7. LISSER LE CHEMIN
    x_smooth, y_smooth = smooth_path_safe(path_simplified, grid_expanded, 
                                         distance_map, width, height, SMOOTHNESS)
    
    # 8. CONVERTIR EN MM
    """if x_smooth is not None and y_smooth is not None:
        x_smooth_mm = [int(x * CASE_MM) for x in x_smooth]
        y_smooth_mm = [int(y * CASE_MM) for y in y_smooth]
        points_bruts = [[x_smooth_mm[i], y_smooth_mm[i]] for i in range(len(x_smooth_mm))]
    else:"""
    px_mm = [int(x * CASE_MM) for x, y in path_simplified]
    py_mm = [int(y * CASE_MM) for x, y in path_simplified]
    points_bruts = [[px_mm[i], py_mm[i]] for i in range(len(px_mm))]
    
    # 9. CALCULER LONGUEUR
    longueur_mm = 0
    for i in range(len(points_bruts)-1):
        dx = points_bruts[i+1][0] - points_bruts[i][0]
        dy = points_bruts[i+1][1] - points_bruts[i][1]
        longueur_mm += math.sqrt(dx*dx + dy*dy)
    
    print(f"📏 Longueur trajectoire : {longueur_mm:.0f} mm")
    print(f"✅ Trajectoire calculée : {len(points_bruts)} points")
    print("="*70 + "\n")
    
    
    return points_bruts


# === FONCTION UTILITAIRE ===

def verifier_cible_disponible(action_en_cours, 
                               Liste_noisettes_libres, 
                               Liste_GM_libres):
    """Vérifie si la cible actuelle est toujours disponible"""
    if action_en_cours is None:
        return False
    
    type_action = action_en_cours.get('type')
    numero_zone = action_en_cours.get('numero_zone')
    
    if type_action == "Attraper":
        if numero_zone in Liste_noisettes_libres:
            return True
        else:
            print(f"⚠️  CIBLE PERDUE : Noisette {numero_zone} n'est plus disponible !")
            return False
    
    elif type_action == "Relacher":
        if numero_zone in Liste_GM_libres:
            return True
        else:
            print(f"⚠️  CIBLE PERDUE : GM {numero_zone} n'est plus disponible !")
            return False
    
    return False

def actualiser_zones_jeu(grid, grid_expanded, obstacle_array, expanded_array, 
                         obs_manager, obs_manager_noisettes,
                         Liste_noisette_xya,  # ⭐ NOUVEAU PARAMÈTRE
                         obstacle_scatter, expanded_scatter, distance_map, 
                         ax, width, height, CASE_MM):
    """
    Met à jour les grilles en fonction des noisettes présentes dans Liste_noisette_xya.
    
    Args:
        Liste_noisette_xya: Liste des noisettes [[x, y, angle, couleur], ...]
    
    Returns:
        Toutes les variables mises à jour
    """
    from scipy.ndimage import distance_transform_edt
    import numpy as np
    
    print("🔄 Actualisation des zones de jeu basée sur Liste_noisette_xya")
    
    # 1️⃣ RÉINITIALISER obs_manager_noisettes
    # Supprimer TOUTES les noisettes existantes
    # ⭐ CORRECTION : Utiliser _obstacles au lieu de obstacles
    noisettes_a_supprimer = [nom for nom in obs_manager_noisettes._obstacles.keys()]
    for nom in noisettes_a_supprimer:
        obs_manager_noisettes.retirer(nom)  # ⭐ CORRECTION : retirer au lieu de supprimer_obstacle
    
    # 2️⃣ AJOUTER les noisettes présentes dans Liste_noisette_xya
    for i, noisette_data in enumerate(Liste_noisette_xya, 1):
        if len(noisette_data) >= 3:
            x, y, angle = noisette_data[0], noisette_data[1], noisette_data[2]
            obs_manager_noisettes.ajouter_rectangle_oriente(
                f"Noisette{i}", 
                (x, y),
                150,   # Longueur
                50,    # Largeur
                angle,
                actif=True
            )
    
    # 3️⃣ RÉGÉNÉRER les grilles
    grid_zones, grid_zones_expanded, obstacle_array_zones, expanded_array_zones = obs_manager.generer_grille()
    grid_noisettes, grid_noisettes_expanded, obstacle_array_noisettes, expanded_array_noisettes = obs_manager_noisettes.generer_grille()
    
    # Combiner les grilles
    grid = np.logical_or(grid_zones, grid_noisettes).astype(int)
    grid_expanded = np.logical_or(grid_zones_expanded, grid_noisettes_expanded).astype(int)
    
    # Combiner les arrays pour l'affichage
    if len(obstacle_array_zones) > 0 and len(obstacle_array_noisettes) > 0:
        obstacle_array = np.vstack([obstacle_array_zones, obstacle_array_noisettes])
        expanded_array = np.vstack([expanded_array_zones, expanded_array_noisettes])
    elif len(obstacle_array_zones) > 0:
        obstacle_array = obstacle_array_zones
        expanded_array = expanded_array_zones
    else:
        obstacle_array = obstacle_array_noisettes
        expanded_array = expanded_array_noisettes
    
    # 4️⃣ RECALCULER la carte de distance
    distance_map = distance_transform_edt(~grid_expanded)
    
    # 5️⃣ METTRE À JOUR l'affichage
    if obstacle_scatter is not None:
        obstacle_scatter.remove()
    if expanded_scatter is not None:
        expanded_scatter.remove()
    
    from affichage import afficher_obstacles
    obstacle_scatter, expanded_scatter = afficher_obstacles(
        ax, obstacle_array, expanded_array, CASE_MM,
        show_expanded=True, show_obstacles=False
    )
    
    print(f"✅ Grilles actualisées : {len(Liste_noisette_xya)} noisettes actives")
    
    return (grid, grid_expanded, obstacle_array, expanded_array,
            obs_manager, obs_manager_noisettes,
            obstacle_scatter, expanded_scatter, distance_map,
            ax, width, height, CASE_MM)




def verifier_segments_trajectoire_ennemi(Liste_actions, x_ennemi, y_ennemi, R_securite, MARGE_TRAJECTOIRE,x_robot_actuel, y_robot_actuel):
    """
    Vérifie si un segment de la trajectoire (entre deux points "Avancer" ou "Consigne"/"Avancer")
    passe dans la zone de sécurité de l'ennemi.
    Si oui, retourne True pour demander un recalcul de trajectoire.
    """

    # Extraire tous les points "Avancer" ou "Consigne" de Liste_actions
    points_trajectoire = [(x_robot_actuel,y_robot_actuel)]
    for action in Liste_actions:
        if isinstance(action, list) and len(action) >= 3 and action[0] in ["Avancer", "Consigne","Reculer"]:
            points_trajectoire.append((action[1], action[2]))

    # Si moins de 2 points, pas de segment à vérifier
    if len(points_trajectoire) < 2:
        return False

    # Parcourir chaque segment consécutif
    for i in range(len(points_trajectoire) - 1):
        x1, y1 = points_trajectoire[i]
        x2, y2 = points_trajectoire[i + 1]

        # Calculer la distance minimale entre ce segment et l'ennemi
        distance_min = distance_segment_point(x1, y1, x2, y2, x_ennemi, y_ennemi)

        # Si la distance est inférieure à la marge de sécurité, demander un recalcul
        if distance_min <= (R_securite):
            print(f"⚠️ Segment {i} ({x1},{y1})→({x2},{y2}) trop proche de l'ennemi (distance={distance_min:.1f} < {R_securite + MARGE_TRAJECTOIRE})")
            return True

    return False

def distance_segment_point(x1, y1, x2, y2, px, py):
    """
    Calcule la distance minimale entre un segment (x1,y1)-(x2,y2) et un point (px,py).
    """
    # Vecteur segment
    seg_x = x2 - x1
    seg_y = y2 - y1
    seg_length_sq = seg_x**2 + seg_y**2

    # Cas particulier : segment de longueur nulle
    if seg_length_sq == 0:
        return math.sqrt((px - x1)**2 + (py - y1)**2)

    # Projection du point sur le segment
    t = max(0, min(1, ((px - x1) * seg_x + (py - y1) * seg_y) / seg_length_sq))
    proj_x = x1 + t * seg_x
    proj_y = y1 + t * seg_y

    # Distance entre le point et sa projection
    return math.sqrt((px - proj_x)**2 + (py - proj_y)**2)
