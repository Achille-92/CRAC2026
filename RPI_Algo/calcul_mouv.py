"""
calcul_mouv.py - Module pour le calcul de trajectoires avec A*
Toutes les fonctions nécessaires au calcul de mouvement du robot
"""

import math
import numpy as np
import heapq
from scipy.ndimage import distance_transform_edt
from scipy.interpolate import splprep, splev

# === CONSTANTES ===
DIRECTIONS = [
    ((1, 0), 1.0), ((-1, 0), 1.0), ((0, 1), 1.0), ((0, -1), 1.0),
    ((1, 1), 1.414), ((-1, -1), 1.414), ((1, -1), 1.414), ((-1, 1), 1.414)
]


# === CLASSE POUR GÉRER LES VARIABLES GLOBALES ===
class ConfigGrille:
    """Classe pour stocker la configuration de la grille"""
    def __init__(self, grid_expanded, distance_map, width, height, case_mm=10):
        self.grid_expanded = grid_expanded
        self.distance_map = distance_map
        self.width = width
        self.height = height
        self.case_mm = case_mm

# Variable globale pour la configuration (sera initialisée)
_config = None

def initialiser_grille(grid_expanded, distance_map, width, height, case_mm):
    """
    Initialise la configuration de grille pour toutes les fonctions du module.
    DOIT être appelé une fois au démarrage du programme principal.
    
    Args:
        grid_expanded: Grille numpy booléenne des obstacles expandés
        distance_map: Carte des distances aux obstacles
        width: Largeur de la grille (en cases)
        height: Hauteur de la grille (en cases)
        case_mm: Taille d'une case en mm
    """
    global _config
    _config = ConfigGrille(grid_expanded, distance_map, width, height, case_mm)
    print(f"✅ Grille initialisée : {width}×{height} cases ({width*case_mm}×{height*case_mm} mm)")


# === FONCTIONS UTILITAIRES ===

def euclidienne(a, b):
    """Distance euclidienne entre deux points"""
    dx, dy = a[0] - b[0], a[1] - b[1]
    return math.sqrt(dx*dx + dy*dy)


def voisins_rapide(x, y, grille=None):
    """
    Retourne les voisins accessibles d'une case.
    
    Args:
        x, y: Position de la case
        grille: Grille à utiliser (optionnel, sinon utilise grid_expanded global)
    
    Returns:
        Liste de tuples ((x, y), coût)
    """
    if _config is None:
        raise RuntimeError("Grille non initialisée ! Appeler initialiser_grille() d'abord.")
    
    result = []
    grille_utilisee = grille if grille is not None else _config.grid_expanded
    
    for (dx, dy), cost in DIRECTIONS:
        nx, ny = x + dx, y + dy
        if 0 <= nx < _config.width and 0 <= ny < _config.height and not grille_utilisee[nx, ny]:
            result.append(((nx, ny), cost))
    return result


# === A* AVEC PONDÉRATION PAR DISTANCE AUX OBSTACLES ===

def astar_safe(start, goal, safety_weight=2.0, grid_dynamique=None):
    """
    A* qui favorise les chemins éloignés des obstacles.
    
    Args:
        start: Tuple (x, y) position de départ en cases
        goal: Tuple (x, y) position d'arrivée en cases
        safety_weight: Poids de pénalité pour proximité obstacles (2.0-4.0)
        grid_dynamique: Grille incluant obstacles dynamiques (optionnel)
    
    Returns:
        Liste de tuples (x, y) représentant le chemin, ou None si aucun chemin
    """
    if _config is None:
        raise RuntimeError("Grille non initialisée ! Appeler initialiser_grille() d'abord.")
    
    # Utiliser grid_dynamique si fourni, sinon grid_expanded
    grille = grid_dynamique if grid_dynamique is not None else _config.grid_expanded
    
    # Recalculer distance_map si grille dynamique
    if grid_dynamique is not None:
        distance_map_local = distance_transform_edt(~grid_dynamique)
    else:
        distance_map_local = _config.distance_map
    
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
        
        # Vérifier voisins avec la grille dynamique
        for (dx, dy), cost in DIRECTIONS:
            nx, ny = x + dx, y + dy
            neighbor = (nx, ny)
            
            # Vérifier limites et obstacles
            if not (0 <= nx < _config.width and 0 <= ny < _config.height and not grille[nx, ny]):
                continue
            
            if neighbor in closed_set:
                continue
            
            # Pénalité de sécurité (plus on est proche d'un obstacle, plus c'est cher)
            dist_to_obstacle = distance_map_local[nx, ny]
            if dist_to_obstacle < 5:
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


# === SIMPLIFICATION SÉCURISÉE DU CHEMIN ===

def simplify_path_safe(path, min_clearance):
    """
    Simplifie le chemin en gardant une distance minimale aux obstacles.
    
    Args:
        path: Liste de tuples (x, y) représentant le chemin
        min_clearance: Distance minimale aux obstacles en cases
    
    Returns:
        Chemin simplifié (liste de tuples)
    """
    if _config is None:
        raise RuntimeError("Grille non initialisée ! Appeler initialiser_grille() d'abord.")
    
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


def is_line_clear_safe(p1, p2, min_clearance):
    """
    Vérifie si la ligne entre deux points est libre ET à distance minimale des obstacles.
    Utilise l'algorithme de Bresenham.
    """
    if _config is None:
        raise RuntimeError("Grille non initialisée ! Appeler initialiser_grille() d'abord.")
    
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
        if x < 0 or x >= _config.width or y < 0 or y >= _config.height:
            return False
        if _config.grid_expanded[x, y]:
            return False
        if _config.distance_map[x, y] < min_clearance:
            return False
    
    return True


# === LISSAGE SÉCURISÉ ===

def smooth_path_safe(path, smoothness):
    """
    Lisse le chemin en vérifiant la sécurité avec des splines cubiques.
    
    Args:
        path: Liste de tuples (x, y) représentant le chemin
        smoothness: Paramètre de lissage (plus élevé = plus lisse)
    
    Returns:
        Tuple (x_smooth, y_smooth) ou (x, y) si échec
    """
    if _config is None:
        raise RuntimeError("Grille non initialisée ! Appeler initialiser_grille() d'abord.")
    
    if not path or len(path) < 4:
        if path:
            x, y = zip(*path)
            return x, y
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
                if xi < 0 or xi >= _config.width or yi < 0 or yi >= _config.height:
                    safe = False
                    break
                if _config.grid_expanded[xi, yi] or _config.distance_map[xi, yi] < 2:
                    safe = False
                    break
            
            if safe:
                return x_smooth, y_smooth
        except:
            continue
    
    # Si aucun lissage ne fonctionne, retourner le chemin simplifié
    return x, y


# === GRILLE DYNAMIQUE AVEC ENNEMI ===

def creer_grille_avec_ennemi(grid_expanded_statique, x_ennemi, y_ennemi, 
                               r_robot, r_ennemi, marge_min, case_mm=10):
    """
    Crée une grille combinant les obstacles statiques et la zone de l'ennemi.
    
    Args:
        grid_expanded_statique: Grille des obstacles statiques déjà expansés
        x_ennemi: Position X de l'ennemi en mm
        y_ennemi: Position Y de l'ennemi en mm
        r_robot: Rayon du robot en mm
        r_ennemi: Rayon de l'ennemi en mm
        marge_min: Marge de sécurité en mm
        case_mm: Taille d'une case en mm
        
    Returns:
        grid_avec_ennemi: Grille booléenne incluant obstacles + ennemi
    """
    # Copier la grille statique
    grid_avec_ennemi = grid_expanded_statique.copy()
    
    # Calculer le rayon de sécurité autour de l'ennemi
    R_securite = r_robot + r_ennemi + marge_min
    rayon_case = int(R_securite // case_mm)
    
    # Convertir position ennemi en cases
    x_ennemi_case = int(x_ennemi // case_mm)
    y_ennemi_case = int(y_ennemi // case_mm)
    
    # Ajouter la zone circulaire de l'ennemi
    height, width = grid_avec_ennemi.shape
    
    for dx in range(-rayon_case, rayon_case + 1):
        for dy in range(-rayon_case, rayon_case + 1):
            # Vérifier si dans le cercle
            if dx*dx + dy*dy <= rayon_case*rayon_case:
                x_case = x_ennemi_case + dx
                y_case = y_ennemi_case + dy
                
                # Vérifier limites terrain
                if 0 <= x_case < height and 0 <= y_case < width:
                    grid_avec_ennemi[x_case, y_case] = True
    
    return grid_avec_ennemi


# === CALCUL D'ANGLE ===

def calculer_angle_vers_point(x_actuel, y_actuel, x_cible, y_cible):
    """
    Calcule l'angle nécessaire pour aller du point actuel vers le point cible.
    
    Args:
        x_actuel, y_actuel: Position actuelle en mm
        x_cible, y_cible: Position cible en mm
    
    Returns:
        angle: Angle en degrés (0-360)
    """
    dx = x_cible - x_actuel
    dy = y_cible - y_actuel
    
    # Calculer l'angle en radians puis convertir en degrés
    angle_rad = math.atan2(dy, dx)
    angle_deg = math.degrees(angle_rad)
    
    # Normaliser entre 0 et 360
    if angle_deg < 0:
        angle_deg += 360
    
    return angle_deg


# === DÉTECTION BLOCAGE ENNEMI ===

def detecter_blocage_ennemi(x_robot, y_robot, x_ennemi, y_ennemi, R_securite):
    """
    Détecte si l'ennemi est trop proche du robot (zone critique).
    
    Args:
        x_robot, y_robot: Position du robot en mm
        x_ennemi, y_ennemi: Position de l'ennemi en mm
        R_securite: Rayon de sécurité en mm
    
    Returns:
        Tuple (bloque, distance) : (True/False, distance en mm)
    """
    dx = x_ennemi - x_robot
    dy = y_ennemi - y_robot
    distance = math.sqrt(dx*dx + dy*dy)
    
    # Ennemi dans zone critique (2x le rayon de sécurité)
    bloque = distance < (R_securite * 2)
    
    return bloque, distance


# === FONCTION DE TEST ===

def tester_module():
    """Fonction de test pour vérifier que le module fonctionne"""
    print("=== TEST MODULE calcul_mouv.py ===")
    
    # Créer une grille de test
    width, height = 100, 100
    grid_test = np.zeros((width, height), dtype=bool)
    
    # Ajouter quelques obstacles
    grid_test[40:60, 40:60] = True
    
    # Calculer distance map
    dist_map = distance_transform_edt(~grid_test)
    
    # Initialiser
    initialiser_grille(grid_test, dist_map, width, height, case_mm=10)
    
    # Test A*
    start = (10, 10)
    goal = (80, 80)
    path = astar_safe(start, goal, safety_weight=2.0)
    
    if path:
        print(f"✅ A* fonctionne : Chemin de {len(path)} points")
        
        # Test simplification
        simplified = simplify_path_safe(path, min_clearance=3)
        print(f"✅ Simplification : {len(path)} → {len(simplified)} points")
        
        # Test angle
        angle = calculer_angle_vers_point(100, 100, 200, 200)
        print(f"✅ Calcul angle : {angle:.1f}°")
    else:
        print("❌ A* a échoué")


if __name__ == "__main__":
    # Exécuter le test si le fichier est lancé directement
    tester_module()
