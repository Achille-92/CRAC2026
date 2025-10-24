import numpy as np
from scipy.ndimage import binary_dilation

# ============================================================================
# FONCTIONS POUR LES OBSTACLES STATIQUES
# ============================================================================

def creer_obstacles(terrain_w_mm, terrain_h_mm, 
                    rayon_robot_mm, marge_obstacles_mm, case_mm):
    """
    Cree les obstacles statiques de la piste avec marge de securite.
    
    Args:
        terrain_w_mm: Largeur du terrain en mm
        terrain_h_mm: Hauteur du terrain en mm
        rayon_robot_mm: Rayon du robot en mm
        marge_obstacles_mm: Marge de securite autour des obstacles en mm
        case_mm: Taille d'une case en mm
    
    Returns:
        tuple: (grid, grid_expanded, obstacle_array, expanded_array)
    """
    width, height = terrain_w_mm // case_mm, terrain_h_mm // case_mm
    rayon_total_case = (rayon_robot_mm + marge_obstacles_mm) // case_mm
    
    # Definition des zones interdites
    zones_centres_mm = [
        (1250, 1450), (1750, 1450),
        (100, 800), (800, 800), (1500, 800), (2200, 800), (2900, 800),
        (700, 100), (1500, 100), (2300, 100),
    ]
    zones_cote_mm = 200
    zones_cote_case = zones_cote_mm // case_mm // 2
    
    # Creation des obstacles
    obstacles = set()
    
    # 1. Zones carrees
    for (cx_mm, cy_mm) in zones_centres_mm:
        cx, cy = cx_mm // case_mm, cy_mm // case_mm
        for x in range(cx - zones_cote_case, cx + zones_cote_case):
            for y in range(cy - zones_cote_case, cy + zones_cote_case):
                if 0 <= x < width and 0 <= y < height:
                    obstacles.add((x, y))
    
    # 2. Zone rectangulaire
    x_start_case, x_end_case = 60, 240
    y_start_case, y_end_case = 155, 200
    for x in range(x_start_case, x_end_case):
        for y in range(y_start_case, y_end_case):
            obstacles.add((x, y))
    
    # Conversion en grille
    grid = np.zeros((width, height), dtype=bool)
    for (x, y) in obstacles:
        grid[x, y] = True
    
    # Expansion des obstacles avec marge de securite
    structure = np.zeros((2*rayon_total_case+1, 2*rayon_total_case+1))
    y_grid, x_grid = np.ogrid[-rayon_total_case:rayon_total_case+1, 
                               -rayon_total_case:rayon_total_case+1]
    mask = x_grid*x_grid + y_grid*y_grid <= rayon_total_case*rayon_total_case
    structure[mask] = 1
    grid_expanded = binary_dilation(grid, structure=structure)
    
    # Conversion en arrays pour l'affichage
    obstacle_array = np.argwhere(grid)
    expanded_array = np.argwhere(grid_expanded & ~grid)
    
    return grid, grid_expanded, obstacle_array, expanded_array

# ============================================================================
# FONCTIONS POUR LA ZONE DE SECURITE DYNAMIQUE DE L'ENNEMI
# ============================================================================

def creer_zone_securite_ennemi(x_ennemi, y_ennemi, r_robot, r_ennemi, 
                                marge_securite=50, case_mm=10,
                                terrain_w_mm=3000, terrain_h_mm=2000):
    """
    Cree une zone de securite circulaire autour de la position de l'ennemi.
    Cette zone est consideree comme un obstacle temporaire.
    
    Args:
        x_ennemi: Position x de l'ennemi en mm
        y_ennemi: Position y de l'ennemi en mm
        r_robot: Rayon du robot en mm
        r_ennemi: Rayon de l'ennemi en mm
        marge_securite: Marge de securite supplementaire en mm (defaut: 50mm)
        case_mm: Taille d'une case en mm
        terrain_w_mm: Largeur du terrain en mm
        terrain_h_mm: Hauteur du terrain en mm
    
    Returns:
        tuple: (zone_ennemi_array, R_securite)
            - zone_ennemi_array: Array numpy des coordonnees (x,y) en mm de la zone
            - R_securite: Rayon total de la zone de securite en mm
    """
    # Calcul du rayon de securite total
    R_securite = r_robot + r_ennemi + marge_securite
    
    # Conversion en cases
    width, height = terrain_w_mm // case_mm, terrain_h_mm // case_mm
    x_ennemi_case = int(x_ennemi // case_mm)
    y_ennemi_case = int(y_ennemi // case_mm)
    rayon_case = int(R_securite // case_mm)
    
    # Creation de la zone circulaire
    zone_points = []
    
    for dx in range(-rayon_case, rayon_case + 1):
        for dy in range(-rayon_case, rayon_case + 1):
            # Verifier si le point est dans le cercle
            if dx*dx + dy*dy <= rayon_case*rayon_case:
                x_case = x_ennemi_case + dx
                y_case = y_ennemi_case + dy
                
                # Verifier que le point est dans les limites du terrain
                if 0 <= x_case < width and 0 <= y_case < height:
                    # Convertir en mm
                    x_mm = x_case * case_mm
                    y_mm = y_case * case_mm
                    zone_points.append([x_mm, y_mm])
    
    zone_ennemi_array = np.array(zone_points) if zone_points else np.array([]).reshape(0, 2)
    
    return zone_ennemi_array, R_securite



# ============================================================================
# FONCTIONS POUR LES BATTERIES
# ============================================================================

def calculer_pourcentage_batteries(Batteries, U_last):
    """
    Calcule le pourcentage de charge de chaque batterie.
    
    Args:
        Batteries: Liste de 3 batteries [[Vmin, Vmax, Vactuel, %], ...]
        U_last: Liste des dernieres tensions [U1, U2, U3]
    
    Returns:
        Batteries: Liste mise a jour avec les nouveaux pourcentages
    """
    for i in range(len(Batteries)):
        if Batteries[i][2] != U_last[i]:
            print(f"MAJ Batterie N°{i+1}")
            Batteries[i][3] = 100 * (Batteries[i][2] - Batteries[i][0]) / (Batteries[i][1] - Batteries[i][0])
            Batteries[i][3] = round(Batteries[i][3], 2)
    
    return Batteries


def gerer_basculement_batteries(Batteries, U_last, Ordre_Batteries, seuil_critique=5.0):
    """
    Gere le basculement automatique entre batteries quand l'une est dechargee.
    
    Args:
        Batteries: Liste des etats des batteries
        U_last: Liste des dernieres tensions
        Ordre_Batteries: Liste [1,0,0] ou [0,1,0] ou [0,0,1]
        seuil_critique: Seuil de pourcentage pour basculer (defaut: 5.0%)
    
    Returns:
        Ordre_Batteries: Ordre mis a jour
    """
    if Batteries[0][2] != U_last[0] and Batteries[0][3] <= seuil_critique:
        print("Utilisation Bat2")
        Ordre_Batteries = [0, 1, 0]
    
    if Batteries[1][2] != U_last[1] and Batteries[1][3] <= seuil_critique:
        print("Utilisation Bat3")
        Ordre_Batteries = [0, 0, 1]
    
    if Batteries[2][2] != U_last[2] and Batteries[2][3] <= seuil_critique:
        print("Batteries dechargees")
    
    return Ordre_Batteries
