"""
fonction.py - Module de gestion des obstacles et batteries
Version avec classe Obstacles pour gestion dynamique
"""

import numpy as np
from scipy.ndimage import binary_dilation
from dataclasses import dataclass
from typing import List, Tuple, Optional, Set
import math

# ============================================================================
# CLASSE OBSTACLE - Représente un obstacle individuel
# ============================================================================

@dataclass
class Obstacle:
    """
    Représente un obstacle unique sur le terrain.
    
    Attributes:
        nom: Identifiant unique de l'obstacle
        forme: Type d'obstacle ('carre', 'rectangle', 'cercle', 'polygone')
        params: Paramètres spécifiques selon la forme
        actif: Si True, l'obstacle est pris en compte
        priorite: Ordre de traitement (0 = priorité max)
    """
    nom: str
    forme: str
    params: dict
    actif: bool = True
    priorite: int = 0
    
    def __str__(self):
        etat = "✅ ACTIF" if self.actif else "⚪ INACTIF"
        return f"{etat} | {self.nom:15s} | {self.forme:10s}"


# ============================================================================
# CLASSE OBSTACLES - Gestionnaire principal
# ============================================================================

class Obstacles:
    """
    Gestionnaire dynamique des obstacles du terrain.
    
    Permet d'ajouter, retirer, activer/désactiver des obstacles
    et de générer les grilles pour l'algorithme A*.
    
    Exemple d'utilisation:
        >>> obs_manager = Obstacles(3000, 2000, 150, 50, 10)
        >>> obs_manager.ajouter_carre("zone1", (1500, 800), 200)
        >>> obs_manager.ajouter_rectangle("grenier", (600, 1550), (2400, 2000))
        >>> grid = obs_manager.generer_grille()
        >>> obs_manager.desactiver("zone1")
    """
    
    def __init__(self, terrain_w_mm: int, terrain_h_mm: int, 
                 rayon_robot_mm: int, marge_obstacles_mm: int, case_mm: int = 10):
        """
        Initialise le gestionnaire d'obstacles.
        
        Args:
            terrain_w_mm: Largeur du terrain en mm
            terrain_h_mm: Hauteur du terrain en mm
            rayon_robot_mm: Rayon du robot en mm
            marge_obstacles_mm: Marge de sécurité autour des obstacles en mm
            case_mm: Taille d'une case en mm (défaut: 10)
        """
        # Paramètres du terrain
        self.terrain_w_mm = terrain_w_mm
        self.terrain_h_mm = terrain_h_mm
        self.case_mm = case_mm
        self.width = terrain_w_mm // case_mm
        self.height = terrain_h_mm // case_mm
        
        # Paramètres de sécurité
        self.rayon_robot_mm = rayon_robot_mm
        self.marge_obstacles_mm = marge_obstacles_mm
        self.rayon_total_case = (rayon_robot_mm + marge_obstacles_mm) // case_mm
        
        # Dictionnaire des obstacles (nom -> Obstacle)
        self._obstacles: dict[str, Obstacle] = {}
        
        # Cache pour éviter recalculs inutiles
        self._cache_grid = None
        self._cache_valide = False
        
        print(f"🏗️  Gestionnaire obstacles initialisé : {self.width}×{self.height} cases")
    
    
    # ========================================================================
    # AJOUT D'OBSTACLES
    # ========================================================================
    
    def ajouter_carre(self, nom: str, centre_mm: Tuple[int, int], 
                      cote_mm: int, actif: bool = True, priorite: int = 0) -> None:
        """
        Ajoute un obstacle carré.
        
        Args:
            nom: Identifiant unique
            centre_mm: (x, y) centre du carré en mm
            cote_mm: Côté du carré en mm
            actif: Si True, obstacle actif dès création
            priorite: Ordre de traitement
            
        Example:
            >>> obs.ajouter_carre("zone1", (1250, 1450), 200)
        """
        obstacle = Obstacle(
            nom=nom,
            forme="carre",
            params={"centre": centre_mm, "cote": cote_mm},
            actif=actif,
            priorite=priorite
        )
        self._obstacles[nom] = obstacle
        self._invalider_cache()
        print(f"➕ Carré ajouté : {nom} @ {centre_mm} ({cote_mm}mm)")
    
    
    def ajouter_rectangle(self, nom: str, coin_bas_gauche_mm: Tuple[int, int],
                         coin_haut_droit_mm: Tuple[int, int], 
                         actif: bool = True, priorite: int = 0) -> None:
        """
        Ajoute un obstacle rectangulaire.
        
        Args:
            nom: Identifiant unique
            coin_bas_gauche_mm: (x_min, y_min) en mm
            coin_haut_droit_mm: (x_max, y_max) en mm
            actif: Si True, obstacle actif dès création
            priorite: Ordre de traitement
            
        Example:
            >>> obs.ajouter_rectangle("grenier", (600, 1550), (2400, 2000))
        """
        obstacle = Obstacle(
            nom=nom,
            forme="rectangle",
            params={"coin_bg": coin_bas_gauche_mm, "coin_hd": coin_haut_droit_mm},
            actif=actif,
            priorite=priorite
        )
        self._obstacles[nom] = obstacle
        self._invalider_cache()
        print(f"➕ Rectangle ajouté : {nom} @ {coin_bas_gauche_mm} - {coin_haut_droit_mm}")
    
    
    def ajouter_cercle(self, nom: str, centre_mm: Tuple[int, int],
                      rayon_mm: int, actif: bool = True, priorite: int = 0) -> None:
        """
        Ajoute un obstacle circulaire.
        
        Args:
            nom: Identifiant unique
            centre_mm: (x, y) centre du cercle en mm
            rayon_mm: Rayon du cercle en mm
            actif: Si True, obstacle actif dès création
            priorite: Ordre de traitement
            
        Example:
            >>> obs.ajouter_cercle("zone_danger", (1500, 1000), 300)
        """
        obstacle = Obstacle(
            nom=nom,
            forme="cercle",
            params={"centre": centre_mm, "rayon": rayon_mm},
            actif=actif,
            priorite=priorite
        )
        self._obstacles[nom] = obstacle
        self._invalider_cache()
        print(f"➕ Cercle ajouté : {nom} @ {centre_mm} (r={rayon_mm}mm)")
    
    
    def ajouter_polygone(self, nom: str, sommets_mm: List[Tuple[int, int]],
                        actif: bool = True, priorite: int = 0) -> None:
        """
        Ajoute un obstacle polygonal.
        
        Args:
            nom: Identifiant unique
            sommets_mm: Liste de (x, y) des sommets en mm
            actif: Si True, obstacle actif dès création
            priorite: Ordre de traitement
            
        Example:
            >>> obs.ajouter_polygone("zone_tri", [(100,100), (200,100), (150,200)])
        """
        obstacle = Obstacle(
            nom=nom,
            forme="polygone",
            params={"sommets": sommets_mm},
            actif=actif,
            priorite=priorite
        )
        self._obstacles[nom] = obstacle
        self._invalider_cache()
        print(f"➕ Polygone ajouté : {nom} ({len(sommets_mm)} sommets)")
    
    
    # ========================================================================
    # ACTIVATION / DÉSACTIVATION
    # ========================================================================
    
    def activer(self, nom: str) -> bool:
        """
        Active un obstacle.
        
        Args:
            nom: Nom de l'obstacle
            
        Returns:
            True si succès, False si obstacle inexistant
        """
        if nom not in self._obstacles:
            print(f"❌ Obstacle '{nom}' introuvable")
            return False
        
        self._obstacles[nom].actif = True
        self._invalider_cache()
        print(f"✅ Obstacle '{nom}' activé")
        return True
    
    
    def desactiver(self, nom: str) -> bool:
        """
        Désactive un obstacle (ne sera plus pris en compte).
        
        Args:
            nom: Nom de l'obstacle
            
        Returns:
            True si succès, False si obstacle inexistant
        """
        if nom not in self._obstacles:
            print(f"❌ Obstacle '{nom}' introuvable")
            return False
        
        self._obstacles[nom].actif = False
        self._invalider_cache()
        print(f"🔴 Obstacle '{nom}' désactivé")
        return True
    
    
    def basculer(self, nom: str) -> bool:
        """
        Inverse l'état actif/inactif d'un obstacle.
        
        Args:
            nom: Nom de l'obstacle
            
        Returns:
            True si succès, False si obstacle inexistant
        """
        if nom not in self._obstacles:
            print(f"❌ Obstacle '{nom}' introuvable")
            return False
        
        self._obstacles[nom].actif = not self._obstacles[nom].actif
        etat = "activé" if self._obstacles[nom].actif else "désactivé"
        self._invalider_cache()
        print(f"🔄 Obstacle '{nom}' {etat}")
        return True
    
    
    # ========================================================================
    # SUPPRESSION
    # ========================================================================
    
    def retirer(self, nom: str) -> bool:
        """
        Retire définitivement un obstacle.
        
        Args:
            nom: Nom de l'obstacle
            
        Returns:
            True si succès, False si obstacle inexistant
        """
        if nom not in self._obstacles:
            print(f"❌ Obstacle '{nom}' introuvable")
            return False
        
        del self._obstacles[nom]
        self._invalider_cache()
        print(f"🗑️  Obstacle '{nom}' retiré")
        return True
    
    
    def retirer_tous(self) -> None:
        """Retire tous les obstacles."""
        count = len(self._obstacles)
        self._obstacles.clear()
        self._invalider_cache()
        print(f"🗑️  {count} obstacles retirés")
    
    
    # ========================================================================
    # CONSULTATION
    # ========================================================================
    
    def lister(self, filtre_actif: Optional[bool] = None) -> None:
        """
        Affiche la liste des obstacles.
        
        Args:
            filtre_actif: Si True, seulement actifs. Si False, seulement inactifs.
                         Si None, tous les obstacles.
        """
        print("\n📋 LISTE DES OBSTACLES")
        print("=" * 70)
        
        obstacles_tries = sorted(self._obstacles.values(), 
                                key=lambda o: (o.priorite, o.nom))
        
        count = 0
        for obs in obstacles_tries:
            if filtre_actif is None or obs.actif == filtre_actif:
                print(f"  {obs}")
                count += 1
        
        if count == 0:
            print("  (aucun obstacle)")
        
        print("=" * 70)
        print(f"Total : {len(self._obstacles)} obstacles ({count} affichés)\n")
    
    
    def existe(self, nom: str) -> bool:
        """Vérifie si un obstacle existe."""
        return nom in self._obstacles
    
    
    def est_actif(self, nom: str) -> Optional[bool]:
        """
        Vérifie si un obstacle est actif.
        
        Returns:
            True si actif, False si inactif, None si inexistant
        """
        if nom not in self._obstacles:
            return None
        return self._obstacles[nom].actif
    
    
    def compter(self, actifs_seulement: bool = False) -> int:
        """
        Compte les obstacles.
        
        Args:
            actifs_seulement: Si True, compte seulement les actifs
            
        Returns:
            Nombre d'obstacles
        """
        if actifs_seulement:
            return sum(1 for obs in self._obstacles.values() if obs.actif)
        return len(self._obstacles)
    
    
    # ========================================================================
    # GÉNÉRATION DE GRILLE
    # ========================================================================
    
    def generer_grille(self, utiliser_cache: bool = True) -> Tuple[np.ndarray, np.ndarray, 
                                                                      np.ndarray, np.ndarray]:
        """
        Génère les grilles d'obstacles pour l'algorithme A*.
        
        Args:
            utiliser_cache: Si True et cache valide, retourne le cache
            
        Returns:
            tuple: (grid, grid_expanded, obstacle_array, expanded_array)
                - grid: Grille booléenne des obstacles
                - grid_expanded: Grille avec marges de sécurité
                - obstacle_array: Array numpy des positions d'obstacles
                - expanded_array: Array numpy des zones de sécurité
        """
        # Utiliser cache si valide
        if utiliser_cache and self._cache_valide and self._cache_grid is not None:
            return self._cache_grid
        
        # Créer ensemble de cases occupées
        obstacles_cases: Set[Tuple[int, int]] = set()
        
        # Trier par priorité
        obstacles_tries = sorted(
            [obs for obs in self._obstacles.values() if obs.actif],
            key=lambda o: o.priorite
        )
        
        # Générer cases pour chaque obstacle
        for obs in obstacles_tries:
            if obs.forme == "carre":
                cases = self._generer_carre(obs.params)
            elif obs.forme == "rectangle":
                cases = self._generer_rectangle(obs.params)
            elif obs.forme == "cercle":
                cases = self._generer_cercle(obs.params)
            elif obs.forme == "polygone":
                cases = self._generer_polygone(obs.params)
            else:
                print(f"⚠️  Forme inconnue : {obs.forme}")
                continue
            
            obstacles_cases.update(cases)
        
        # Conversion en grille
        grid = np.zeros((self.width, self.height), dtype=bool)
        for (x, y) in obstacles_cases:
            grid[x, y] = True
        
        # Expansion avec marge de sécurité
        structure = np.zeros((2*self.rayon_total_case+1, 2*self.rayon_total_case+1))
        y_grid, x_grid = np.ogrid[-self.rayon_total_case:self.rayon_total_case+1, 
                                   -self.rayon_total_case:self.rayon_total_case+1]
        mask = x_grid*x_grid + y_grid*y_grid <= self.rayon_total_case*self.rayon_total_case
        structure[mask] = 1
        grid_expanded = binary_dilation(grid, structure=structure)
        
        # Conversion en arrays pour affichage
        obstacle_array = np.argwhere(grid)
        expanded_array = np.argwhere(grid_expanded & ~grid)
        
        # Mise en cache
        self._cache_grid = (grid, grid_expanded, obstacle_array, expanded_array)
        self._cache_valide = True
        
        print(f"🗺️  Grille générée : {len(obstacles_cases)} cases obstacles, "
              f"{len(expanded_array)} cases sécurité")
        
        return grid, grid_expanded, obstacle_array, expanded_array
    
    
    # ========================================================================
    # GÉNÉRATEURS DE FORMES (méthodes privées)
    # ========================================================================
    
    def _generer_carre(self, params: dict) -> Set[Tuple[int, int]]:
        """Génère les cases d'un carré."""
        cx_mm, cy_mm = params["centre"]
        cote_mm = params["cote"]
        
        cx = cx_mm // self.case_mm
        cy = cy_mm // self.case_mm
        demi_cote = (cote_mm // self.case_mm) // 2
        
        cases = set()
        for x in range(cx - demi_cote, cx + demi_cote):
            for y in range(cy - demi_cote, cy + demi_cote):
                if 0 <= x < self.width and 0 <= y < self.height:
                    cases.add((x, y))
        
        return cases
    
    
    def _generer_rectangle(self, params: dict) -> Set[Tuple[int, int]]:
        """Génère les cases d'un rectangle."""
        x_min_mm, y_min_mm = params["coin_bg"]
        x_max_mm, y_max_mm = params["coin_hd"]
        
        x_min = x_min_mm // self.case_mm
        x_max = x_max_mm // self.case_mm
        y_min = y_min_mm // self.case_mm
        y_max = y_max_mm // self.case_mm
        
        cases = set()
        for x in range(x_min, x_max):
            for y in range(y_min, y_max):
                if 0 <= x < self.width and 0 <= y < self.height:
                    cases.add((x, y))
        
        return cases
    
    
    def _generer_cercle(self, params: dict) -> Set[Tuple[int, int]]:
        """Génère les cases d'un cercle."""
        cx_mm, cy_mm = params["centre"]
        rayon_mm = params["rayon"]
        
        cx = cx_mm // self.case_mm
        cy = cy_mm // self.case_mm
        rayon = rayon_mm // self.case_mm
        
        cases = set()
        for dx in range(-rayon, rayon + 1):
            for dy in range(-rayon, rayon + 1):
                if dx*dx + dy*dy <= rayon*rayon:
                    x = cx + dx
                    y = cy + dy
                    if 0 <= x < self.width and 0 <= y < self.height:
                        cases.add((x, y))
        
        return cases
    
    
    def _generer_polygone(self, params: dict) -> Set[Tuple[int, int]]:
        """
        Génère les cases d'un polygone.
        Utilise l'algorithme de scan-line.
        """
        sommets_mm = params["sommets"]
        if len(sommets_mm) < 3:
            return set()
        
        # Convertir en cases
        sommets = [(x // self.case_mm, y // self.case_mm) for x, y in sommets_mm]
        
        # Trouver bounding box
        x_min = min(x for x, y in sommets)
        x_max = max(x for x, y in sommets)
        y_min = min(y for x, y in sommets)
        y_max = max(y for x, y in sommets)
        
        cases = set()
        
        # Point-in-polygon test pour chaque case
        for x in range(max(0, x_min), min(self.width, x_max + 1)):
            for y in range(max(0, y_min), min(self.height, y_max + 1)):
                if self._point_dans_polygone((x, y), sommets):
                    cases.add((x, y))
        
        return cases
    
    
    def _point_dans_polygone(self, point: Tuple[int, int], 
                            sommets: List[Tuple[int, int]]) -> bool:
        """
        Test si un point est dans un polygone (ray casting algorithm).
        """
        x, y = point
        n = len(sommets)
        inside = False
        
        p1x, p1y = sommets[0]
        for i in range(1, n + 1):
            p2x, p2y = sommets[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
    
    
    def _invalider_cache(self) -> None:
        """Invalide le cache de grille."""
        self._cache_valide = False


# ============================================================================
# FONCTION COMPATIBLE AVEC ANCIEN CODE
# ============================================================================

def creer_obstacles(terrain_w_mm, terrain_h_mm, 
                    rayon_robot_mm, marge_obstacles_mm, case_mm):
    """
    Fonction de compatibilité avec l'ancien code.
    Crée les obstacles par défaut de la Coupe de France de Robotique.
    
    ⚠️  DEPRECATED : Utilisez plutôt la classe Obstacles
    
    Returns:
        tuple: (grid, grid_expanded, obstacle_array, expanded_array)
    """
    # Créer gestionnaire
    obs_manager = Obstacles(terrain_w_mm, terrain_h_mm, 
                           rayon_robot_mm, marge_obstacles_mm, case_mm)
    
    # Ajouter obstacles par défaut (Coupe de France 2024/2025)
    zones_centres = [
        (1250, 1450), (1750, 1450),
        (100, 800), (800, 800), (1500, 800), (2200, 800), (2900, 800),
        (700, 100), (1500, 100), (2300, 100),
    ]
    
    for i, (x, y) in enumerate(zones_centres, 1):
        obs_manager.ajouter_carre(f"zone{i}", (x, y), 200, actif=True)

    zones_noisettes = [
        [(100,1100),(250,1300)],
         [(100,300),(250,500)],
         [(2750,1100),(2900,1300)],
         [(2750,300),(2900,500)],
         [(1050,725),(1250,875)],
         [(1750,725),(1950,875)],
         [(1000,100),(1200,250)],
         [(1800,100),(2000,250)]
    ]
    i = 1
    for noisettes in zones_noisettes :
        x1 = noisettes[0][0]
        y1 = noisettes[0][1]
        x2 = noisettes[1][0]
        y2 = noisettes[1][1]
        obs_manager.ajouter_rectangle(f"Noisette{i}", (x1, y1), (x2, y2), actif=True)
        i+=1
    # Zone rectangulaire (grenier)
    obs_manager.ajouter_rectangle("grenier", (600, 1550), (2400, 2000), actif=True)
    
    return obs_manager.generer_grille()


# ============================================================================
# FONCTIONS POUR LA ZONE DE SÉCURITÉ DYNAMIQUE DE L'ENNEMI
# ============================================================================

def creer_zone_securite_ennemi(x_ennemi, y_ennemi, r_robot, r_ennemi, 
                                marge_securite=50, case_mm=10,
                                terrain_w_mm=3000, terrain_h_mm=2000):
    """
    Crée une zone de sécurité circulaire autour de la position de l'ennemi.
    Cette zone est considérée comme un obstacle temporaire.
    
    Args:
        x_ennemi: Position x de l'ennemi en mm
        y_ennemi: Position y de l'ennemi en mm
        r_robot: Rayon du robot en mm
        r_ennemi: Rayon de l'ennemi en mm
        marge_securite: Marge de sécurité supplémentaire en mm (défaut: 50mm)
        case_mm: Taille d'une case en mm
        terrain_w_mm: Largeur du terrain en mm
        terrain_h_mm: Hauteur du terrain en mm
    
    Returns:
        tuple: (zone_ennemi_array, R_securite)
            - zone_ennemi_array: Array numpy des coordonnées (x,y) en mm de la zone
            - R_securite: Rayon total de la zone de sécurité en mm
    """
    # Calcul du rayon de sécurité total
    R_securite = r_robot + r_ennemi + marge_securite
    
    # Conversion en cases
    width, height = terrain_w_mm // case_mm, terrain_h_mm // case_mm
    x_ennemi_case = int(x_ennemi // case_mm)
    y_ennemi_case = int(y_ennemi // case_mm)
    rayon_case = int(R_securite // case_mm)
    
    # Création de la zone circulaire
    zone_points = []
    
    for dx in range(-rayon_case, rayon_case + 1):
        for dy in range(-rayon_case, rayon_case + 1):
            # Vérifier si le point est dans le cercle
            if dx*dx + dy*dy <= rayon_case*rayon_case:
                x_case = x_ennemi_case + dx
                y_case = y_ennemi_case + dy
                
                # Vérifier que le point est dans les limites du terrain
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
        U_last: Liste des dernières tensions [U1, U2, U3]
    
    Returns:
        Batteries: Liste mise à jour avec les nouveaux pourcentages
    """
    for i in range(len(Batteries)):
        if Batteries[i][2] != U_last[i]:
            print(f"MAJ Batterie N°{i+1}")
            Batteries[i][3] = 100 * (Batteries[i][2] - Batteries[i][0]) / (Batteries[i][1] - Batteries[i][0])
            Batteries[i][3] = round(Batteries[i][3], 2)
    
    return Batteries


def gerer_basculement_batteries(Batteries, U_last, Ordre_Batteries, seuil_critique=5.0):
    """
    Gère le basculement automatique entre batteries quand l'une est déchargée.
    
    Args:
        Batteries: Liste des états des batteries
        U_last: Liste des dernières tensions
        Ordre_Batteries: Liste [1,0,0] ou [0,1,0] ou [0,0,1]
        seuil_critique: Seuil de pourcentage pour basculer (défaut: 5.0%)
    
    Returns:
        Ordre_Batteries: Ordre mis à jour
    """
    if Batteries[0][2] != U_last[0] and Batteries[0][3] <= seuil_critique:
        print("Utilisation Bat2")
        Ordre_Batteries = [0, 1, 0]
    
    if Batteries[1][2] != U_last[1] and Batteries[1][3] <= seuil_critique:
        print("Utilisation Bat3")
        Ordre_Batteries = [0, 0, 1]
    
    if Batteries[2][2] != U_last[2] and Batteries[2][3] <= seuil_critique:
        print("Batteries déchargées")
    
    return Ordre_Batteries


# ============================================================================
# FONCTION DE TEST
# ============================================================================

def tester_classe_obstacles():
    """Fonction de test pour la classe Obstacles."""
    print("\n" + "="*70)
    print("TEST DE LA CLASSE OBSTACLES")
    print("="*70 + "\n")
    
    # Créer gestionnaire
    obs = Obstacles(3000, 2000, 150, 50, 10)
    
    # Ajouter différents types d'obstacles
    print("\n--- AJOUT D'OBSTACLES ---")
    obs.ajouter_carre("zone1", (1250, 1450), 200)
    obs.ajouter_carre("zone2", (1750, 1450), 200, actif=False)
    obs.ajouter_rectangle("grenier", (600, 1550), (2400, 2000))
    obs.ajouter_cercle("danger", (1500, 1000), 300)
    obs.ajouter_polygone("triangle", [(100, 100), (300, 100), (200, 300)])
    
    # Lister
    print("\n--- LISTE COMPLÈTE ---")
    obs.lister()
    
    # Activer/Désactiver
    print("\n--- ACTIVATION/DÉSACTIVATION ---")
    obs.desactiver("zone1")
    obs.activer("zone2")
    obs.basculer("grenier")
    
    print("\n--- OBSTACLES ACTIFS ---")
    obs.lister(filtre_actif=True)
    
    # Générer grille
    print("\n--- GÉNÉRATION GRILLE ---")
    grid, grid_expanded, obstacle_array, expanded_array = obs.generer_grille()
    print(f"✅ Grille : {grid.shape}, {np.sum(grid)} cases obstacles")
    
    # Retirer
    print("\n--- SUPPRESSION ---")
    obs.retirer("danger")
    obs.lister()


"""if __name__ == "__main__":
    # Exécuter le test si le fichier est lancé directement
    tester_classe_obstacles()"""

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def calcul_angle_vers_point(x_actuel, y_actuel, x_suivant, y_suivant):
    """
    Calcule l'angle absolu (en degrés 0–360) à viser pour aller vers (x_suivant, y_suivant)
    depuis (x_actuel, y_actuel).
    """
    dx = x_suivant - x_actuel
    dy = y_suivant - y_actuel
    angle = (math.degrees(math.atan2(dy, dx)) + 360) % 360
    return round(angle, 2)


def distance_euclidienne(x1, y1, x2, y2):
    """Calcule la distance euclidienne entre deux points"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

