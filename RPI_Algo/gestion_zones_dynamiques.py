from affichage import afficher_obstacles
from calcul_mouv import initialiser_grille, verifier_cible_disponible
from fonction import distance_euclidienne
"""
gestion_zones_dynamiques.py
Module pour gérer dynamiquement les zones interdites selon l'état du jeu
"""

def mettre_a_jour_zones_dynamiques(obs_manager, obs_manager_noisettes,
                                    Liste_gardemanger_libres, Liste_gardemanger_occuper,
                                    Liste_noisettes_libres, Liste_noisettes_prises):
    """
    Met à jour l'état actif/inactif des zones et noisettes selon les listes.
    
    RÈGLES :
    - Zones (gardemanger) : ACTIVES si dans Liste_gardemanger_occuper
                           INACTIVES si dans Liste_gardemanger_libres
    - Noisettes : ACTIVES si dans Liste_noisettes_libres (obstacles à éviter)
                  INACTIVES si dans Liste_noisettes_prises (déjà prises, libres)
    
    Args:
        obs_manager: Gestionnaire Obstacles pour les zones carrées
        obs_manager_noisettes: Gestionnaire Obstacles pour les noisettes
        Liste_gardemanger_libres: Liste des numéros de zones libres [1,2,3...]
        Liste_gardemanger_occuper: Liste des numéros de zones occupées [...]
        Liste_noisettes_libres: Liste des numéros de noisettes à éviter [1,2,3...]
        Liste_noisettes_prises: Liste des numéros de noisettes prises [...]
    
    Returns:
        tuple: (grilles_modifiees, rapport_zones, rapport_noisettes)
            - grilles_modifiees: True si au moins une zone a changé d'état
            - rapport_zones: Dict avec le statut de chaque zone
            - rapport_noisettes: Dict avec le statut de chaque noisette
    """
    grilles_modifiees = False
    rapport_zones = {}
    rapport_noisettes = {}
    
    
    # ========================================================================
    # GESTION DES ZONES (GARDEMANGER) - numérotées de 1 à 10
    # ========================================================================
    
    for numero_zone in range(1, 11):  # Zones 1 à 10
        nom_zone = f"zone{numero_zone}"
        
        # Déterminer l'état souhaité
        if numero_zone in Liste_gardemanger_occuper:
            etat_souhaite = True  # Zone occupée = obstacle ACTIF
            raison = "occupée"
        elif numero_zone in Liste_gardemanger_libres:
            etat_souhaite = False  # Zone libre = obstacle INACTIF
            raison = "libre"
        else:
            # Ne devrait pas arriver, mais au cas où
            etat_souhaite = False
            raison = "indéfinie"
        
        # Vérifier si la zone existe
        if nom_zone in obs_manager._obstacles:
            etat_actuel = obs_manager._obstacles[nom_zone].actif
            
            # Mettre à jour si nécessaire
            if etat_actuel != etat_souhaite:
                if etat_souhaite:
                    obs_manager.activer(nom_zone)
                else:
                    obs_manager.desactiver(nom_zone)
                grilles_modifiees = True
            
            rapport_zones[numero_zone] = {
                "actif": etat_souhaite,
                "raison": raison,
                "modifie": etat_actuel != etat_souhaite
            }
        else:
            print(f"  ⚠️  Zone {numero_zone} n'existe pas dans obs_manager")
    
    # ========================================================================
    # GESTION DES NOISETTES - numérotées de 1 à 8
    # ========================================================================
    
    for numero_noisette in range(1, 9):  # Noisettes 1 à 8
        nom_noisette = f"Noisette{numero_noisette}"
        
        # Déterminer l'état souhaité
        if numero_noisette in Liste_noisettes_libres:
            etat_souhaite = True  # Noisette libre = obstacle ACTIF (à éviter)
            raison = "à éviter"
        elif numero_noisette in Liste_noisettes_prises:
            etat_souhaite = False  # Noisette prise = obstacle INACTIF (libre)
            raison = "prise (libre)"
        else:
            # Ne devrait pas arriver
            etat_souhaite = True  # Par défaut, on évite
            raison = "indéfinie"
        
        # Vérifier si la noisette existe
        if nom_noisette in obs_manager_noisettes._obstacles:
            etat_actuel = obs_manager_noisettes._obstacles[nom_noisette].actif
            
            # Mettre à jour si nécessaire
            if etat_actuel != etat_souhaite:
                if etat_souhaite:
                    obs_manager_noisettes.activer(nom_noisette)
                else:
                    obs_manager_noisettes.desactiver(nom_noisette)
                grilles_modifiees = True
    
            rapport_noisettes[numero_noisette] = {
                "actif": etat_souhaite,
                "raison": raison,
                "modifie": etat_actuel != etat_souhaite
            }
        else:
            print(f"  ⚠️  Noisette {numero_noisette} n'existe pas dans obs_manager_noisettes")
    
    # ========================================================================
    # RÉSUMÉ
    # ========================================================================$
    if grilles_modifiees:
        print("⚠️  GRILLES MODIFIÉES : Régénération nécessaire")
    else:
        print("✓  Aucune modification - Grilles inchangées")
    
    return grilles_modifiees, rapport_zones, rapport_noisettes


def regenerer_grilles_combinees(obs_manager, obs_manager_noisettes):
    """
    Régénère et combine les grilles après modification des zones.
    
    Args:
        obs_manager: Gestionnaire des zones standard
        obs_manager_noisettes: Gestionnaire des noisettes
    
    Returns:
        tuple: (grid, grid_expanded, obstacle_array, expanded_array)
    """
    import numpy as np
    
    print("🔧 Régénération des grilles combinées...")
    
    # Générer les grilles séparément
    grid_zones, grid_zones_expanded, obstacle_array_zones, expanded_array_zones = \
        obs_manager.generer_grille()
    
    grid_noisettes, grid_noisettes_expanded, obstacle_array_noisettes, expanded_array_noisettes = \
        obs_manager_noisettes.generer_grille()
    
    # Combiner les deux grilles (union logique OR)
    grid = np.logical_or(grid_zones, grid_noisettes).astype(int)
    grid_expanded = np.logical_or(grid_zones_expanded, grid_noisettes_expanded).astype(int)
    
    # Combiner les arrays d'obstacles pour l'affichage
    if len(obstacle_array_zones) > 0 and len(obstacle_array_noisettes) > 0:
        obstacle_array = np.vstack([obstacle_array_zones, obstacle_array_noisettes])
    elif len(obstacle_array_zones) > 0:
        obstacle_array = obstacle_array_zones
    else:
        obstacle_array = obstacle_array_noisettes
    
    if len(expanded_array_zones) > 0 and len(expanded_array_noisettes) > 0:
        expanded_array = np.vstack([expanded_array_zones, expanded_array_noisettes])
    elif len(expanded_array_zones) > 0:
        expanded_array = expanded_array_zones
    else:
        expanded_array = expanded_array_noisettes
    
    print(f"✓ Grille combinée : {np.sum(grid)} cases d'obstacles, "
          f"{np.sum(grid_expanded)} cases avec marge")
    
    return grid, grid_expanded, obstacle_array, expanded_array


# ============================================================================
# EXEMPLE D'UTILISATION
# ============================================================================

def exemple_utilisation():
    """
    Exemple montrant comment utiliser les fonctions de gestion dynamique.
    """
    print("\n" + "="*70)
    print("EXEMPLE D'UTILISATION - GESTION DYNAMIQUE DES ZONES")
    print("="*70 + "\n")
    
    # Simuler l'import de la classe Obstacles
    from fonction import Obstacles
    import numpy as np
    
    # Créer les gestionnaires (comme dans main.py)
    obs_manager = Obstacles(3000, 2000, 130, 50, 10)
    obs_manager_noisettes = Obstacles(3000, 2000, 130, 20, 10)
    
    # Ajouter les zones (simplifié pour l'exemple)
    for i in range(1, 11):
        obs_manager.ajouter_carre(f"zone{i}", (1000+i*100, 800), 200, actif=False)
    
    for i in range(1, 9):
        obs_manager_noisettes.ajouter_rectangle(
            f"Noisette{i}", 
            (100+i*200, 100), (200+i*200, 200), 
            actif=True
        )
    
    # --- SCÉNARIO 1 : Début du jeu ---
    print("\n🎮 SCÉNARIO 1 : DÉBUT DU JEU")
    Liste_gardemanger_libres = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
    Liste_gardemanger_occuper = []
    Liste_noisettes_libres = [1, 2, 3, 4, 5, 6, 7, 8]
    Liste_noisettes_prises = []
    
    modifie, rapport_z, rapport_n = mettre_a_jour_zones_dynamiques(
        obs_manager, obs_manager_noisettes,
        Liste_gardemanger_libres, Liste_gardemanger_occuper,
        Liste_noisettes_libres, Liste_noisettes_prises
    )
    
    if modifie:
        grid, grid_exp, obs_arr, exp_arr = regenerer_grilles_combinees(
            obs_manager, obs_manager_noisettes
        )
    
    # --- SCÉNARIO 2 : Prise de noisettes ---
    print("\n🎮 SCÉNARIO 2 : PRISE DE 3 NOISETTES")
    Liste_noisettes_libres = [4, 5, 6, 7, 8]  # Noisettes 1,2,3 retirées
    Liste_noisettes_prises = [1, 2, 3]
    
    modifie, rapport_z, rapport_n = mettre_a_jour_zones_dynamiques(
        obs_manager, obs_manager_noisettes,
        Liste_gardemanger_libres, Liste_gardemanger_occuper,
        Liste_noisettes_libres, Liste_noisettes_prises
    )
    
    if modifie:
        grid, grid_exp, obs_arr, exp_arr = regenerer_grilles_combinees(
            obs_manager, obs_manager_noisettes
        )
    
    # --- SCÉNARIO 3 : Occupation de zones ---
    print("\n🎮 SCÉNARIO 3 : OCCUPATION DE 2 ZONES")
    Liste_gardemanger_libres = [3, 4, 5, 6, 7, 8, 9, 10]  # Zones 1,2 retirées
    Liste_gardemanger_occuper = [1, 2]
    
    modifie, rapport_z, rapport_n = mettre_a_jour_zones_dynamiques(
        obs_manager, obs_manager_noisettes,
        Liste_gardemanger_libres, Liste_gardemanger_occuper,
        Liste_noisettes_libres, Liste_noisettes_prises
    )
    
    if modifie:
        grid, grid_exp, obs_arr, exp_arr = regenerer_grilles_combinees(
            obs_manager, obs_manager_noisettes
        )
    
    print("\n✅ EXEMPLE TERMINÉ\n")


"""if __name__ == "__main__":
    exemple_utilisation()"""

def actualiser_zones_jeu(grid, grid_expanded, obstacle_array, expanded_array, obs_manager, obs_manager_noisettes, Liste_GM_libres, Liste_GM_occuper, Liste_noisettes_libres, Liste_noisettes_prises, obstacle_scatter, expanded_scatter, distance_map, ax, width, height, case_mm):
    """
    Met à jour l'état des zones et régénère les grilles si nécessaire.
    Version ultra-simple : ne touche PAS aux boutons.
    """

    # Mettre à jour les états selon les listes
    grilles_modifiees, _, _ = mettre_a_jour_zones_dynamiques(
        obs_manager, obs_manager_noisettes,
        Liste_GM_libres, Liste_GM_occuper,
        Liste_noisettes_libres, Liste_noisettes_prises
    )
    
    # Régénérer si nécessaire
    if grilles_modifiees:
        print("🔄 Régénération des grilles...")
        
        grid, grid_expanded, obstacle_array, expanded_array = \
            regenerer_grilles_combinees(obs_manager, obs_manager_noisettes)
        
        # Recalculer distance map
        from scipy.ndimage import distance_transform_edt
        distance_map = distance_transform_edt(~grid_expanded)
        initialiser_grille(grid_expanded, distance_map, width, height, case_mm)
        
        # Mettre à jour affichage
        if obstacle_scatter is not None:
            obstacle_scatter.remove()
            obstacle_scatter = None
        if expanded_scatter is not None:
            expanded_scatter.remove()
            expanded_scatter = None
        
        obstacle_scatter, expanded_scatter = afficher_obstacles(
            ax, obstacle_array, expanded_array, case_mm,
            show_expanded=True, show_obstacles=False
        )
        
        print("✅ Grilles mises à jour")
    return grid, grid_expanded, obstacle_array, expanded_array, obs_manager, obs_manager_noisettes, Liste_GM_libres, Liste_GM_occuper, Liste_noisettes_libres, Liste_noisettes_prises, obstacle_scatter, expanded_scatter, distance_map, ax, width, height, case_mm
 


def verifier_et_changer_cible_si_necessaire(action_en_cours,
                                             Liste_actions,
                                             Liste_noisettes_libres,
                                             Liste_GM_libres,
                                             x_robot_actuel, y_robot_actuel,
                                             x_ennemi, y_ennemi,
                                             robot_a_objets,
                                             Liste_zones_recup_noisettes_xy,
                                             Liste_zones_recup_noisettes_angle,
                                             Liste_zones_gm_xy,
                                             Liste_zones_gm_angle,
                                             R_securite,
                                             action_voulu,
                                             DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE):
    """
    Vérifie si la cible est toujours disponible et change si nécessaire.
    """
    
    # Ne réévaluer QUE si on est en déplacement
    if action_voulu not in ["Consigne", "Avancer", "Rotation"]:
        return action_en_cours
    
    if action_en_cours is None:
        return action_en_cours
    
    cible_disponible = verifier_cible_disponible(
        action_en_cours,
        Liste_noisettes_libres,
        Liste_GM_libres
    )
    
    if not cible_disponible:
        print("="*70)
        print("🔄 RÉÉVALUATION FORCÉE : Cible n'est plus disponible")
        print("="*70)
        
        Liste_actions.clear()
        
        action_en_cours = mise_a_jour_decision(
            Liste_actions,
            x_robot_actuel, y_robot_actuel,
            x_ennemi, y_ennemi,
            robot_a_objets,
            Liste_noisettes_libres,
            Liste_GM_libres,
            Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
            Liste_zones_gm_xy, Liste_zones_gm_angle,
            R_securite,
            DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE,
            verbose=True
        )
        
        if len(Liste_actions) == 0:
            print("⚠️  AUCUNE AUTRE CIBLE DISPONIBLE")
        else:
            print(f"✅ Nouvelle cible : {action_en_cours['type']} zone {action_en_cours['numero_zone']}")
    
    return action_en_cours


def mise_a_jour_decision(Liste_actions, 
                        x_robot_actuel, y_robot_actuel, 
                        x_ennemi, y_ennemi, 
                        robot_a_objets,
                        Liste_noisettes_libres,
                        Liste_GM_libres,
                        Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
                        Liste_zones_gm_xy, Liste_zones_gm_angle,
                        R_securite,
                        DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE,
                        verbose=True,
                        grid_expanded=None, case_mm=10):
    """
    Fonction de décision qui choisit la prochaine action.
    Version améliorée pour multi-positions + test faisabilité A*.
    
    Args:
        grid_expanded: Grille des obstacles (optionnel, pour test A*)
        case_mm: Taille d'une case en mm
    """
    
    # Choisir la meilleure action (et la meilleure position)
    action = choisir_prochaine_action(
        x_robot_actuel, y_robot_actuel,
        x_ennemi, y_ennemi,
        robot_a_objets,
        Liste_noisettes_libres, Liste_GM_libres,
        Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
        Liste_zones_gm_xy, Liste_zones_gm_angle,
        R_securite,
        DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE,
        grid_expanded=grid_expanded, case_mm=case_mm
    )
    
    # Ajouter l'action à la liste
    action_ajoutee = ajouter_action_a_liste(Liste_actions, action, verbose)
    
    return action_ajoutee


def choisir_prochaine_action(x_robot_actuel, y_robot_actuel, 
                            x_ennemi, y_ennemi, 
                            robot_a_objets,
                            Liste_noisettes_libres, Liste_GM_libres,
                            Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
                            Liste_zones_gm_xy, Liste_zones_gm_angle,
                            R_securite,
                            DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE,
                            grid_expanded=None, case_mm=10):
    """
    Choisit la prochaine action en gérant plusieurs positions possibles par zone.
    
    ⚠️ VERSION AMÉLIORÉE : Gère 1, 2, 3, 4+ positions par zone
    ⭐ NOUVELLE : Teste la faisabilité du chemin A* avant de retenir une action
    
    Args:
        grid_expanded: Grille des obstacles (optionnel, pour test A*)
        case_mm: Taille d'une case en mm (pour conversion)
    
    Returns:
        dict ou None: {
            'type': "Attraper" ou "Relacher",
            'numero_zone': int,
            'x': float,
            'y': float,
            'angle': float,
            'score': float,
            'position_index': int
        }
    """
    from calcul_mouv import astar_safe
    
    # Liste pour stocker toutes les actions candidates avec leur faisabilité
    actions_candidates = []
    
    # ===== PHASE 1 : Si le robot n'a pas d'objets → ATTRAPER =====
    if not robot_a_objets:
        for num_noisette in Liste_noisettes_libres:
            index = num_noisette - 1
            
            zone_data = Liste_zones_recup_noisettes_xy[index]
            angle_data = Liste_zones_recup_noisettes_angle[index]
            positions = extraire_positions_zone(zone_data, angle_data)
            
            for pos_index, (x, y, angle) in enumerate(positions):
                score = calculer_score_tache(
                    x, y, "Attraper", 
                    robot_a_objets,
                    x_robot_actuel, y_robot_actuel,
                    x_ennemi, y_ennemi, R_securite,
                    DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE
                )
                
                # ⭐ TESTER LA FAISABILITÉ DU CHEMIN A* ⭐
                chemin_possible = True
                if grid_expanded is not None:
                    try:
                        start = (int(x_robot_actuel / case_mm), int(y_robot_actuel / case_mm))
                        goal = (int(x / case_mm), int(y / case_mm))
                        path = astar_safe(start, goal, safety_weight=2.0, grid_dynamique=grid_expanded)
                        chemin_possible = (path is not None and len(path) > 0)
                    except:
                        chemin_possible = False
                
                actions_candidates.append({
                    'type': 'Attraper',
                    'numero_zone': num_noisette,
                    'x': x,
                    'y': y,
                    'angle': angle,
                    'score': score,
                    'position_index': pos_index,
                    'nb_positions': len(positions),
                    'chemin_possible': chemin_possible
                })
    
    # ===== PHASE 2 : Si le robot a des objets → RELACHER =====
    else:
        for num_gm in Liste_GM_libres:
            index = num_gm - 1
            
            zone_data = Liste_zones_gm_xy[index]
            angle_data = Liste_zones_gm_angle[index]
            positions = extraire_positions_zone(zone_data, angle_data)
            
            for pos_index, (x, y, angle) in enumerate(positions):
                score = calculer_score_tache(
                    x, y, "Relacher",
                    robot_a_objets,
                    x_robot_actuel, y_robot_actuel,
                    x_ennemi, y_ennemi, R_securite,
                    DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE
                )
                
                # ⭐ TESTER LA FAISABILITÉ DU CHEMIN A* ⭐
                chemin_possible = True
                if grid_expanded is not None:
                    try:
                        start = (int(x_robot_actuel / case_mm), int(y_robot_actuel / case_mm))
                        goal = (int(x / case_mm), int(y / case_mm))
                        path = astar_safe(start, goal, safety_weight=2.0, grid_dynamique=grid_expanded)
                        chemin_possible = (path is not None and len(path) > 0)
                    except:
                        chemin_possible = False
                
                actions_candidates.append({
                    'type': 'Relacher',
                    'numero_zone': num_gm,
                    'x': x,
                    'y': y,
                    'angle': angle,
                    'score': score,
                    'position_index': pos_index,
                    'nb_positions': len(positions),
                    'chemin_possible': chemin_possible
                })
    
    # ⭐ SÉLECTIONNER LA MEILLEURE ACTION PARMI LES ACCESSIBLES ⭐
    actions_accessibles = [a for a in actions_candidates if a['chemin_possible']]
    
    if len(actions_accessibles) > 0:
        meilleure_action = max(actions_accessibles, key=lambda a: a['score'])
        del meilleure_action['chemin_possible']
        return meilleure_action
    else:
        if len(actions_candidates) > 0:
            print("⚠️  ATTENTION : Aucune action avec chemin A* trouvé !")
            print("   → Choix de la meilleure action par score (peut être inaccessible)")
            meilleure_action = max(actions_candidates, key=lambda a: a['score'])
            del meilleure_action['chemin_possible']
            return meilleure_action
        else:
            return None


def ajouter_action_a_liste(Liste_actions, action_choisie, verbose=True):
    """
    Ajoute les commandes nécessaires à Liste_actions.
    Version améliorée avec affichage de la position choisie.
    """
    if action_choisie is None:
        if verbose:
            print("⚠️  AUCUNE ACTION POSSIBLE !")
        return None
    
    type_action = action_choisie['type']
    numero_zone = action_choisie['numero_zone']
    x = action_choisie['x']
    y = action_choisie['y']
    angle = action_choisie['angle']
    score = action_choisie['score']
    position_index = action_choisie.get('position_index', 0)
    nb_positions = action_choisie.get('nb_positions', 1)
    
    if verbose:
        print(f"\n{'='*70}")
        print(f"✅ ACTION CHOISIE : {type_action} zone {numero_zone}")
        print(f"{'='*70}")
        print(f"   📍 Position : ({x}, {y}) mm")
        print(f"   🧭 Angle    : {angle}°")
        print(f"   ⭐ Score    : {score:.3f}")
        
        # Afficher l'info sur les positions multiples
        if nb_positions > 1:
            print(f"   🔢 Position {position_index + 1}/{nb_positions} choisie")
            print(f"      (Zone avec {nb_positions} positions possibles)")
        
        print(f"{'='*70}\n")
    
    # Ajouter les commandes à la liste
    Liste_actions.append(["Consigne", x, y])
    Liste_actions.append(["Rotation", angle])
    Liste_actions.append([type_action])
    
    return action_choisie


def extraire_positions_zone(zone_data, angle_data):
    """
    Extrait toutes les positions possibles d'une zone.
    
    Args:
        zone_data: Peut être (x, y) ou ((x1, y1), (x2, y2), ...)
        angle_data: Peut être angle ou (angle1, angle2, ...)
    
    Returns:
        Liste de tuples [(x1, y1, angle1), (x2, y2, angle2), ...]
    """
    positions = []
    
    # Vérifier si zone_data est un tuple de positions ou une position unique
    if isinstance(zone_data, tuple) and len(zone_data) >= 2:
        # Vérifier si c'est une position unique (x, y) ou plusieurs positions
        if isinstance(zone_data[0], (int, float)):
            # C'est une position unique : (x, y)
            positions.append((zone_data[0], zone_data[1], angle_data))
        else:
            # C'est plusieurs positions : ((x1, y1), (x2, y2), ...)
            # Extraire les angles correspondants
            if isinstance(angle_data, tuple):
                angles = angle_data
            else:
                # Si un seul angle donné, le répéter pour toutes les positions
                angles = (angle_data,) * len(zone_data)
            
            # Créer la liste des positions avec leurs angles
            for i, pos in enumerate(zone_data):
                if isinstance(pos, tuple) and len(pos) >= 2:
                    x, y = pos[0], pos[1]
                    angle = angles[i] if i < len(angles) else angles[0]
                    positions.append((x, y, angle))
    
    return positions


def calculer_score_tache(x_tache, y_tache, type_tache, robot_a_objets,
                         x_robot_actuel, y_robot_actuel, 
                         x_ennemi, y_ennemi, R_securite,
                         DISTANCE_MAX_TERRAIN,W_DISTANCE,W_SECURITE,W_PRIORITE,W_EFFICACITE):
    """
    Calcule le score d'une tâche selon la moyenne pondérée.
    """
    
    # 1. FACTEUR DISTANCE
    distance = distance_euclidienne(x_robot_actuel, y_robot_actuel, x_tache, y_tache)
    distance_normalisee = distance / DISTANCE_MAX_TERRAIN
    f_distance = 1.0 / (1.0 + distance_normalisee)
    
    # 2. FACTEUR SÉCURITÉ
    distance_tache_ennemi = distance_euclidienne(x_tache, y_tache, x_ennemi, y_ennemi)
    
    if distance_tache_ennemi < R_securite:
        f_securite = 0.0
    else:
        f_securite = min(1.0, distance_tache_ennemi / (2.5 * R_securite))
    
    # 3. FACTEUR PRIORITÉ
    if type_tache == "Attraper":
        f_priorite = 1.0 
    else:
        f_priorite = 0.8
    
    # 4. FACTEUR EFFICACITÉ
    if robot_a_objets and type_tache == "Relacher":
        f_efficacite = 1.0
    elif not robot_a_objets and type_tache == "Attraper":
        f_efficacite = 1.0
    else:
        f_efficacite = 0.3
    
    # SCORE TOTAL
    score = (W_DISTANCE * f_distance + 
             W_SECURITE * f_securite + 
             W_PRIORITE * f_priorite + 
             W_EFFICACITE * f_efficacite)
    
    return score


def confirmer_action_terminee(action_en_cours,
                              robot_a_objets,
                              Liste_noisettes_libres, Liste_noisettes_prises,
                              Liste_GM_libres, Liste_GM_occuper,
                              verbose=True):
    """
    Met à jour les listes de zones et l'état du robot APRÈS confirmation.
    """
    if action_en_cours is None:
        return robot_a_objets
    
    type_action = action_en_cours['type']
    numero_zone = action_en_cours['numero_zone']
    
    if type_action == "Attraper":
        if numero_zone in Liste_noisettes_libres:
            Liste_noisettes_libres.remove(numero_zone)
            Liste_noisettes_prises.append(numero_zone)
            if verbose:
                print(f"✅ Noisette {numero_zone} RÉCUPÉRÉE avec succès !")
                print(f"   Noisettes restantes : {Liste_noisettes_libres}")
        robot_a_objets = True
        
    elif type_action == "Relacher":
        if numero_zone in Liste_GM_libres:
            Liste_GM_libres.remove(numero_zone)
            Liste_GM_occuper.append(numero_zone)
            if verbose:
                print(f"✅ GM {numero_zone} REMPLI avec succès !")
                print(f"   GM restants : {Liste_GM_libres}")
        robot_a_objets = False
    
    return robot_a_objets


def afficher_tous_les_scores(x_robot_actuel, y_robot_actuel,
                             x_ennemi, y_ennemi,
                             robot_a_objets,
                             Liste_noisettes_libres, Liste_GM_libres,
                             Liste_zones_recup_noisettes_xy, Liste_zones_recup_noisettes_angle,
                             Liste_zones_gm_xy, Liste_zones_gm_angle,
                             R_securite):
    """
    Fonction de debug : affiche les scores de TOUTES les positions possibles.
    Version améliorée pour multi-positions.
    """
    print("\n" + "="*70)
    print("DEBUG : SCORES DE TOUTES LES ACTIONS POSSIBLES")
    print("="*70)
    
    if not robot_a_objets:
        print("\n🔵 PHASE RÉCOLTE (ATTRAPER)")
        print("-" * 70)
        for num_noisette in Liste_noisettes_libres:
            index = num_noisette - 1
            zone_data = Liste_zones_recup_noisettes_xy[index]
            angle_data = Liste_zones_recup_noisettes_angle[index]
            positions = extraire_positions_zone(zone_data, angle_data)
            
            if len(positions) == 1:
                # Une seule position
                x, y, angle = positions[0]
                score = calculer_score_tache(
                    x, y, "Attraper", robot_a_objets,
                    x_robot_actuel, y_robot_actuel,
                    x_ennemi, y_ennemi, R_securite
                )
                dist = distance_euclidienne(x_robot_actuel, y_robot_actuel, x, y)
                dist_ennemi = distance_euclidienne(x, y, x_ennemi, y_ennemi)
                print(f"  Noisette {num_noisette:2d} → Score: {score:.3f} | "
                      f"Dist robot: {dist:4.0f}mm | Dist ennemi: {dist_ennemi:4.0f}mm")
            else:
                # Plusieurs positions
                print(f"  Noisette {num_noisette:2d} ({len(positions)} positions) :")
                for pos_idx, (x, y, angle) in enumerate(positions):
                    score = calculer_score_tache(
                        x, y, "Attraper", robot_a_objets,
                        x_robot_actuel, y_robot_actuel,
                        x_ennemi, y_ennemi, R_securite
                    )
                    dist = distance_euclidienne(x_robot_actuel, y_robot_actuel, x, y)
                    dist_ennemi = distance_euclidienne(x, y, x_ennemi, y_ennemi)
                    print(f"    Pos {pos_idx + 1} ({x:4.0f}, {y:4.0f}) → "
                          f"Score: {score:.3f} | Dist robot: {dist:4.0f}mm | "
                          f"Dist ennemi: {dist_ennemi:4.0f}mm")
    
    else:
        print("\n🔴 PHASE DÉPÔT (RELACHER)")
        print("-" * 70)
        for num_gm in Liste_GM_libres:
            index = num_gm - 1
            zone_data = Liste_zones_gm_xy[index]
            angle_data = Liste_zones_gm_angle[index]
            positions = extraire_positions_zone(zone_data, angle_data)
            
            if len(positions) == 1:
                # Une seule position
                x, y, angle = positions[0]
                score = calculer_score_tache(
                    x, y, "Relacher", robot_a_objets,
                    x_robot_actuel, y_robot_actuel,
                    x_ennemi, y_ennemi, R_securite
                )
                dist = distance_euclidienne(x_robot_actuel, y_robot_actuel, x, y)
                dist_ennemi = distance_euclidienne(x, y, x_ennemi, y_ennemi)
                print(f"  GM {num_gm:2d} → Score: {score:.3f} | "
                      f"Dist robot: {dist:4.0f}mm | Dist ennemi: {dist_ennemi:4.0f}mm")
            else:
                # Plusieurs positions
                print(f"  GM {num_gm:2d} ({len(positions)} positions) :")
                for pos_idx, (x, y, angle) in enumerate(positions):
                    score = calculer_score_tache(
                        x, y, "Relacher", robot_a_objets,
                        x_robot_actuel, y_robot_actuel,
                        x_ennemi, y_ennemi, R_securite
                    )
                    dist = distance_euclidienne(x_robot_actuel, y_robot_actuel, x, y)
                    dist_ennemi = distance_euclidienne(x, y, x_ennemi, y_ennemi)
                    print(f"    Pos {pos_idx + 1} ({x:4.0f}, {y:4.0f}) → "
                          f"Score: {score:.3f} | Dist robot: {dist:4.0f}mm | "
                          f"Dist ennemi: {dist_ennemi:4.0f}mm")
    
    print("="*70 + "\n")
