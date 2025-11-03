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
    
    print("\n" + "="*70)
    print("🔄 MISE À JOUR DYNAMIQUE DES ZONES")
    print("="*70)
    
    # ========================================================================
    # GESTION DES ZONES (GARDEMANGER) - numérotées de 1 à 10
    # ========================================================================
    print("\n--- ZONES GARDEMANGER ---")
    
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
                    print(f"  ✅ Zone {numero_zone:2d} ACTIVÉE (zone {raison})")
                else:
                    obs_manager.desactiver(nom_zone)
                    print(f"  ⚪ Zone {numero_zone:2d} DÉSACTIVÉE (zone {raison})")
                grilles_modifiees = True
            else:
                # Pas de changement
                symbole = "✅" if etat_actuel else "⚪"
                print(f"  {symbole} Zone {numero_zone:2d} déjà {'active' if etat_actuel else 'inactive'} (zone {raison})")
            
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
    print("\n--- NOISETTES ---")
    
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
                    print(f"  🌰 Noisette {numero_noisette} ACTIVÉE ({raison})")
                else:
                    obs_manager_noisettes.desactiver(nom_noisette)
                    print(f"  ✓  Noisette {numero_noisette} DÉSACTIVÉE ({raison})")
                grilles_modifiees = True
            else:
                # Pas de changement
                symbole = "🌰" if etat_actuel else "✓ "
                print(f"  {symbole} Noisette {numero_noisette} déjà {'active' if etat_actuel else 'inactive'} ({raison})")
            
            rapport_noisettes[numero_noisette] = {
                "actif": etat_souhaite,
                "raison": raison,
                "modifie": etat_actuel != etat_souhaite
            }
        else:
            print(f"  ⚠️  Noisette {numero_noisette} n'existe pas dans obs_manager_noisettes")
    
    # ========================================================================
    # RÉSUMÉ
    # ========================================================================
    print("\n" + "-"*70)
    if grilles_modifiees:
        print("⚠️  GRILLES MODIFIÉES : Régénération nécessaire")
    else:
        print("✓  Aucune modification - Grilles inchangées")
    print("="*70 + "\n")
    
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


if __name__ == "__main__":
    exemple_utilisation()
