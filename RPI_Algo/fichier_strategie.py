from fonction import peuvent_etre_groupees, distance
import math
import numpy as np
def trouver_groupes_initiaux(noisettes):
    """Trouve tous les groupes de noisettes connectées"""
    n = len(noisettes)
    adjacence = [[] for _ in range(n)]
    
    # Construire le graphe d'adjacence
    for i in range(n):
        for j in range(i + 1, n):
            if peuvent_etre_groupees(noisettes[i], noisettes[j]):
                adjacence[i].append(j)
                adjacence[j].append(i)
    
    # Trouver les composantes connexes
    visite = [False] * n
    groupes = []
    
    for i in range(n):
        if not visite[i]:
            groupe = []
            pile = [i]
            while pile:
                noeud = pile.pop()
                if not visite[noeud]:
                    visite[noeud] = True
                    groupe.append(noeud)
                    pile.extend(adjacence[noeud])
            groupes.append(sorted(groupe))
    
    return groupes, adjacence


def separer_groupe(groupe_indices, noisettes, adjacence):
    """Sépare un groupe en paires et noisettes seules selon les règles"""
    if len(groupe_indices) == 1:
        return [[groupe_indices[0]]]
    
    if len(groupe_indices) == 2:
        return [groupe_indices]
    
    # Pour les groupes de 3 ou plus, on utilise une approche gloutonne
    # On forme des paires en priorisant les noisettes avec le moins de voisins
    indices_restants = set(groupe_indices)
    paires = []
    
    while len(indices_restants) >= 2:
        # Trouver la noisette avec le moins de voisins non appariés
        min_voisins = float('inf')
        noisette_depart = None
        
        for idx in indices_restants:
            voisins_disponibles = [v for v in adjacence[idx] if v in indices_restants and v != idx]
            if len(voisins_disponibles) < min_voisins:
                min_voisins = len(voisins_disponibles)
                noisette_depart = idx
        
        # Trouver le voisin le plus proche
        voisins_disponibles = [v for v in adjacence[noisette_depart] if v in indices_restants and v != noisette_depart]
        
        if voisins_disponibles:
            # Choisir le voisin le plus proche
            voisin_choisi = min(voisins_disponibles, 
                               key=lambda v: distance(noisettes[noisette_depart], noisettes[v]))
            paires.append([noisette_depart, voisin_choisi])
            indices_restants.remove(noisette_depart)
            indices_restants.remove(voisin_choisi)
        else:
            # Pas de voisin disponible, mettre seul
            paires.append([noisette_depart])
            indices_restants.remove(noisette_depart)
    
    # Ajouter les noisettes restantes seules
    for idx in indices_restants:
        paires.append([idx])
    
    return paires

def regrouper_par_quatre(groupe_indices, noisettes):
    """Regroupe les noisettes en groupes de 4 maximum"""
    groupes_de_quatre = []
    
    # Trier les indices par position pour un regroupement cohérent
    indices_tries = sorted(groupe_indices, key=lambda i: (noisettes[i][0], noisettes[i][1]))
    
    # Créer des groupes de 4
    for i in range(0, len(indices_tries), 4):
        groupe = indices_tries[i:i+4]
        groupes_de_quatre.append(groupe)
    
    return groupes_de_quatre

def positionner_robot_devant_Noisette1(x_strategie,y_strategie,Noisettes_groupees,strategie_en_cours,demande_nouvelle_strat,Liste_actions,couleur,Liste_zones_gm_coins,TOLERANCE_STRATEGIE_NOISETTE,Noisettes_stockees_dans_robot,Debug_strategie,MARGE_NOISETTE,LONGUEUR_ROBOT,Liste_noisette_xya,x_robot_actuel,y_robot_actuel,Pince_Avant, Pince_Av_1, Pince_Av_2,Pince_Arriere, Pince_Ar_1, Pince_Ar_2):

    Liste_actions.clear()

    chercher_Noisette = None
    """for num_gm in range(len(Liste_zones_gm_coins)):
        if Liste_zones_gm_coins[num_gm][0][0]<= x_strategie <= Liste_zones_gm_coins[num_gm][1][0] and Liste_zones_gm_coins[num_gm][0][1]<= y_strategie <= Liste_zones_gm_coins[num_gm][1][1]:
            strategie_en_cours = num_gm
            chercher_Noisette = False
            break
    print(f"GM n°{num_gm}")"""

    for grpNoisette in Noisettes_groupees:
        nbr_Noisette = len(grpNoisette)
        x_centre = 0
        y_centre = 0
        angle_groupe = 0
        for num_Noisette in range(nbr_Noisette):
            x_centre += grpNoisette[num_Noisette][0]
            y_centre += grpNoisette[num_Noisette][1]
            angle_groupe += grpNoisette[num_Noisette][2]
        x_centre/=nbr_Noisette
        y_centre/=nbr_Noisette
        angle_groupe /=nbr_Noisette
        distance_Noisette_strategie = math.sqrt((x_strategie - x_centre)**2 + (y_strategie - y_centre)**2)
        if distance_Noisette_strategie <= TOLERANCE_STRATEGIE_NOISETTE:
            strategie_en_cours = grpNoisette
            chercher_Noisette = True
            break

    print("strategie_en_cours : ",strategie_en_cours)

    if chercher_Noisette:
        if len(strategie_en_cours)==1:
            angle_noisette = strategie_en_cours[0][2]

            pince_a_utilise = None
            sous_pince = None
            if Noisettes_stockees_dans_robot[0]!=["J","B"] and Noisettes_stockees_dans_robot[0]!=["B","J"] and Noisettes_stockees_dans_robot[0]!=["B","B"] and Noisettes_stockees_dans_robot[0]!=["J","J"]:
                if Debug_strategie:
                    print("Utiliser Pince Avant")
                pince_a_utilise = 0
                if Noisettes_stockees_dans_robot[0][0]=="N":
                    sous_pince = 0
                else:
                    sous_pince = 1
            elif Noisettes_stockees_dans_robot[1]!=["J","B"] and Noisettes_stockees_dans_robot[1]!=["B","J"] and Noisettes_stockees_dans_robot[1]!=["B","B"] and Noisettes_stockees_dans_robot[1]!=["J","J"]:
                pince_a_utilise = 1
                if Noisettes_stockees_dans_robot[1][0]=="N":
                    sous_pince = 0
                else:
                    sous_pince = 1
            else:
                demande_nouvelle_strat = True

            if Debug_strategie:
                print("pince_a_utilise : ",pince_a_utilise)
                print("sous_pince : ",sous_pince)

            distance = 50*sous_pince + 25 + MARGE_NOISETTE + LONGUEUR_ROBOT/2
            angle_rad1 = math.radians(angle_noisette + 90)
            angle_rad2 = math.radians(angle_noisette + 90 - 180)
            
            x_arrivee_1 = strategie_en_cours[0][0]+distance*math.cos(angle_rad1)
            y_arrivee_1 = strategie_en_cours[0][1]+distance*math.sin(angle_rad1)

            x_arrivee_2 = strategie_en_cours[0][0]+distance*math.cos(angle_rad2)
            y_arrivee_2 = strategie_en_cours[0][1]+distance*math.sin(angle_rad2)
            
            MARGE_X = 45+45*math.cos(np.radians(angle_groupe))   # Largeur du couloir en X
            MARGE_Y = 45+45*math.sin(np.radians(angle_groupe))  # Largeur du couloir en Y
            point_1_bloquee = False
            point_2_bloquee = False

            for Noisette in Liste_noisette_xya:
                if Noisette not in strategie_en_cours:
                    # Vérification point 1 : la noisette doit être dans le rectangle ET dans les deux axes
                    x_dans_intervalle_1 = (min(x_arrivee_1, strategie_en_cours[0][0]) - MARGE_X <= Noisette[0] 
                                        <= max(x_arrivee_1, strategie_en_cours[0][0]) + MARGE_X)
                    y_dans_intervalle_1 = (min(y_arrivee_1, strategie_en_cours[0][1]) - MARGE_Y <= Noisette[1] 
                                        <= max(y_arrivee_1, strategie_en_cours[0][1]) + MARGE_Y)
                    
                    if x_dans_intervalle_1 and y_dans_intervalle_1:
                        if Debug_strategie:
                            print(f"Point d'arrivée 1 bloqué par Noisette : {Noisette}")
                        point_1_bloquee = True
                    
                    # Vérification point 2
                    x_dans_intervalle_2 = (min(x_arrivee_2, strategie_en_cours[-1][0]) - MARGE_X <= Noisette[0] 
                                        <= max(x_arrivee_2, strategie_en_cours[-1][0]) + MARGE_X)
                    y_dans_intervalle_2 = (min(y_arrivee_2, strategie_en_cours[-1][1]) - MARGE_Y <= Noisette[1] 
                                        <= max(y_arrivee_2, strategie_en_cours[-1][1]) + MARGE_Y)
                    
                    if x_dans_intervalle_2 and y_dans_intervalle_2:
                        if Debug_strategie:
                            print(f"Point d'arrivée 2 bloqué par Noisette : {Noisette}")
                        point_2_bloquee = True

            if point_1_bloquee and point_2_bloquee:
                demande_nouvelle_strat = True
            else:
                distance = 50*sous_pince + 25 + 4*MARGE_NOISETTE+LONGUEUR_ROBOT/2
                if point_1_bloquee:
                    if pince_a_utilise == 0:
                        Liste_actions = [["Rotation",180-90+angle_noisette],["Consigne",x_arrivee_2,y_arrivee_2],["Rotation",180-90+angle_noisette]]
                    elif pince_a_utilise == 1:
                        Liste_actions = [["Rotation",-90+angle_noisette],["ReculerPrecis",x_arrivee_2,y_arrivee_2],["Rotation",-90+angle_noisette]]

                    x_arrivee_Astar = strategie_en_cours[0][0]+distance*math.cos(angle_rad2)
                    y_arrivee_Astar = strategie_en_cours[0][1]+distance*math.sin(angle_rad2)
                    Liste_actions.insert(0,["Consigne",x_arrivee_Astar,y_arrivee_Astar])
                elif point_2_bloquee:
                    if pince_a_utilise == 0:
                        Liste_actions = [["Rotation",-90+angle_noisette],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",-90+angle_noisette]]
                    elif pince_a_utilise == 1:
                        Liste_actions = [["Rotation",180-90+angle_noisette],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",180-90+angle_noisette]]

                    x_arrivee_Astar = strategie_en_cours[0][0]+distance*math.cos(angle_rad1)
                    y_arrivee_Astar = strategie_en_cours[0][1]+distance*math.sin(angle_rad1)
                    Liste_actions.insert(0,["Consigne",x_arrivee_Astar,y_arrivee_Astar])
                else:
                    distance_robot_point1 = math.sqrt((x_arrivee_1 - x_robot_actuel)**2 + (y_arrivee_1 - y_robot_actuel)**2)
                    distance_robot_point2 = math.sqrt((x_arrivee_2 - x_robot_actuel)**2 + (y_arrivee_2 - y_robot_actuel)**2)
                    if distance_robot_point1 <= distance_robot_point2:
                        if pince_a_utilise == 0:
                            Liste_actions = [["Rotation",-90+angle_noisette],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",-90+angle_noisette]]
                        elif pince_a_utilise == 1:
                            Liste_actions = [["Rotation",180-90+angle_noisette],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",180-90+angle_noisette]]

                        x_arrivee_Astar = strategie_en_cours[0][0]+distance*math.cos(angle_rad1)
                        y_arrivee_Astar = strategie_en_cours[0][1]+distance*math.sin(angle_rad1)
                        Liste_actions.insert(0,["Consigne",x_arrivee_Astar,y_arrivee_Astar])
                    else :
                        if pince_a_utilise == 0:
                            Liste_actions = [["Rotation",180-90+angle_noisette],["Consigne",x_arrivee_2,y_arrivee_2],["Rotation",180-90+angle_noisette]]
                        elif pince_a_utilise == 1:
                            Liste_actions = [["Rotation",-90+angle_noisette],["ReculerPrecis",x_arrivee_2,y_arrivee_2],["Rotation",90+angle_noisette]]

                        x_arrivee_Astar = strategie_en_cours[0][0]+distance*math.cos(angle_rad2)
                        y_arrivee_Astar = strategie_en_cours[0][1]+distance*math.sin(angle_rad2)
                        Liste_actions.insert(0,["Consigne",x_arrivee_Astar,y_arrivee_Astar])
            Liste_actions.append(["Attraper",pince_a_utilise,sous_pince+1])

            if strategie_en_cours[0][3] != couleur:
                Liste_actions.append(["Retourner",pince_a_utilise,sous_pince+1])


        else :
            pince_a_utilise = None
            sous_pince = None
            if Noisettes_stockees_dans_robot[0]==["N","N"] and Pince_Av_1 and Pince_Av_2:
                pince_a_utilise = 0
                sous_pince = 12
            elif Noisettes_stockees_dans_robot[1]==["N","N"] and Pince_Ar_1 and Pince_Ar_2:
                pince_a_utilise = 1
                sous_pince = 12

            elif Noisettes_stockees_dans_robot[0]==["N","N"] and Pince_Av_1 and not Pince_Av_2:
                pince_a_utilise = 0
                sous_pince = 1
            elif Noisettes_stockees_dans_robot[0]==["N","N"] and not Pince_Av_1 and Pince_Av_2:
                pince_a_utilise = 0
                sous_pince = 2

            elif Noisettes_stockees_dans_robot[0]==["N","N"] and Pince_Ar_1 and not Pince_Ar_2:
                pince_a_utilise = 1
                sous_pince = 1
            elif Noisettes_stockees_dans_robot[0]==["N","N"] and not Pince_Ar_1 and Pince_Ar_2:
                pince_a_utilise = 1
                sous_pince = 2

            else:
                demande_nouvelle_strat = True

            if Debug_strategie:
                print("pince_a_utilise : ",pince_a_utilise)
                print("sous_pince : ",sous_pince)
            distance_robot_n0 = math.sqrt((x_robot_actuel - strategie_en_cours[0][0])**2 + (y_robot_actuel - strategie_en_cours[0][1])**2)
            distance_robot_n1 = math.sqrt((x_robot_actuel - strategie_en_cours[1][0])**2 + (y_robot_actuel - strategie_en_cours[1][1])**2)
            if Debug_strategie:
                print("Distance entre robot et n0 : ",distance_robot_n0)
                print("Distance entre robot et n1 : ",distance_robot_n1)
                print("Strategie avant tri : ",strategie_en_cours)
            # Tri des noisettes par distance
            if distance_robot_n0 > distance_robot_n1:
                temp = strategie_en_cours[0]
                strategie_en_cours[0] = strategie_en_cours[1]
                strategie_en_cours[1] = temp
                distance_robot_n0, distance_robot_n1 = distance_robot_n1, distance_robot_n0
            
            if Debug_strategie:
                print("Strategie après tri : ",strategie_en_cours)
            
            angle_noisette1 = strategie_en_cours[0][2]
            angle_noisette2 = strategie_en_cours[len(strategie_en_cours)-1][2]
            distance = 25 + MARGE_NOISETTE + LONGUEUR_ROBOT/2

            # Normaliser l'angle entre 0 et 180°
            angle_moyen = (angle_noisette1 + angle_noisette2) / 2
            angle_normalise = angle_moyen % 180

            # ⭐ CALCULER LA POSITION RELATIVE DU ROBOT PAR RAPPORT AUX NOISETTES ⭐
            x_centre_noisettes = (strategie_en_cours[0][0] + strategie_en_cours[1][0]) / 2
            y_centre_noisettes = (strategie_en_cours[0][1] + strategie_en_cours[1][1]) / 2


            # Déterminer si les noisettes sont horizontales (~0°) ou verticales (~90°)
            if 45 < angle_normalise < 135:  # Noisettes verticales (~90°)
                if Debug_strategie:
                    print("Noisettes verticales détectées")
                
                # Robot à gauche ou à droite des noisettes ?
                robot_a_gauche = x_robot_actuel < x_centre_noisettes
                
                if robot_a_gauche:
                    # Approcher par la gauche
                    angle_rad1 = math.radians(angle_noisette1 + 90)  # Côté gauche
                    angle_rad2 = math.radians(angle_noisette2 - 90)
                else:
                    # Approcher par la droite
                    angle_rad1 = math.radians(angle_noisette1 - 90)  # Côté droit
                    angle_rad2 = math.radians(angle_noisette2 + 90)

            else:  # Noisettes horizontales (~0° ou ~180°)
                if Debug_strategie:
                    print("Noisettes horizontales détectées")
                
                # Robot en haut ou en bas des noisettes ?
                robot_en_bas = y_robot_actuel < y_centre_noisettes
                
                if robot_en_bas:
                    # Approcher par le bas
                    angle_rad1 = math.radians(angle_noisette1 - 90)  # En bas
                    angle_rad2 = math.radians(angle_noisette2 + 90)
                else:
                    # Approcher par le haut
                    angle_rad1 = math.radians(angle_noisette1 + 90)  # En haut
                    angle_rad2 = math.radians(angle_noisette2 - 90)

            if Debug_strategie:
                print("angle_noisette1 : ",angle_noisette1-90)
                print("angle_noisette2 : ",angle_noisette2+90)
                print("Point1 associé à Noisette : ",strategie_en_cours[0])
                print("Point2 associé à Noisette : ",strategie_en_cours[1])
            x_arrivee_1 = strategie_en_cours[0][0]+distance*math.cos(angle_rad1)
            y_arrivee_1 = strategie_en_cours[0][1]+distance*math.sin(angle_rad1)

            x_arrivee_2 = strategie_en_cours[len(strategie_en_cours)-1][0]+distance*math.cos(angle_rad2)
            y_arrivee_2 = strategie_en_cours[len(strategie_en_cours)-1][1]+distance*math.sin(angle_rad2)
            
            Liste_actions = [["Consigne",x_arrivee_1,y_arrivee_1],["Consigne",x_arrivee_2,y_arrivee_2]]
            
            MARGE_X = 45+45*math.cos(np.radians(angle_groupe))   # Largeur du couloir en X
            MARGE_Y = 45+45*math.sin(np.radians(angle_groupe))  # Largeur du couloir en Y
            
            point_1_bloquee = False
            point_2_bloquee = False

            for Noisette in Liste_noisette_xya:
                if Noisette not in strategie_en_cours:
                    # Vérification point 1 : la noisette doit être dans le rectangle ET dans les deux axes
                    x_dans_intervalle_1 = (min(x_arrivee_1, strategie_en_cours[0][0]) - MARGE_X <= Noisette[0] 
                                        <= max(x_arrivee_1, strategie_en_cours[0][0]) + MARGE_X)
                    y_dans_intervalle_1 = (min(y_arrivee_1, strategie_en_cours[0][1]) - MARGE_Y <= Noisette[1] 
                                        <= max(y_arrivee_1, strategie_en_cours[0][1]) + MARGE_Y)
                    
                    if x_dans_intervalle_1 and y_dans_intervalle_1:
                        if Debug_strategie:
                            print(f"Point d'arrivée 1 bloqué par Noisette : {Noisette}")
                        point_1_bloquee = True
                    
                    # Vérification point 2
                    x_dans_intervalle_2 = (min(x_arrivee_2, strategie_en_cours[-1][0]) - MARGE_X <= Noisette[0] 
                                        <= max(x_arrivee_2, strategie_en_cours[-1][0]) + MARGE_X)
                    y_dans_intervalle_2 = (min(y_arrivee_2, strategie_en_cours[-1][1]) - MARGE_Y <= Noisette[1] 
                                        <= max(y_arrivee_2, strategie_en_cours[-1][1]) + MARGE_Y)
                    
                    if x_dans_intervalle_2 and y_dans_intervalle_2:
                        if Debug_strategie:
                            print(f"Point d'arrivée 2 bloqué par Noisette : {Noisette}")
                        point_2_bloquee = True

            if point_1_bloquee and point_2_bloquee:
                demande_nouvelle_strat = True

            else:
                if point_1_bloquee:
                    if Debug_strategie:
                        print("point 1 bloqué")
                    angle_pointarrivee_noisette = int(np.degrees(math.atan2(strategie_en_cours[1][1] - y_arrivee_2, strategie_en_cours[1][0] - x_arrivee_2)))

                    if pince_a_utilise == 0:
                            Liste_actions = [["Rotation",angle_pointarrivee_noisette],["Consigne",x_arrivee_2,y_arrivee_2],["Rotation",angle_pointarrivee_noisette]]
                    elif pince_a_utilise == 1:
                            Liste_actions = [["Rotation",180+angle_pointarrivee_noisette],["ReculerPrecis",x_arrivee_2,y_arrivee_2],["Rotation",180+angle_pointarrivee_noisette]]

                    distance = 25 + 4*MARGE_NOISETTE+LONGUEUR_ROBOT/2
                    x_arrivee_Astar = strategie_en_cours[len(strategie_en_cours)-1][0]+distance*math.cos(angle_rad2)
                    y_arrivee_Astar = strategie_en_cours[len(strategie_en_cours)-1][1]+distance*math.sin(angle_rad2)
                    Liste_actions.insert(0,["Consigne",x_arrivee_Astar,y_arrivee_Astar])

                elif point_2_bloquee:
                    if Debug_strategie:
                        print("point 2 bloqué")
                    angle_pointarrivee_noisette = int(np.degrees(math.atan2(strategie_en_cours[0][1] - y_arrivee_1, strategie_en_cours[0][0] - x_arrivee_1)))
                    if pince_a_utilise == 0:
                        Liste_actions = [["Rotation",angle_pointarrivee_noisette],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",angle_pointarrivee_noisette]]
                    elif pince_a_utilise == 1:
                        Liste_actions = [["Rotation",180+angle_pointarrivee_noisette],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",180+angle_pointarrivee_noisette]]

                    distance = 25 + 4*MARGE_NOISETTE+LONGUEUR_ROBOT/2
                    x_arrivee_Astar = strategie_en_cours[0][0]+distance*math.cos(angle_rad1)
                    y_arrivee_Astar = strategie_en_cours[0][1]+distance*math.sin(angle_rad1)
                    Liste_actions.insert(0,["Consigne",x_arrivee_Astar,y_arrivee_Astar])
                else:
                    print("Point 1 :",x_arrivee_1,"  ",y_arrivee_1)
                    print("Point 2 :",x_arrivee_2,"  ",y_arrivee_2)
                    distance_robot_point1 = math.sqrt((x_arrivee_1 - x_robot_actuel)**2 + (y_arrivee_1 - y_robot_actuel)**2)
                    distance_robot_point2 = math.sqrt((x_arrivee_2 - x_robot_actuel)**2 + (y_arrivee_2 - y_robot_actuel)**2)
                    if Debug_strategie:
                        print("distance_robot_point1 : ",distance_robot_point1)
                        print("distance_robot_point2 : ",distance_robot_point2)
                    if distance_robot_point1 < distance_robot_point2:
                        if Debug_strategie:
                            print("aller point 1")
                        angle_pointarrivee_noisette = int(np.degrees(math.atan2(strategie_en_cours[0][1] - y_arrivee_1, strategie_en_cours[0][0] - x_arrivee_1)))
                        if pince_a_utilise == 0:
                            Liste_actions = [["Rotation",angle_pointarrivee_noisette],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",angle_pointarrivee_noisette]]
                        elif pince_a_utilise == 1:
                            Liste_actions = [["Rotation",180+angle_pointarrivee_noisette],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",180+angle_pointarrivee_noisette]]

                        distance = 25 + 4*MARGE_NOISETTE+LONGUEUR_ROBOT/2
                        x_arrivee_Astar = strategie_en_cours[0][0]+distance*math.cos(angle_rad1)
                        y_arrivee_Astar = strategie_en_cours[0][1]+distance*math.sin(angle_rad1)
                        Liste_actions.insert(0,["Consigne",x_arrivee_Astar,y_arrivee_Astar])
                    else :
                        if Debug_strategie:
                            print("aller point 2")
                        angle_pointarrivee_noisette = int(np.degrees(math.atan2(strategie_en_cours[1][1] - y_arrivee_2, strategie_en_cours[1][0] - x_arrivee_2)))
                        if pince_a_utilise == 0:
                            Liste_actions =[["Rotation",angle_pointarrivee_noisette],["Consigne",x_arrivee_2,y_arrivee_2],["Rotation",angle_pointarrivee_noisette]]
                        elif pince_a_utilise == 1:
                            Liste_actions = [["Rotation",180+angle_pointarrivee_noisette],["Consigne",x_arrivee_2,y_arrivee_2],["Rotation",180+angle_pointarrivee_noisette]]

                        distance = 25 + 4*MARGE_NOISETTE+LONGUEUR_ROBOT/2
                        x_arrivee_Astar = strategie_en_cours[len(strategie_en_cours)-1][0]+distance*math.cos(angle_rad2)
                        y_arrivee_Astar = strategie_en_cours[len(strategie_en_cours)-1][1]+distance*math.sin(angle_rad2)
                        Liste_actions.insert(0,["Consigne",x_arrivee_Astar,y_arrivee_Astar])
            
            Liste_actions.append(["Attraper",pince_a_utilise,sous_pince])

            if strategie_en_cours[0][3] != couleur and strategie_en_cours[1][3] != couleur:
                Liste_actions.append(["Retourner",pince_a_utilise,12])
                
            if point_1_bloquee:
                if strategie_en_cours[0][3] != couleur and strategie_en_cours[1][3] == couleur:
                    Liste_actions.append(["Retourner",pince_a_utilise,2])
                elif strategie_en_cours[0][3] == couleur and strategie_en_cours[1][3] != couleur:
                    Liste_actions.append(["Retourner",pince_a_utilise,1])
            else:
                if strategie_en_cours[0][3] != couleur and strategie_en_cours[1][3] == couleur:
                    Liste_actions.append(["Retourner",pince_a_utilise,1])
                elif strategie_en_cours[0][3] == couleur and strategie_en_cours[1][3] != couleur:
                    Liste_actions.append(["Retourner",pince_a_utilise,2])
    """else:
        print("aller gm")
        if Noisettes_stockees_dans_robot == [["N","N"],["N","N"]]:
            print("Pas de Noisette dans robot")
            demande_nouvelle_strat = True
        else:
            if Noisettes_stockees_dans_robot[0] != ["N","N"]:
                pince_a_utilise = 0
            else :
                pince_a_utilise = 1
            if Noisettes_stockees_dans_robot[pince_a_utilise] == ["J","B"] or Noisettes_stockees_dans_robot[pince_a_utilise] == ["B","J"] or Noisettes_stockees_dans_robot[pince_a_utilise] == ["J","J"] or Noisettes_stockees_dans_robot[pince_a_utilise] == ["B","B"]:
                sous_pince = 12
            elif Noisettes_stockees_dans_robot[pince_a_utilise] == ["J","N"] or Noisettes_stockees_dans_robot[pince_a_utilise] == ["B","N"]:
                sous_pince = 1
            elif Noisettes_stockees_dans_robot[pince_a_utilise] == ["N","J"] or Noisettes_stockees_dans_robot[pince_a_utilise] == ["N","B"]:
                sous_pince = 2

            print("pince_a_utilise",pince_a_utilise)
            print("sous_pince : ",sous_pince)

            x_centre_gm = (Liste_zones_gm_coins[strategie_en_cours][0][0]+Liste_zones_gm_coins[strategie_en_cours][1][0])/2
            y_centre_gm = (Liste_zones_gm_coins[strategie_en_cours][0][1]+Liste_zones_gm_coins[strategie_en_cours][1][1])/2
            print(x_centre_gm)
            print(y_centre_gm)

            Strat_Noisettes_dans_GM = []
            for Noisette in Liste_noisette_xya:
                x_centre, y_centre, angle = Noisette[0], Noisette[1], Noisette[2]
    
                # Dimensions
                longueur = 150  # mm (dans la direction de l'angle)
                largeur = 50    # mm (perpendiculaire à l'angle)
                
                # Conversion angle en radians
                angle_rad = math.radians(angle)
                
                # Vecteurs directeurs
                dx_long = (longueur / 2) * math.cos(angle_rad)
                dy_long = (longueur / 2) * math.sin(angle_rad)
                dx_larg = (largeur / 2) * math.sin(angle_rad)  # Perpendiculaire = rotation de 90°
                dy_larg = -(largeur / 2) * math.cos(angle_rad)
                
                # Calcul des 4 coins (sens trigonométrique depuis le centre)
                Noisette_coin_hg = (x_centre - dx_long - dx_larg, y_centre - dy_long - dy_larg)  # Haut-Gauche
                Noisette_coin_hd = (x_centre + dx_long - dx_larg, y_centre + dy_long - dy_larg)  # Haut-Droite
                Noisette_coin_bd = (x_centre + dx_long + dx_larg, y_centre + dy_long + dy_larg)  # Bas-Droite
                Noisette_coin_bg = (x_centre - dx_long + dx_larg, y_centre - dy_long + dy_larg)  # Bas-Gauche
    
                 # Vérifier si AU MOINS UN coin est dans une zone GM
                zone = Liste_zones_gm_coins[strategie_en_cours]
                x_min, y_min = zone[0]
                x_max, y_max = zone[1]
                
                # Liste des 4 coins
                coins = [Noisette_coin_hg, Noisette_coin_hd, Noisette_coin_bd, Noisette_coin_bg]
                
                # Vérifier si au moins un coin est dans la zone
                if any(x_min <= coin[0] <= x_max and y_min <= coin[1] <= y_max for coin in coins):
                    Strat_Noisettes_dans_GM.append(Noisette)
            if Debug_Action:
                print("Strat_Noisettes_dans_GM : ",Strat_Noisettes_dans_GM) 

            # Si Noisette dans GM :
            #   Si Noisette à gauche ET à droite du centre :
            #       demande nouvelle strats
            #   Sinon
            #       Trouver la Noisette la plus proche du centre du GM
            #       Se positionner (avec Astar) devant la Noisette
            #       Relacher
            #       Reculer assez pour au cas où nouvelle Noisette à déposer
            # Sinon :
            #   Trouver le centre du côté d'arrivée
            #   Se positionner (avec Astar) devant le bord
            #   Relacher
            #   Reculer assez pour au cas où nouvelle Noisette à déposer
            if Strat_Noisettes_dans_GM != []:
                if len(Strat_Noisettes_dans_GM)==3 and sous_pince == 12:
                    demande_nouvelle_strat = True
                
                
            else:
                angle_robot_gm = np.degrees(math.atan2(y_centre_gm - y_robot_actuel, x_centre_gm - x_robot_actuel))
                angle_robot_gm += 360
                angle_robot_gm %= 360
                print("angle_robot_gm : ",angle_robot_gm)
                if 45<=angle_robot_gm<135:
                    print("haut")
                    x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                    y_cote = Liste_zones_gm_coins[num_gm][1][1]
                if 0<=angle_robot_gm<45 or 315<=angle_robot_gm<360:
                    print("droite")
                    x_cote = Liste_zones_gm_coins[num_gm][1][0]
                    y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                if 135<=angle_robot_gm<225:
                    print("gauche")
                    x_cote = Liste_zones_gm_coins[num_gm][0][0]
                    y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                if 225<=angle_robot_gm<315:
                    print("bas")
                    x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                    y_cote = Liste_zones_gm_coins[num_gm][0][1]

                print("x_cote : ",x_cote,"  y_cote : ",y_cote)
                if sous_pince == 12 or sous_pince == 2:
                    angle_centre_cote = math.atan2(y_centre_gm - y_cote, x_centre_gm - x_cote)
                    print("angle_centre_cote : ",np.degrees(angle_centre_cote))
                    distance = 120 + MARGE_NOISETTE + LONGUEUR_ROBOT/2
                    x_arrivee_1 = x_cote + distance*math.cos(angle_centre_cote)
                    y_arrivee_1 = y_cote + distance*math.sin(angle_centre_cote)
                    x_arrivee_Astar = x_cote + (distance+70)*math.cos(angle_centre_cote)
                    y_arrivee_Astar = y_cote + (distance+70)*math.sin(angle_centre_cote)

                    distance_point_noisette_min = 100000
                    Noisette_la_plus_proche = []
                    for Noisette in Liste_noisette_xya:
                        distance_point_noisette = math.sqrt((x_arrivee_1 - Noisette[0])**2 + (y_arrivee_1 - Noisette[1])**2)
                        if distance_point_noisette <= distance_point_noisette_min:
                            distance_point_noisette_min = distance_point_noisette
                            Noisette_la_plus_proche = Noisette
                    print("Noisette_la_plus_proche : ",Noisette_la_plus_proche)
                    print("distance_point_noisette_min : ",distance_point_noisette_min)
                    print("R_ROBOT + 2*MARGE_NOISETTE : ",R_ROBOT + 2*MARGE_NOISETTE)
                    robot_hors_piste = ((600-(R_ROBOT+MARGE_TRAJECTOIRE) <= x_arrivee_1 <= 2400+(R_ROBOT+MARGE_TRAJECTOIRE)) and (1550 -(R_ROBOT+MARGE_TRAJECTOIRE)<= y_arrivee_1 <= Y_PISTE)) or not((R_ROBOT <= x_arrivee_1 <= X_PISTE - (R_ROBOT)) and (R_ROBOT <= y_arrivee_1 <= Y_PISTE - (R_ROBOT)))
                    print("robot_hors_piste : ",robot_hors_piste)
                    if distance_point_noisette_min <= R_ROBOT + 2*MARGE_NOISETTE or robot_hors_piste:
                        print("Changer point")
                        angle_robot_gm -= 90
                        angle_robot_gm %= 360
                        print("angle_robot_gm : ",angle_robot_gm)
                        if 45<=angle_robot_gm<135:
                            print("haut")
                            x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                            y_cote = Liste_zones_gm_coins[num_gm][1][1]
                        if 0<=angle_robot_gm<45 or 315<=angle_robot_gm<360:
                            print("droite")
                            x_cote = Liste_zones_gm_coins[num_gm][1][0]
                            y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                        if 135<=angle_robot_gm<225:
                            print("gauche")
                            x_cote = Liste_zones_gm_coins[num_gm][0][0]
                            y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                        if 225<=angle_robot_gm<315:
                            print("bas")
                            x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                            y_cote = Liste_zones_gm_coins[num_gm][0][1]

                        print("x_cote : ",x_cote,"  y_cote : ",y_cote)
                        angle_centre_cote = math.atan2(y_centre_gm - y_cote, x_centre_gm - x_cote)
                        print("angle_centre_cote : ",np.degrees(angle_centre_cote))
                        distance = 120 + MARGE_NOISETTE + LONGUEUR_ROBOT/2
                        x_arrivee_1 = x_cote + distance*math.cos(angle_centre_cote)
                        y_arrivee_1 = y_cote + distance*math.sin(angle_centre_cote)
                        x_arrivee_Astar = x_cote + (distance+70)*math.cos(angle_centre_cote)
                        y_arrivee_Astar = y_cote + (distance+70)*math.sin(angle_centre_cote)

                        distance_point_noisette_min = 100000
                        Noisette_la_plus_proche = []
                        for Noisette in Liste_noisette_xya:
                            distance_point_noisette = math.sqrt((x_arrivee_1 - Noisette[0])**2 + (y_arrivee_1 - Noisette[1])**2)
                            if distance_point_noisette <= distance_point_noisette_min:
                                distance_point_noisette_min = distance_point_noisette
                                Noisette_la_plus_proche = Noisette
                        print("Noisette_la_plus_proche : ",Noisette_la_plus_proche)
                        print("distance_point_noisette_min : ",distance_point_noisette_min)
                        print("R_ROBOT + 2*MARGE_NOISETTE : ",R_ROBOT + 2*MARGE_NOISETTE)
                        robot_hors_piste = ((600-(R_ROBOT+MARGE_TRAJECTOIRE) <= x_arrivee_1 <= 2400+(R_ROBOT+MARGE_TRAJECTOIRE)) and (1550 -(R_ROBOT+MARGE_TRAJECTOIRE)<= y_arrivee_1 <= Y_PISTE)) or not((R_ROBOT <= x_arrivee_1 <= X_PISTE - (R_ROBOT)) and (R_ROBOT <= y_arrivee_1 <= Y_PISTE - (R_ROBOT)))
                        print("robot_hors_piste : ",robot_hors_piste)
                        if distance_point_noisette_min <= R_ROBOT + 2*MARGE_NOISETTE or robot_hors_piste:
                            print("Changer point")
                            angle_robot_gm -= 90
                            angle_robot_gm %= 360
                            print(angle_robot_gm)
                            if 45<=angle_robot_gm<135:
                                print("haut")
                                x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                                y_cote = Liste_zones_gm_coins[num_gm][1][1]
                            if 0<=angle_robot_gm<45 or 315<=angle_robot_gm<360:
                                print("droite")
                                x_cote = Liste_zones_gm_coins[num_gm][1][0]
                                y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                            if 135<=angle_robot_gm<225:
                                print("gauche")
                                x_cote = Liste_zones_gm_coins[num_gm][0][0]
                                y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                            if 225<=angle_robot_gm<315:
                                print("bas")
                                x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                                y_cote = Liste_zones_gm_coins[num_gm][0][1]

                            print("x_cote : ",x_cote,"  y_cote : ",y_cote)
                            angle_centre_cote = math.atan2(y_centre_gm - y_cote, x_centre_gm - x_cote)
                            print("angle_centre_cote : ",np.degrees(angle_centre_cote))
                            distance = 120 + MARGE_NOISETTE + LONGUEUR_ROBOT/2
                            x_arrivee_1 = x_cote + distance*math.cos(angle_centre_cote)
                            y_arrivee_1 = y_cote + distance*math.sin(angle_centre_cote)
                            x_arrivee_Astar = x_cote + (distance+70)*math.cos(angle_centre_cote)
                            y_arrivee_Astar = y_cote + (distance+70)*math.sin(angle_centre_cote)

                            distance_point_noisette_min = 100000
                            Noisette_la_plus_proche = []
                            for Noisette in Liste_noisette_xya:
                                distance_point_noisette = math.sqrt((x_arrivee_1 - Noisette[0])**2 + (y_arrivee_1 - Noisette[1])**2)
                                if distance_point_noisette <= distance_point_noisette_min:
                                    distance_point_noisette_min = distance_point_noisette
                                    Noisette_la_plus_proche = Noisette
                            robot_hors_piste = ((600-(R_ROBOT+MARGE_TRAJECTOIRE) <= x_arrivee_1 <= 2400+(R_ROBOT+MARGE_TRAJECTOIRE)) and (1550 -(R_ROBOT+MARGE_TRAJECTOIRE)<= y_arrivee_1 <= Y_PISTE)) or not((R_ROBOT <= x_arrivee_1 <= X_PISTE - (R_ROBOT)) and (R_ROBOT <= y_arrivee_1 <= Y_PISTE - (R_ROBOT)))
                            print("robot_hors_piste : ",robot_hors_piste)
                            print("Noisette_la_plus_proche : ",Noisette_la_plus_proche)
                            print("distance_point_noisette_min : ",distance_point_noisette_min)
                            print("R_ROBOT + 2*MARGE_NOISETTE : ",R_ROBOT + 2*MARGE_NOISETTE)
                            if distance_point_noisette_min <= R_ROBOT + 2*MARGE_NOISETTE or robot_hors_piste:
                                print("Changer point")
                                angle_robot_gm -= 90
                                angle_robot_gm %= 360
                                print(angle_robot_gm)
                                if 45<=angle_robot_gm<135:
                                    print("haut")
                                    x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                                    y_cote = Liste_zones_gm_coins[num_gm][1][1]
                                if 0<=angle_robot_gm<45 or 315<=angle_robot_gm<360:
                                    print("droite")
                                    x_cote = Liste_zones_gm_coins[num_gm][1][0]
                                    y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                                if 135<=angle_robot_gm<225:
                                    print("gauche")
                                    x_cote = Liste_zones_gm_coins[num_gm][0][0]
                                    y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                                if 225<=angle_robot_gm<315:
                                    print("bas")
                                    x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                                    y_cote = Liste_zones_gm_coins[num_gm][0][1]
                                print("x_cote : ",x_cote,"  y_cote : ",y_cote)
                                angle_centre_cote = math.atan2(y_centre_gm - y_cote, x_centre_gm - x_cote)
                                print("angle_centre_cote : ",np.degrees(angle_centre_cote))
                                distance = 120 + MARGE_NOISETTE + LONGUEUR_ROBOT/2
                                x_arrivee_1 = x_cote + distance*math.cos(angle_centre_cote)
                                y_arrivee_1 = y_cote + distance*math.sin(angle_centre_cote)
                                x_arrivee_Astar = x_cote + (distance+70)*math.cos(angle_centre_cote)
                                y_arrivee_Astar = y_cote + (distance+70)*math.sin(angle_centre_cote)

                                distance_point_noisette_min = 100000
                                Noisette_la_plus_proche = []
                                for Noisette in Liste_noisette_xya:
                                    distance_point_noisette = math.sqrt((x_arrivee_1 - Noisette[0])**2 + (y_arrivee_1 - Noisette[1])**2)
                                    if distance_point_noisette <= distance_point_noisette_min:
                                        distance_point_noisette_min = distance_point_noisette
                                        Noisette_la_plus_proche = Noisette
                                robot_hors_piste = ((600-(R_ROBOT+MARGE_TRAJECTOIRE) <= x_arrivee_1 <= 2400+(R_ROBOT+MARGE_TRAJECTOIRE)) and (1550 -(R_ROBOT+MARGE_TRAJECTOIRE)<= y_arrivee_1 <= Y_PISTE)) or not((R_ROBOT <= x_arrivee_1 <= X_PISTE - (R_ROBOT)) and (R_ROBOT <= y_arrivee_1 <= Y_PISTE - (R_ROBOT)))
                                print("robot_hors_piste : ",robot_hors_piste)
                                print("Noisette_la_plus_proche : ",Noisette_la_plus_proche)
                                if distance_point_noisette_min <= R_ROBOT + 2*MARGE_NOISETTE or robot_hors_piste:
                                    demande_nouvelle_strat = True
                                    print("DEMANDE NOUVELLE STRAT")
                                else : 
                                    if pince_a_utilise == 0:
                                        Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Relacher",pince_a_utilise,sous_pince],["ReculerPrecis",x_arrivee_Astar,y_arrivee_Astar]]
                                    else :
                                        Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote))],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote))],["Relacher",pince_a_utilise,sous_pince],["Consigne",x_arrivee_Astar,y_arrivee_Astar]]
                            else : 
                                if pince_a_utilise == 0:
                                    Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Relacher",pince_a_utilise,sous_pince],["ReculerPrecis",x_arrivee_Astar,y_arrivee_Astar]]
                                else :
                                    Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote))],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote))],["Relacher",pince_a_utilise,sous_pince],["Consigne",x_arrivee_Astar,y_arrivee_Astar]]
                        else : 
                            if pince_a_utilise == 0:
                                Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Relacher",pince_a_utilise,sous_pince],["ReculerPrecis",x_arrivee_Astar,y_arrivee_Astar]]
                            else :
                                Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote))],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote))],["Relacher",pince_a_utilise,sous_pince],["Consigne",x_arrivee_Astar,y_arrivee_Astar]]

                    if pince_a_utilise == 0:
                        Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Relacher",pince_a_utilise,sous_pince],["ReculerPrecis",x_arrivee_Astar,y_arrivee_Astar]]
                    else :
                        Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote))],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote))],["Relacher",pince_a_utilise,sous_pince],["Consigne",x_arrivee_Astar,y_arrivee_Astar]]
                elif sous_pince == 1: 
                    angle_centre_cote = math.atan2(y_centre_gm - y_cote, x_centre_gm - x_cote)
                    print("angle_centre_cote : ",np.degrees(angle_centre_cote))
                    distance = 75 + MARGE_NOISETTE + LONGUEUR_ROBOT/2
                    x_arrivee_1 = x_cote + distance*math.cos(angle_centre_cote)
                    y_arrivee_1 = y_cote + distance*math.sin(angle_centre_cote)
                    x_arrivee_Astar = x_cote + (distance+75)*math.cos(angle_centre_cote)
                    y_arrivee_Astar = y_cote + (distance+75)*math.sin(angle_centre_cote)

                    distance_point_noisette_min = 100000
                    Noisette_la_plus_proche = []
                    for Noisette in Liste_noisette_xya:
                        distance_point_noisette = math.sqrt((x_arrivee_1 - Noisette[0])**2 + (y_arrivee_1 - Noisette[1])**2)
                        if distance_point_noisette <= distance_point_noisette_min:
                            distance_point_noisette_min = distance_point_noisette
                            Noisette_la_plus_proche = Noisette
                    print("Noisette_la_plus_proche : ",Noisette_la_plus_proche)
                    print("distance_point_noisette_min : ",distance_point_noisette_min)
                    print("R_ROBOT + 2*MARGE_NOISETTE : ",R_ROBOT + 2*MARGE_NOISETTE)
                    robot_hors_piste = ((600-(R_ROBOT+MARGE_TRAJECTOIRE) <= x_arrivee_1 <= 2400+(R_ROBOT+MARGE_TRAJECTOIRE)) and (1550 -(R_ROBOT+MARGE_TRAJECTOIRE)<= y_arrivee_1 <= Y_PISTE)) or not((R_ROBOT <= x_arrivee_1 <= X_PISTE - (R_ROBOT)) and (R_ROBOT <= y_arrivee_1 <= Y_PISTE - (R_ROBOT)))
                    print("robot_hors_piste : ",robot_hors_piste)
                    if distance_point_noisette_min <= R_ROBOT + 2*MARGE_NOISETTE or robot_hors_piste:
                        print("Changer point")
                        angle_robot_gm -= 90
                        angle_robot_gm %= 360
                        print("angle_robot_gm : ",angle_robot_gm)
                        if 45<=angle_robot_gm<135:
                            print("haut")
                            x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                            y_cote = Liste_zones_gm_coins[num_gm][1][1]
                        if 0<=angle_robot_gm<45 or 315<=angle_robot_gm<360:
                            print("droite")
                            x_cote = Liste_zones_gm_coins[num_gm][1][0]
                            y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                        if 135<=angle_robot_gm<225:
                            print("gauche")
                            x_cote = Liste_zones_gm_coins[num_gm][0][0]
                            y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                        if 225<=angle_robot_gm<315:
                            print("bas")
                            x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                            y_cote = Liste_zones_gm_coins[num_gm][0][1]

                        print("x_cote : ",x_cote,"  y_cote : ",y_cote)
                        angle_centre_cote = math.atan2(y_centre_gm - y_cote, x_centre_gm - x_cote)
                        print("angle_centre_cote : ",np.degrees(angle_centre_cote))
                        distance = 75 + MARGE_NOISETTE + LONGUEUR_ROBOT/2
                        x_arrivee_1 = x_cote + distance*math.cos(angle_centre_cote)
                        y_arrivee_1 = y_cote + distance*math.sin(angle_centre_cote)
                        x_arrivee_Astar = x_cote + (distance+75)*math.cos(angle_centre_cote)
                        y_arrivee_Astar = y_cote + (distance+75)*math.sin(angle_centre_cote)

                        distance_point_noisette_min = 100000
                        Noisette_la_plus_proche = []
                        for Noisette in Liste_noisette_xya:
                            distance_point_noisette = math.sqrt((x_arrivee_1 - Noisette[0])**2 + (y_arrivee_1 - Noisette[1])**2)
                            if distance_point_noisette <= distance_point_noisette_min:
                                distance_point_noisette_min = distance_point_noisette
                                Noisette_la_plus_proche = Noisette
                        print("Noisette_la_plus_proche : ",Noisette_la_plus_proche)
                        print("distance_point_noisette_min : ",distance_point_noisette_min)
                        print("R_ROBOT + 2*MARGE_NOISETTE : ",R_ROBOT + 2*MARGE_NOISETTE)
                        robot_hors_piste = ((600-(R_ROBOT+MARGE_TRAJECTOIRE) <= x_arrivee_1 <= 2400+(R_ROBOT+MARGE_TRAJECTOIRE)) and (1550 -(R_ROBOT+MARGE_TRAJECTOIRE)<= y_arrivee_1 <= Y_PISTE)) or not((R_ROBOT <= x_arrivee_1 <= X_PISTE - (R_ROBOT)) and (R_ROBOT <= y_arrivee_1 <= Y_PISTE - (R_ROBOT)))
                        print("robot_hors_piste : ",robot_hors_piste)
                        if distance_point_noisette_min <= R_ROBOT + 2*MARGE_NOISETTE or robot_hors_piste:
                            print("Changer point")
                            angle_robot_gm -= 90
                            angle_robot_gm %= 360
                            print(angle_robot_gm)
                            if 45<=angle_robot_gm<135:
                                print("haut")
                                x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                                y_cote = Liste_zones_gm_coins[num_gm][1][1]
                            if 0<=angle_robot_gm<45 or 315<=angle_robot_gm<360:
                                print("droite")
                                x_cote = Liste_zones_gm_coins[num_gm][1][0]
                                y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                            if 135<=angle_robot_gm<225:
                                print("gauche")
                                x_cote = Liste_zones_gm_coins[num_gm][0][0]
                                y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                            if 225<=angle_robot_gm<315:
                                print("bas")
                                x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                                y_cote = Liste_zones_gm_coins[num_gm][0][1]

                            print("x_cote : ",x_cote,"  y_cote : ",y_cote)
                            angle_centre_cote = math.atan2(y_centre_gm - y_cote, x_centre_gm - x_cote)
                            print("angle_centre_cote : ",np.degrees(angle_centre_cote))
                            distance = 75 + MARGE_NOISETTE + LONGUEUR_ROBOT/2
                            x_arrivee_1 = x_cote + distance*math.cos(angle_centre_cote)
                            y_arrivee_1 = y_cote + distance*math.sin(angle_centre_cote)
                            x_arrivee_Astar = x_cote + (distance+75)*math.cos(angle_centre_cote)
                            y_arrivee_Astar = y_cote + (distance+75)*math.sin(angle_centre_cote)

                            distance_point_noisette_min = 100000
                            Noisette_la_plus_proche = []
                            for Noisette in Liste_noisette_xya:
                                distance_point_noisette = math.sqrt((x_arrivee_1 - Noisette[0])**2 + (y_arrivee_1 - Noisette[1])**2)
                                if distance_point_noisette <= distance_point_noisette_min:
                                    distance_point_noisette_min = distance_point_noisette
                                    Noisette_la_plus_proche = Noisette
                            robot_hors_piste = ((600-(R_ROBOT+MARGE_TRAJECTOIRE) <= x_arrivee_1 <= 2400+(R_ROBOT+MARGE_TRAJECTOIRE)) and (1550 -(R_ROBOT+MARGE_TRAJECTOIRE)<= y_arrivee_1 <= Y_PISTE)) or not((R_ROBOT <= x_arrivee_1 <= X_PISTE - (R_ROBOT)) and (R_ROBOT <= y_arrivee_1 <= Y_PISTE - (R_ROBOT)))
                            print("robot_hors_piste : ",robot_hors_piste)
                            print("Noisette_la_plus_proche : ",Noisette_la_plus_proche)
                            print("distance_point_noisette_min : ",distance_point_noisette_min)
                            print("R_ROBOT + 2*MARGE_NOISETTE : ",R_ROBOT + 2*MARGE_NOISETTE)
                            if distance_point_noisette_min <= R_ROBOT + 2*MARGE_NOISETTE or robot_hors_piste:
                                print("Changer point")
                                angle_robot_gm -= 90
                                angle_robot_gm %= 360
                                print(angle_robot_gm)
                                if 45<=angle_robot_gm<135:
                                    print("haut")
                                    x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                                    y_cote = Liste_zones_gm_coins[num_gm][1][1]
                                if 0<=angle_robot_gm<45 or 315<=angle_robot_gm<360:
                                    print("droite")
                                    x_cote = Liste_zones_gm_coins[num_gm][1][0]
                                    y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                                if 135<=angle_robot_gm<225:
                                    print("gauche")
                                    x_cote = Liste_zones_gm_coins[num_gm][0][0]
                                    y_cote = (Liste_zones_gm_coins[num_gm][0][1]+Liste_zones_gm_coins[num_gm][1][1])/2
                                if 225<=angle_robot_gm<315:
                                    print("bas")
                                    x_cote = (Liste_zones_gm_coins[num_gm][0][0]+Liste_zones_gm_coins[num_gm][1][0])/2
                                    y_cote = Liste_zones_gm_coins[num_gm][0][1]
                                print("x_cote : ",x_cote,"  y_cote : ",y_cote)
                                angle_centre_cote = math.atan2(y_centre_gm - y_cote, x_centre_gm - x_cote)
                                print("angle_centre_cote : ",np.degrees(angle_centre_cote))
                                distance = 75 + MARGE_NOISETTE + LONGUEUR_ROBOT/2
                                x_arrivee_1 = x_cote + distance*math.cos(angle_centre_cote)
                                y_arrivee_1 = y_cote + distance*math.sin(angle_centre_cote)
                                x_arrivee_Astar = x_cote + (distance+75)*math.cos(angle_centre_cote)
                                y_arrivee_Astar = y_cote + (distance+75)*math.sin(angle_centre_cote)

                                distance_point_noisette_min = 100000
                                Noisette_la_plus_proche = []
                                for Noisette in Liste_noisette_xya:
                                    distance_point_noisette = math.sqrt((x_arrivee_1 - Noisette[0])**2 + (y_arrivee_1 - Noisette[1])**2)
                                    if distance_point_noisette <= distance_point_noisette_min:
                                        distance_point_noisette_min = distance_point_noisette
                                        Noisette_la_plus_proche = Noisette
                                robot_hors_piste = ((600-(R_ROBOT+MARGE_TRAJECTOIRE) <= x_arrivee_1 <= 2400+(R_ROBOT+MARGE_TRAJECTOIRE)) and (1550 -(R_ROBOT+MARGE_TRAJECTOIRE)<= y_arrivee_1 <= Y_PISTE)) or not((R_ROBOT <= x_arrivee_1 <= X_PISTE - (R_ROBOT)) and (R_ROBOT <= y_arrivee_1 <= Y_PISTE - (R_ROBOT)))
                                print("robot_hors_piste : ",robot_hors_piste)
                                print("Noisette_la_plus_proche : ",Noisette_la_plus_proche)
                                if distance_point_noisette_min <= R_ROBOT + 2*MARGE_NOISETTE or robot_hors_piste:
                                    demande_nouvelle_strat = True
                                    print("DEMANDE NOUVELLE STRAT")
                                else : 
                                    if pince_a_utilise == 0:
                                        Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Relacher",pince_a_utilise,sous_pince],["ReculerPrecis",x_arrivee_Astar,y_arrivee_Astar]]
                                    else :
                                        Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote))],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote))],["Relacher",pince_a_utilise,sous_pince],["Consigne",x_arrivee_Astar,y_arrivee_Astar]]
                            else : 
                                if pince_a_utilise == 0:
                                    Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Relacher",pince_a_utilise,sous_pince],["ReculerPrecis",x_arrivee_Astar,y_arrivee_Astar]]
                                else :
                                    Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote))],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote))],["Relacher",pince_a_utilise,sous_pince],["Consigne",x_arrivee_Astar,y_arrivee_Astar]]
                        else : 
                            if pince_a_utilise == 0:
                                Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Relacher",pince_a_utilise,sous_pince],["ReculerPrecis",x_arrivee_Astar,y_arrivee_Astar]]
                            else :
                                Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote))],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote))],["Relacher",pince_a_utilise,sous_pince],["Consigne",x_arrivee_Astar,y_arrivee_Astar]]

                    if pince_a_utilise == 0:
                        Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote)-180)],["Relacher",pince_a_utilise,sous_pince],["ReculerPrecis",x_arrivee_Astar,y_arrivee_Astar]]
                    else :
                        Liste_actions = [["Consigne",x_arrivee_Astar,y_arrivee_Astar],["Rotation",int(np.degrees(angle_centre_cote))],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",int(np.degrees(angle_centre_cote))],["Relacher",pince_a_utilise,sous_pince],["Consigne",x_arrivee_Astar,y_arrivee_Astar]]     
    """

    Liste_actions.append(["Attente"])
    return Liste_actions

def positionner_robot_devant_Noisette(x_strategie,y_strategie,Noisettes_groupees,strategie_en_cours,demande_nouvelle_strat,Liste_actions,couleur,Liste_zones_gm_coins,TOLERANCE_STRATEGIE_NOISETTE,Noisettes_stockees_dans_robot,Debug_strategie,MARGE_NOISETTE,LONGUEUR_ROBOT,Liste_noisette_xya,x_robot_actuel,y_robot_actuel,Pince_Avant, Pince_Av_1, Pince_Av_2,Pince_Arriere, Pince_Ar_1, Pince_Ar_2,width,height,CASE_MM,grid_expanded):

    Liste_actions.clear()
    chercher_Noisette = None

    for num_gm in range(len(Liste_zones_gm_coins)):
        if Liste_zones_gm_coins[num_gm][0][0]<= x_strategie <= Liste_zones_gm_coins[num_gm][1][0] and Liste_zones_gm_coins[num_gm][0][1]<= y_strategie <= Liste_zones_gm_coins[num_gm][1][1]:
            strategie_en_cours = num_gm
            chercher_Noisette = False
            break
    
    for grpNoisette in Noisettes_groupees:
        nbr_Noisette = len(grpNoisette)
        x_centre = 0
        y_centre = 0
        angle_groupe = 0
        for num_Noisette in range(nbr_Noisette):
            x_centre += grpNoisette[num_Noisette][0]
            y_centre += grpNoisette[num_Noisette][1]
            angle_groupe += grpNoisette[num_Noisette][2]
        x_centre/=nbr_Noisette
        y_centre/=nbr_Noisette
        angle_groupe /=nbr_Noisette
        distance_Noisette_strategie = math.sqrt((x_strategie - x_centre)**2 + (y_strategie - y_centre)**2)
        if distance_Noisette_strategie <= TOLERANCE_STRATEGIE_NOISETTE:
            strategie_en_cours = grpNoisette
            chercher_Noisette = True
            break
    
    if chercher_Noisette == True:
        pince_a_utilise = None
        sous_pince = None
        print(strategie_en_cours)
        if len(strategie_en_cours)==1:
            angle_noisette = strategie_en_cours[0][2]

            if Noisettes_stockees_dans_robot[0]!=["J","B"] and Noisettes_stockees_dans_robot[0]!=["B","J"] and Noisettes_stockees_dans_robot[0]!=["B","B"] and Noisettes_stockees_dans_robot[0]!=["J","J"] and (Pince_Av_1 or Pince_Av_2):
                pince_a_utilise = 0
                if Noisettes_stockees_dans_robot[0][0]=="N" and Pince_Av_1:
                    sous_pince = 1
                elif Noisettes_stockees_dans_robot[0][1]=="N" and Pince_Av_2:
                    sous_pince = 2
                else:
                    demande_nouvelle_strat = True
            elif Noisettes_stockees_dans_robot[1]!=["J","B"] and Noisettes_stockees_dans_robot[1]!=["B","J"] and Noisettes_stockees_dans_robot[1]!=["B","B"] and Noisettes_stockees_dans_robot[1]!=["J","J"] and (Pince_Ar_1 or Pince_Ar_2):
                pince_a_utilise = 1
                if Noisettes_stockees_dans_robot[1][0]=="N" and Pince_Ar_1:
                    sous_pince = 1
                elif Noisettes_stockees_dans_robot[1][1]=="N" and Pince_Ar_2:
                    sous_pince = 2
                else : demande_nouvelle_strat = True
            else:
                demande_nouvelle_strat = True

            if Debug_strategie:
                print("pince_a_utilise : ",pince_a_utilise)
                print("sous_pince : ",sous_pince)
            point_1_bloquee = False
            point_2_bloquee = False

            distance = 50*(sous_pince-1) + 25 + MARGE_NOISETTE + LONGUEUR_ROBOT/2
            angle_rad1 = math.radians(angle_noisette + 90)
            angle_rad2 = math.radians(angle_noisette + 90 - 180)
            
            x_arrivee_1 = strategie_en_cours[0][0]+distance*math.cos(angle_rad1)
            y_arrivee_1 = strategie_en_cours[0][1]+distance*math.sin(angle_rad1)
            x_arrivee_2 = strategie_en_cours[0][0]+distance*math.cos(angle_rad2)
            y_arrivee_2 = strategie_en_cours[0][1]+distance*math.sin(angle_rad2)
        
            distance = 50*(sous_pince-1) + 25 + 6*MARGE_NOISETTE+LONGUEUR_ROBOT/2
            x_arrivee_Astar1 = strategie_en_cours[0][0]+distance*math.cos(angle_rad1)
            y_arrivee_Astar1 = strategie_en_cours[0][1]+distance*math.sin(angle_rad1)
            x_arrivee_Astar2 = strategie_en_cours[0][0]+distance*math.cos(angle_rad2)
            y_arrivee_Astar2 = strategie_en_cours[0][1]+distance*math.sin(angle_rad2)
            Liste_actions = [["Consigne",x_arrivee_Astar2,y_arrivee_Astar2],["Consigne",x_arrivee_2,y_arrivee_2],["Consigne",x_arrivee_Astar1,y_arrivee_Astar1],["Consigne",x_arrivee_1,y_arrivee_1],]

            if grid_expanded[max(0, min(width - 1, int(x_arrivee_Astar1 // CASE_MM))), max(0, min(height - 1, int(y_arrivee_Astar1 // CASE_MM)))]:
                point_1_bloquee = True
            if grid_expanded[max(0, min(width - 1, int(x_arrivee_Astar2 // CASE_MM))), max(0, min(height - 1, int(y_arrivee_Astar2 // CASE_MM)))]:
                point_2_bloquee = True

            if point_1_bloquee and point_2_bloquee:
                demande_nouvelle_strat = True
            else:
                if point_2_bloquee:
                    if pince_a_utilise == 0:
                        Liste_actions = [["Consigne",x_arrivee_Astar1,y_arrivee_Astar1],["Rotation",180-90+angle_noisette],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",180-90+angle_noisette]]
                    else:
                        Liste_actions = [["ReculerPrecis",x_arrivee_Astar1,y_arrivee_Astar1],["Rotation",-90+angle_noisette],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",-90+angle_noisette]]
                    
                elif point_1_bloquee:
                    if pince_a_utilise == 0:
                        Liste_actions = [["Consigne",x_arrivee_Astar2,y_arrivee_Astar2],["Rotation",180-90+angle_noisette],["Consigne",x_arrivee_2,y_arrivee_2],["Rotation",180-90+angle_noisette]]
                    else:
                        Liste_actions = [["ReculerPrecis",x_arrivee_Astar2,y_arrivee_Astar2],["Rotation",-90+angle_noisette],["ReculerPrecis",x_arrivee_2,y_arrivee_2],["Rotation",-90+angle_noisette]]
                    
                else:
                    distance_robot_point1 = math.sqrt((x_arrivee_1 - x_robot_actuel)**2 + (y_arrivee_1 - y_robot_actuel)**2)
                    distance_robot_point2 = math.sqrt((x_arrivee_2 - x_robot_actuel)**2 + (y_arrivee_2 - y_robot_actuel)**2)
                    if distance_robot_point1 <= distance_robot_point2:
                        if pince_a_utilise == 0:
                            Liste_actions = [["Consigne",x_arrivee_Astar1,y_arrivee_Astar1],["Rotation",180-90+angle_noisette],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",180-90+angle_noisette]]
                        else:
                            Liste_actions = [["ReculerPrecis",x_arrivee_Astar1,y_arrivee_Astar1],["Rotation",-90+angle_noisette],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",-90+angle_noisette]]
                    else:
                        if pince_a_utilise == 0:
                            Liste_actions = [["Consigne",x_arrivee_Astar2,y_arrivee_Astar2],["Rotation",180-90+angle_noisette],["Consigne",x_arrivee_2,y_arrivee_2],["Rotation",180-90+angle_noisette]]
                        else:
                            Liste_actions = [["ReculerPrecis",x_arrivee_Astar2,y_arrivee_Astar2],["Rotation",-90+angle_noisette],["ReculerPrecis",x_arrivee_2,y_arrivee_2],["Rotation",-90+angle_noisette]]
                Liste_actions.append(["Attraper",pince_a_utilise,sous_pince])
                if (pince_a_utilise == 0 and Pince_Avant) or (pince_a_utilise == 1 and Pince_Arriere):
                    if strategie_en_cours[0][3] != "R":
                        if strategie_en_cours[0][3] != couleur:
                            Liste_actions.append(["Retourner",pince_a_utilise,sous_pince])
                    else:
                        demande_nouvelle_strat = True
        else :
            pince_a_utilise = None
            sous_pince = None
            if Noisettes_stockees_dans_robot[0]==["N","N"] and Pince_Av_1 and Pince_Av_2:
                pince_a_utilise = 0
                sous_pince = 12
            elif Noisettes_stockees_dans_robot[1]==["N","N"] and Pince_Ar_1 and Pince_Ar_2:
                pince_a_utilise = 1
                sous_pince = 12
            else:
                demande_nouvelle_strat = True

            if Debug_strategie:
                print("pince_a_utilise : ",pince_a_utilise)
                print("sous_pince : ",sous_pince)

            distance_robot_n0 = math.sqrt((x_robot_actuel - strategie_en_cours[0][0])**2 + (y_robot_actuel - strategie_en_cours[0][1])**2)
            distance_robot_n1 = math.sqrt((x_robot_actuel - strategie_en_cours[1][0])**2 + (y_robot_actuel - strategie_en_cours[1][1])**2)
            if Debug_strategie:
                print("Distance entre robot et n0 : ",distance_robot_n0)
                print("Distance entre robot et n1 : ",distance_robot_n1)
                print("Strategie avant tri : ",strategie_en_cours)
            # Tri des noisettes par distance
            if distance_robot_n0 > distance_robot_n1:
                temp = strategie_en_cours[0]
                strategie_en_cours[0] = strategie_en_cours[1]
                strategie_en_cours[1] = temp
                distance_robot_n0, distance_robot_n1 = distance_robot_n1, distance_robot_n0
            
            if Debug_strategie:
                print("Strategie après tri : ",strategie_en_cours)

            angle_noisette1 = strategie_en_cours[0][2]
            angle_noisette2 = strategie_en_cours[1][2]
            distance = 25 + MARGE_NOISETTE + LONGUEUR_ROBOT/2

            # Normaliser l'angle entre 0 et 180°
            angle_moyen = (angle_noisette1 + angle_noisette2) / 2
            angle_normalise = angle_moyen % 180

            # ⭐ CALCULER LA POSITION RELATIVE DU ROBOT PAR RAPPORT AUX NOISETTES ⭐
            x_centre_noisettes = (strategie_en_cours[0][0] + strategie_en_cours[1][0]) / 2
            y_centre_noisettes = (strategie_en_cours[0][1] + strategie_en_cours[1][1]) / 2

            # Déterminer si les noisettes sont horizontales (~0°) ou verticales (~90°)
            if 45 < angle_normalise < 135:  # Noisettes verticales (~90°)
                if Debug_strategie:
                    print("Noisettes verticales détectées")
                # Robot à gauche ou à droite des noisettes ?
                if x_robot_actuel < x_centre_noisettes:
                    # Approcher par la gauche
                    angle_rad1 = math.radians(angle_noisette1 + 90)  # Côté gauche
                    angle_rad2 = math.radians(angle_noisette2 - 90)
                else:
                    # Approcher par la droite
                    angle_rad1 = math.radians(angle_noisette1 - 90)  # Côté droit
                    angle_rad2 = math.radians(angle_noisette2 + 90)

            else:  # Noisettes horizontales (~0° ou ~180°)
                if Debug_strategie:
                    print("Noisettes horizontales détectées")
                # Robot en haut ou en bas des noisettes ?
                if y_robot_actuel < y_centre_noisettes:
                    # Approcher par le bas
                    angle_rad1 = math.radians(angle_noisette1 - 90)  # En bas
                    angle_rad2 = math.radians(angle_noisette2 + 90)
                else:
                    # Approcher par le haut
                    angle_rad1 = math.radians(angle_noisette1 + 90)  # En haut
                    angle_rad2 = math.radians(angle_noisette2 - 90)

            if Debug_strategie:
                print("angle_noisette1 : ",angle_noisette1-90)
                print("angle_noisette2 : ",angle_noisette2+90)
                print("Point1 associé à Noisette : ",strategie_en_cours[0])
                print("Point2 associé à Noisette : ",strategie_en_cours[1])

            x_arrivee_1 = strategie_en_cours[0][0]+distance*math.cos(angle_rad1)
            y_arrivee_1 = strategie_en_cours[0][1]+distance*math.sin(angle_rad1)
            x_arrivee_2 = strategie_en_cours[1][0]+distance*math.cos(angle_rad2)
            y_arrivee_2 = strategie_en_cours[1][1]+distance*math.sin(angle_rad2)

            distance = 50 + 25 + 5*MARGE_NOISETTE+LONGUEUR_ROBOT/2
            x_arrivee_Astar1 = strategie_en_cours[0][0]+distance*math.cos(angle_rad1)
            y_arrivee_Astar1 = strategie_en_cours[0][1]+distance*math.sin(angle_rad1)
            x_arrivee_Astar2 = strategie_en_cours[1][0]+distance*math.cos(angle_rad2)
            y_arrivee_Astar2 = strategie_en_cours[1][1]+distance*math.sin(angle_rad2)

            #Liste_actions = [["Consigne",x_arrivee_1,y_arrivee_1],["Consigne",x_arrivee_2,y_arrivee_2],["Consigne",x_arrivee_Astar1,y_arrivee_Astar1],["Consigne",x_arrivee_Astar2,y_arrivee_Astar2]]
            
            point_1_bloquee = False
            point_2_bloquee = False
            if grid_expanded[max(0, min(width - 1, int(x_arrivee_Astar1 // CASE_MM))), max(0, min(height - 1, int(y_arrivee_Astar1 // CASE_MM)))]:
                point_1_bloquee = True
            if grid_expanded[max(0, min(width - 1, int(x_arrivee_Astar2 // CASE_MM))), max(0, min(height - 1, int(y_arrivee_Astar2 // CASE_MM)))]:
                point_2_bloquee = True

            if point_1_bloquee and point_2_bloquee:
                demande_nouvelle_strat = True
            else:
                distance_robot_point1 = math.sqrt((x_arrivee_1 - x_robot_actuel)**2 + (y_arrivee_1 - y_robot_actuel)**2)
                distance_robot_point2 = math.sqrt((x_arrivee_2 - x_robot_actuel)**2 + (y_arrivee_2 - y_robot_actuel)**2)
                if point_2_bloquee:
                    print("aaaaaaaaa")
                    angle_pointarrivee_noisette = int(np.degrees(math.atan2(strategie_en_cours[0][1] - y_arrivee_1, strategie_en_cours[0][0] - x_arrivee_1)))
                    if pince_a_utilise == 0:
                        Liste_actions = [["Consigne",x_arrivee_Astar1,y_arrivee_Astar1],["Rotation",angle_pointarrivee_noisette],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",angle_pointarrivee_noisette]]
                    else:
                        Liste_actions = [["ReculerPrecis",x_arrivee_Astar1,y_arrivee_Astar1],["Rotation",180+angle_pointarrivee_noisette],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",180+angle_pointarrivee_noisette]]
                    
                elif point_1_bloquee:
                    print("bbbbbbbbb")
                    angle_pointarrivee_noisette = int(np.degrees(math.atan2(strategie_en_cours[1][1] - y_arrivee_2, strategie_en_cours[1][0] - x_arrivee_2)))
                    if pince_a_utilise == 0:
                        Liste_actions = [["Consigne",x_arrivee_Astar2,y_arrivee_Astar2],["Rotation",angle_pointarrivee_noisette],["Consigne",x_arrivee_2,y_arrivee_2],["Rotation",angle_pointarrivee_noisette]]
                    else:
                        Liste_actions = [["ReculerPrecis",x_arrivee_Astar2,y_arrivee_Astar2],["Rotation",180+angle_pointarrivee_noisette],["ReculerPrecis",x_arrivee_2,y_arrivee_2],["Rotation",180+angle_pointarrivee_noisette]]
                    
                else:
                    distance_robot_point1 = math.sqrt((x_arrivee_1 - x_robot_actuel)**2 + (y_arrivee_1 - y_robot_actuel)**2)
                    distance_robot_point2 = math.sqrt((x_arrivee_2 - x_robot_actuel)**2 + (y_arrivee_2 - y_robot_actuel)**2)
                    if distance_robot_point1 <= distance_robot_point2:
                        print("cccccccccc")
                        angle_pointarrivee_noisette = int(np.degrees(math.atan2(strategie_en_cours[0][1] - y_arrivee_1, strategie_en_cours[0][0] - x_arrivee_1)))
                        if pince_a_utilise == 0:
                            Liste_actions = [["Consigne",x_arrivee_Astar1,y_arrivee_Astar1],["Rotation",angle_pointarrivee_noisette],["Consigne",x_arrivee_1,y_arrivee_1],["Rotation",angle_pointarrivee_noisette]]
                        else:
                            Liste_actions = [["ReculerPrecis",x_arrivee_Astar1,y_arrivee_Astar1],["Rotation",180+angle_pointarrivee_noisette],["ReculerPrecis",x_arrivee_1,y_arrivee_1],["Rotation",180+angle_pointarrivee_noisette]]
                    else:
                        print("dddddddd")
                        angle_pointarrivee_noisette = int(np.degrees(math.atan2(strategie_en_cours[1][1] - y_arrivee_2, strategie_en_cours[1][0] - x_arrivee_2)))
                        if pince_a_utilise == 0:
                            Liste_actions = [["Consigne",x_arrivee_Astar2,y_arrivee_Astar2],["Rotation",angle_pointarrivee_noisette],["Consigne",x_arrivee_2,y_arrivee_2],["Rotation",angle_pointarrivee_noisette]]
                        else:
                            Liste_actions = [["ReculerPrecis",x_arrivee_Astar2,y_arrivee_Astar2],["Rotation",180+angle_pointarrivee_noisette],["ReculerPrecis",x_arrivee_2,y_arrivee_2],["Rotation",180+angle_pointarrivee_noisette]]
                Liste_actions.append(["Attraper",pince_a_utilise,sous_pince])
                if (pince_a_utilise == 0 and Pince_Avant) or (pince_a_utilise == 1 and Pince_Arriere):
                    if strategie_en_cours[0][3] != "R" and strategie_en_cours[1][3] != "R":
                        if point_1_bloquee:
                            if strategie_en_cours[0][3] != couleur and strategie_en_cours[1][3] == couleur:
                                Liste_actions.append(["Retourner",pince_a_utilise,2])
                            elif strategie_en_cours[0][3] == couleur and strategie_en_cours[1][3] != couleur:
                                Liste_actions.append(["Retourner",pince_a_utilise,1])
                        elif point_2_bloquee:
                            if strategie_en_cours[0][3] != couleur and strategie_en_cours[1][3] == couleur:
                                Liste_actions.append(["Retourner",pince_a_utilise,1])
                            elif strategie_en_cours[0][3] == couleur and strategie_en_cours[1][3] != couleur:
                                Liste_actions.append(["Retourner",pince_a_utilise,2])
                        else:
                            if distance_robot_point1 <= distance_robot_point2:
                                print("eeeeee")
                                if strategie_en_cours[0][3] != couleur and strategie_en_cours[1][3] != couleur:
                                    Liste_actions.append(["Retourner",pince_a_utilise,12])
                                elif strategie_en_cours[0][3] != couleur and strategie_en_cours[1][3] == couleur:
                                    Liste_actions.append(["Retourner",pince_a_utilise,1])
                                elif strategie_en_cours[0][3] == couleur and strategie_en_cours[1][3] != couleur:
                                    Liste_actions.append(["Retourner",pince_a_utilise,2])
                            else:
                                print("ffffffff")
                                if strategie_en_cours[0][3] != couleur and strategie_en_cours[1][3] != couleur:
                                    Liste_actions.append(["Retourner",pince_a_utilise,12])
                                elif strategie_en_cours[0][3] != couleur and strategie_en_cours[1][3] == couleur:
                                    Liste_actions.append(["Retourner",pince_a_utilise,2])
                                elif strategie_en_cours[0][3] == couleur and strategie_en_cours[1][3] != couleur:
                                    Liste_actions.append(["Retourner",pince_a_utilise,1])
                        
                    else:
                        demande_nouvelle_strat = True
                    

    elif chercher_Noisette == False:
        print(f"GM n°{strategie_en_cours}")
    else:
        demande_nouvelle_strat = True


    Liste_actions.append(["Attente"])
    return Liste_actions,demande_nouvelle_strat
