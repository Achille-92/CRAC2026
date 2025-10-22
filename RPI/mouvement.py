import numpy as np

def angle_diff(a1, a2):
    diff = (a1 - a2 + 180) % 360 - 180
    return abs(diff)

def verif_et_ajoute_contournement(Liste_actions, x_actuel, y_actuel, x_voulu, y_voulu, x_ennemi, y_ennemi, r_robot, r_ennemi, marge_min):
    """
    Vérifie si l'ennemi est sur la trajectoire robot->consigne.
    Si oui, insère un point de contournement dans Liste_actions.
    """
    R_securite = r_robot + r_ennemi + marge_min

    Angle_robot_consigne = round(np.atan2(y_voulu-y_actuel,x_voulu-x_actuel)*180/np.pi,0)
    Angle_robot_ennemi = round(np.atan2(y_ennemi-y_actuel,x_ennemi-x_actuel)*180/np.pi,0)
    Distance_robot_ennemi = round(np.sqrt((x_ennemi-x_actuel)**2 + (y_ennemi-y_actuel)**2),0)
    Distance_robot_consigne = round(np.sqrt((x_actuel-x_voulu)**2 + (y_actuel-y_voulu)**2),0)
 
    if Distance_robot_ennemi < R_securite :
        print("Robot trop proche de l'Ennemi")
        ratio = R_securite / Distance_robot_ennemi
        x_recul = round(x_ennemi - (x_ennemi-x_actuel) * ratio,0)
        y_recul = round(y_ennemi - (y_ennemi-y_actuel) * ratio,0)
        Liste_actions.insert(0,[x_actuel,y_actuel,Angle_robot_ennemi])
        Liste_actions.insert(0,[x_recul,y_recul,Angle_robot_ennemi])
        
    else :
        seuil_angle = np.degrees(np.arcsin(R_securite / Distance_robot_ennemi))
        if angle_diff(Angle_robot_consigne, Angle_robot_ennemi) <= seuil_angle and Distance_robot_ennemi < Distance_robot_consigne:
            print("⚠️ Trajectoire traverse la zone interdite → génération de 5 points tangents")

            # Détermination du sens de contournement (produit vectoriel)
            cross = (x_ennemi - x_actuel) * (y_voulu - y_actuel) - (y_ennemi - y_actuel) * (x_voulu - x_actuel)
            sens = 1 if cross > 0 else -1  # gauche (+1) ou droite (-1)

            # Angle du point tangent principal
            theta_centre = np.arctan2(y_ennemi - y_actuel, x_ennemi - x_actuel)
            theta_tangent = theta_centre + sens * np.pi / 2  # perpendiculaire au rayon

            # Génération de 5 points espacés de ±30° autour du point tangent
            angles = [theta_tangent + sens * np.radians(a) for a in [-40, -20, 0, 20, 40]]
            points = []

            for a in angles:
                x_c = round(x_ennemi + R_securite * np.cos(a),0)
                y_c = round(y_ennemi + R_securite * np.sin(a),0)
                points.append([x_c, y_c, round(np.degrees(a)-90,0)])

            # Tri pour que le point le plus proche du robot soit exécuté en premier
            points.sort(key=lambda p: np.hypot(p[0] - x_actuel, p[1] - y_actuel))

            # Insertion dans la liste d’actions
            Liste_actions = points + Liste_actions

        else :
            Liste_actions[0][2]=Angle_robot_consigne

    nouvelle_liste = []
    for action in Liste_actions:
        if isinstance(action, list):
            if len(action) == 3:
                # Triplet (x, y, angle)
                x, y, a = action
                nouvelle_liste.append([float(x), float(y), float(a)])
            elif len(action) == 1:
                # Simple angle
                a = action[0]
                nouvelle_liste.append([float(a)])
            else:
                # Cas inattendu
                raise ValueError(f"Action mal formée : {action}")
        elif isinstance(action, str):
            nouvelle_liste.append(action)
        else :
            raise ValueError(f"Action doit être une liste ou un str : {action}")

    Liste_actions = nouvelle_liste
    return Liste_actions
