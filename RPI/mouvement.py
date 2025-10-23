import numpy as np
import math

def angle_diff(a1, a2):
    diff = (a1 - a2 + 180) % 360 - 180
    return abs(diff)

def point_arret_perimetre(x_r, y_r, x_c, y_c, x_e, y_e, R):
    dx = x_c - x_r
    dy = y_c - y_r
    a = dx**2 + dy**2
    b = 2 * (dx*(x_r - x_e) + dy*(y_r - y_e))
    c = (x_r - x_e)**2 + (y_r - y_e)**2 - R**2
    
    delta = b**2 - 4*a*c
    if delta < 0:
        return None  # pas d'intersection
    t1 = (-b + math.sqrt(delta)) / (2*a)
    t2 = (-b - math.sqrt(delta)) / (2*a)
    
    t_candidates = [t for t in [t1, t2] if 0 <= t <= 1]
    if not t_candidates:
        return None
    
    t = min(t_candidates)  # plus proche du robot
    x_s = x_r + t*dx
    y_s = y_r + t*dy
    return x_s, y_s

def intersection_segment_cercle(x_r, y_r, x_c, y_c, x_e, y_e, R, require_on_segment=True):
    """
    Retourne (x_entry, y_entry, t_entry, x_exit, y_exit, t_exit)
    - (x_entry, y_entry) : premier point d'intersection rencontré en partant du robot (t faible -> proche robot)
    - t dans paramétrisation (0 = robot, 1 = consigne)
    - Si pas d'intersection : retourne None
    - Si tangent : entry == exit (même point)
    - require_on_segment=True : n'accepte que les intersections avec 0<=t<=1 (segment). 
      Si False, étend la droite infinie et peut retourner t en dehors [0,1].
    """
    dx = x_c - x_r
    dy = y_c - y_r
    a = dx*dx + dy*dy
    if a == 0:
        return None  # robot == consigne: pas de ligne définie

    bx = x_r - x_e
    by = y_r - y_e
    b = 2 * (dx * bx + dy * by)
    c = bx*bx + by*by - R*R

    delta = b*b - 4*a*c
    if delta < 0:
        return None  # pas d'intersection réelle

    sqrt_delta = math.sqrt(max(0.0, delta))
    t1 = (-b - sqrt_delta) / (2*a)
    t2 = (-b + sqrt_delta) / (2*a)

    # ordonner t1 <= t2
    if t1 > t2:
        t1, t2 = t2, t1

    # Si on exige le segment, filtrer
    ts = []
    for t in (t1, t2):
        if not require_on_segment or (0.0 <= t <= 1.0):
            ts.append(t)

    if not ts:
        return None

    # premier point rencontré depuis le robot = plus petit t valide
    t_entry = max(ts)
    t_exit = max(ts) if len(ts) == 2 else t_entry

    x_entry = x_r + t_entry * dx
    y_entry = y_r + t_entry * dy
    x_exit = x_r + t_exit * dx
    y_exit = y_r + t_exit * dy

    return (float(x_entry), float(y_entry), float(t_entry),
            float(x_exit), float(y_exit), float(t_exit))


def verif_et_ajoute_contournement(Liste_actions, action_voulu, x_actuel, y_actuel, x_voulu, y_voulu, angle_voulu, x_ennemi, y_ennemi, r_robot, r_ennemi, marge_min):
    """
    Vérifie si l'ennemi est sur la trajectoire robot->consigne.
    Si oui, insère un point de contournement dans Liste_actions.
    """
    R_securite = r_robot + r_ennemi + marge_min

    Distance_robot_consigne = round(np.sqrt((x_actuel-x_voulu)**2 + (y_actuel-y_voulu)**2),0)

    Distance_consigne_ennemi = round(np.sqrt((x_ennemi-x_voulu)**2 + (y_ennemi-y_voulu)**2),0)
    Distance_robot_ennemi = round(np.sqrt((x_ennemi-x_actuel)**2 + (y_ennemi-y_actuel)**2),0)
    
    if action_voulu == "Recul" : # On souhaite reculer
        print("Action_voulu : Recul")
        if Distance_robot_ennemi < R_securite : # Si Distance_robot_ennemi < R_securite, on doit vraiment reculer
            print("Réel besoin de reculer")
            ratio = R_securite / Distance_robot_ennemi
            x_recul = round(x_ennemi - (x_ennemi-x_actuel) * ratio,0)
            y_recul = round(y_ennemi - (y_ennemi-y_actuel) * ratio,0)
            # Supprimer le "Recul" actuel 
            Liste_actions.pop(0)
            # Reculer, donc ajouter dans Liste_actions : ["Recul",x_recul,y_recul,angle_recul]
            Liste_actions.insert(0,["Recul",int(x_recul),int(y_recul)])
            print("On a supprimé l'ancien recul pour en ajouter un nouveau")
        else : # Plus besoin de reculer
            print("Plus besoin de reculer, donc on retire la consigne Recul")
            Liste_actions.pop(0)
            
    else :
    # Sinon, on souhaite autre chose
        print("Action voulu : Autre ")
        # On vérifie quand même la distance par rapport au robot ennemi
        if Distance_robot_ennemi < R_securite and action_voulu == "Avancer":# Si Consigne dans Périmètre et Robot dans Périmètre
            print("Mais besoin de reculer")
            ratio = R_securite / Distance_robot_ennemi
            x_recul = round(x_ennemi - (x_ennemi-x_actuel) * ratio,0)
            y_recul = round(y_ennemi - (y_ennemi-y_actuel) * ratio,0)
            # Supprimer le "Recul" actuel 
            Liste_actions.pop(0)
            # Reculer, donc ajouter dans Liste_actions : ["Recul",x_recul,y_recul,angle_recul]
            Liste_actions.insert(0,["Avancer",int(x_voulu),int(y_voulu)])
            Liste_actions.insert(0,["Recul",int(x_recul),int(y_recul)])
            print("On ajoute un Recul")

        elif Distance_consigne_ennemi < R_securite and action_voulu == "Avancer": # Si Consigne dans Périmètre et Robot pas dans Périmètre
            print("Avancer jusqu'à la limite")
            x_s, y_s = map(float,point_arret_perimetre(x_actuel, y_actuel, x_voulu, y_voulu, x_ennemi, y_ennemi, R_securite))
            Liste_actions.insert(0,["Avancer",int(x_s),int(y_s)])

        else : # Robot et Consigne pas dans périmètre de l'Ennemi
            print("Pas besoin de reculer")
            Angle_robot_ennemi = round(math.atan2(y_ennemi-y_actuel,x_ennemi-x_actuel)*180/np.pi,0)
            Angle_robot_consigne = round(math.atan2(y_voulu-y_actuel,x_voulu-x_actuel)*180/np.pi,0)
            seuil_angle = np.degrees(np.arcsin(R_securite / Distance_robot_ennemi))
            delta_angle = np.abs(Angle_robot_consigne - Angle_robot_ennemi)
            if delta_angle < seuil_angle and Distance_robot_ennemi < Distance_robot_consigne :
                print("La trajectoire traverse le périmètre → contourner")
                cross = (x_ennemi - x_actuel) * (y_voulu - y_actuel) - (y_ennemi - y_actuel) * (x_voulu - x_actuel)
                sens = 1 if cross > 0 else -1  # gauche (+1) ou droite (-1)

                # Génération de 5 points sur le cercle pour contournement
                theta_centre = math.atan2(y_ennemi - y_actuel, x_ennemi - x_actuel)
                theta_tangent = theta_centre + sens * math.pi/2

                angles = [theta_tangent + sens * math.radians(a) for a in [40]]
                points_contournement = []
                for a in angles:
                    x_c = x_ennemi + R_securite * math.cos(a)
                    y_c = y_ennemi + R_securite * math.sin(a)
                    points_contournement.append(["Contournement", int(round(x_c)), int(round(y_c))])
                points_contournement.sort(key=lambda p: math.hypot(p[1]-x_actuel, p[2]-y_actuel))
                # Insertion dans la liste d’actions avant la consigne finale
                Liste_actions = points_contournement + [["Avancer", int(round(x_voulu)), int(round(y_voulu))]]

            else:
                print("Pas besoin de contourner")
            if action_voulu == "Tourner":
                Liste_actions.insert(0,["Tourner",Liste_actions[0][1]])

    Liste_action_format = []
    for actions in Liste_actions:
        if len(actions)==3:
            (action,x,y) = actions
            Liste_action_format.append([action, int(x),int(y)])
        elif len(actions)==2:
            (action,a) = actions
            Liste_action_format.append([action, float(a)])
        else :
            Liste_action_format.append(actions)
    return Liste_action_format
