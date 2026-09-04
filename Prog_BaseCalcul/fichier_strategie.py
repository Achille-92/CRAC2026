from fonction import peuvent_etre_groupees, distance

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
