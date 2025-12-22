import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, List

@dataclass
class Config:
    tags_largeur_mm: float = 910.0   # Largeur entre les tags
    tags_longueur_mm: float = 450.0  # Longueur entre les tags
    ajout_horizontal_mm: float = 303.0  # Marge à gauche et à droite
    ajout_vertical_mm: float = 225.0    # Marge en haut et en bas
    tag_taille_mm: float = 100.0           # Tags de référence (20 à 23)
    tag_taille_mm_Noisette: float = 40.0     # Tags mobiles (36, 47)
    hauteur_Noisette_mm: float = 30.0       # Hauteur des tags mobiles au-dessus du plan de référence
    hauteur_camera_mm: float = 1480.0     # Hauteur estimée de la caméra au-dessus du plan de référence
    camera_largeur: int = 1280             # Résolution de la caméra
    camera_longueur: int = 720
    tag_reference: Tuple[int, ...] = (20, 21, 22, 23)
    tag_Noisette: Tuple[int, ...] = (36, 47)

    @property
    def largeur_totale_mm(self) -> float:
        """
        Renvoi un entier qui correspond à la largueur totale de la Piste
        """
        return self.tags_largeur_mm + 2 * self.ajout_horizontal_mm

    @property
    def longueur_totale_mm(self) -> float:
        """
        Renvoi un entier qui correspond à la longueur totale de la Piste
        """
        return self.tags_longueur_mm + 2 * self.ajout_vertical_mm

class CalculHomographie:
    def __init__(self, config: Config):
        """
        Initialise la Classe "CalculHomographie" avec la config, et avec les Matrice pour passer du plan Cam vers Piste, et celle de Piste vers Cam
        """
        self.config = config
        self.H_cam_to_ref: Optional[np.ndarray] = None
        self.H_ref_to_cam: Optional[np.ndarray] = None

    def position_tag_reference(self) -> Dict[int, np.ndarray]:
        """
        Renvoi un dicionnaire avec comme clé les N° des Tag de référence, et comme valeur les coordonnées (x,y) des coins extérieurs des Tag dans le plan de référence
        """
        cfg = self.config
        # Positions des coins intérieurs des tags de référence dans le plan de référence
        # Origine en haut à droite, X positif vers la gauche, Y positif vers le bas
        positions = {
            22: np.array([cfg.ajout_horizontal_mm, cfg.ajout_vertical_mm]),  # Coin supérieur droit
            23: np.array([cfg.ajout_horizontal_mm + cfg.tags_largeur_mm, cfg.ajout_vertical_mm]),  # Coin supérieur gauche
            20: np.array([cfg.ajout_horizontal_mm, cfg.ajout_vertical_mm + cfg.tags_longueur_mm]),  # Coin inférieur droit
            21: np.array([cfg.ajout_horizontal_mm + cfg.tags_largeur_mm, cfg.ajout_vertical_mm + cfg.tags_longueur_mm]),  # Coin inférieur gauche
        }
        return positions

    def tag_vers_coin(self, tag_id: int) -> int:
        """ 
        Renvoi, en int, le N° du coin du rectangle vert associé à un Tag de référence
        """
        num_coin = {
            22: 3,  # coin bas-gauche
            23: 2,  # coin bas-droit
            21: 1,  # coin haut-droit
            20: 0,  # coin haut-gauche
        }
        return num_coin[tag_id]

    def calcul_homographie(self, detected_tags: Dict[int, np.ndarray]) -> bool:
        """
        Calcule la Matrice d'Homographie à partir des Tag de référence, et les stocke
        Arg : Dict
        Return : Bool
        """
        position_ref = self.position_tag_reference()
        # Obtient les position de référence des Tag dans le plan de référence

        src_points = [] # Coordonnées des coins extérieurs des Tag dans le plan Cam 
        dst_points = [] # Coordonnées des coins extérieurs des Tag dans le plan Réf 

        for tag_id in self.config.tag_reference:                    # Pour chaque Tag de référence
            if tag_id in detected_tags and tag_id in position_ref:  #   Si le Tag est détecté ET présent dans la liste des Tag de Réf
                coin_tag = detected_tags[tag_id]                    #      Récupère les corordonnées, dans le plan Cam, du coin extérieur du Tag
                corner_idx = self.tag_vers_coin(tag_id)             #      Cherche le N° du coin

                src_points.append(coin_tag[corner_idx])             #      Stocke les coordonnées, du plan Cam, dans "src_points", avec comme index le N° du coin
                dst_points.append(position_ref[tag_id])             #      Stocke les coordonnées, du plan Ref, dans "dst_points", avec comme index le N° du coin

        if len(src_points) >= 4:                                    # Si les 4 Tag de Réf ont été détectés,
            src = np.array(src_points, dtype=np.float32)            #   Converti "src_points" en tableau Numpy : src
            dst = np.array(dst_points, dtype=np.float32)            #   Converti "dst_points" en tableau Numpy : dst

            self.H_cam_to_ref, _ = cv2.findHomography(src, dst)     #   Calcule et stocke la Matrice d'Homographie du plan Cam vers plan Ref

            if self.H_cam_to_ref is not None:                       #   Si la Matrice Cam vers Ref a bien été calculée
                self.H_ref_to_cam = np.linalg.inv(self.H_cam_to_ref)#       Calcule et stocke la Matrice Ref vers Cam
                return True                                         #       Renvoi True

        elif len(src_points) == 3:                                  # Sinon, si seulement 3 Tag de Réf ont été détectés
            return self.calcul_homographie_3(src_points, dst_points, detected_tags, position_ref) # Calcule l'Homographie à partir de 3 points

        return False                                                # Sinon, renvoi False

    def calcul_homographie_3(self, src_points: List[np.ndarray],dst_points: List[np.ndarray],detected_tags: Dict[int, np.ndarray],position_ref: Dict[int, np.ndarray]) -> bool:
        """
        Calcule la Matrice d'Homographie avec 3 points
        Arg : src_points, dst_points, detected_tags, position_ref
        Return : Bool
        Stocke les Matrices d'Homographie Cam vers Réf et Réf vers Cam
        """
        tag_ref_detectes = [id for id in self.config.tag_reference if id in detected_tags]  # Stocke les N° de Tag de Réf détectés par la caméra
        id_manquant = None                                                                  # Initialise le N° de Tag manquant
        for id in self.config.tag_reference:                                                # Pour chaque N° de Tag de Rf
            if id not in tag_ref_detectes:                                                  #   Si le Tag n'est pas détectés à la Cam
                id_manquant = id                                                            #       Stocke la valeur
                break                                                                       #       Interrompt la boucle

        if id_manquant is None:                                                             # Si aucun Tag de Réf n'a été trouvé comme manquant
            return False                                                                    #   Renvoi False

        pos_manquante = position_ref[id_manquant]                                           # Récupère les coordonnées, dans le Plan de Réf, du Tag manquant
        dst_points.append(pos_manquante)                                                    # Stocke les coordonnées, du plan Ref, de ce Tag manquant dans "dst_points"

        if len(src_points) == 3:                                                            # Si 3 Tag de Réf ont été détectés dans la Plan Cam
            estimated_src = src_points[0] + src_points[2] - src_points[1]                   #   Estime les coordonnées du point manquant
            src_points.append(estimated_src)                                                #   Stocke ces (x,y) dans les coordonnées des Tags du Plan Cam

        src = np.array(src_points, dtype=np.float32)                                        # Converti "src_points" en tableau Numpy : src
        dst = np.array(dst_points, dtype=np.float32)                                        # Converti "dst_points" en tableau Numpy : dst

        self.H_cam_to_ref, _ = cv2.findHomography(src, dst)                                 # Calcule et stocke la Matrice d'Homographie du plan Cam vers plan Ref

        if self.H_cam_to_ref is not None:                                                   # Si la Matrice Cam vers Ref a bien été calculée
            self.H_ref_to_cam = np.linalg.inv(self.H_cam_to_ref)                            #   Calcule et stocke la Matrice Ref vers Cam
            return True                                                                     #   Renvoi True

        return False                                                                        # Sinon, renvoi False

    def point_cam_to_ref(self, pixel_point: np.ndarray) -> Optional[np.ndarray]:
        """
        Transforme un point des coordonnées de la caméra vers le plan de référence
        Arg : pixel_point
        Return : Bool ou (x,y)
        """

        if self.H_cam_to_ref is None:                                   # S'il n'y a pas de Matrice d'Homographie de Cam vers Ref
            return None                                                 #   Renvoi None                                 

        pt = np.array([[pixel_point]], dtype=np.float32)                # Converti "pixel_point" en tableau Numpy : pt
        transformed = cv2.perspectiveTransform(pt, self.H_cam_to_ref)   # Calcule, avec la Matrice Cam vers Ref, les coordonnées du point dans le Plan réf
        
        return transformed[0, 0]                                        # Renvoi les coordonnées (x,y)

    def point_ref_to_cam(self, reference_point: np.ndarray) -> Optional[np.ndarray]:
        """
        Transforme un point des coordonnées du plan de référence vers la caméra 
        Arg : reference_point
        Return : Bool ou (x,y)
        Commentaire : même chose que la fonction "point_cam_to_ref"
        """
        if self.H_ref_to_cam is None:
            return None

        pt = np.array([[reference_point]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.H_ref_to_cam)

        return transformed[0, 0]


class CorrectionParallaxe:
    """
    Classe pour annuler l'effet de Parallaxe : vue d'un objet en fonction d'un point de vue
    """
    def __init__(self, config: Config, centre_image: np.ndarray):
        self.config = config
        self.centre_image = centre_image
        self.facteur_parallaxe = config.hauteur_Noisette_mm / config.hauteur_camera_mm # Rapport entre la hauteur du tag Noisette et la hauteur de la caméra 

    def correct_position(self, observed_pixel: np.ndarray) -> np.ndarray:
        """
        Arg : observed_pixel, (coordonnées en pixels observées dans l'image)
        """
        displacement = observed_pixel - self.centre_image                       # Calcule le déplacement du point observé par rapport au centre de l'image. Ce déplacement est un vecteur qui indique de combien et dans quelle direction le point est éloigné du centre.

        corrected = observed_pixel - displacement * self.facteur_parallaxe      # Applique la correction de parallaxe en soustrayant au point observé (observed_pixel) le déplacement (displacement) multiplié par le facteur de parallaxe (parallax_factor).
        return corrected                                                        # Renvoi le point corrigé


class ArUcoDetector:
    def __init__(self, dictionary_type=cv2.aruco.DICT_4X4_50):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_type)    # Dictionnaire des Tag Aruco 4x4
        self.detecteur = cv2.aruco.ArucoDetector(self.aruco_dict)                # Crée un détecteur ArUco en utilisant le dictionnaire aruco_dict

    def detect(self, image: np.ndarray) -> Tuple[Dict[int, np.ndarray], np.ndarray]:
        """
        Détermine les coordonnées des 4 coins de chaque Tag Réf dans le plan Cam
        """

        if len(image.shape) == 3:                                               # Si l'image est sous forme RGB
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)                      #   Converti l'image en gris
        else:                                                                   # Sinon
            gray = image                                                        #   L'image est déjà grises

        coins, id, rejected = self.detecteur.detectMarkers(gray)                 # Détecte les coordonnées des 4 coins et les N° des Tag Aruco sur l'image grisée, dans le plan Cam
        
        detected_tags = {}              
        if id is not None:                                                      # Si des Tag ont été détectés
            for i, tag_id in enumerate(id.flatten()):                           #   Pour chaque i et tag_id dans les id détectés (flatten permet de transformer [[1,2],[3,4]] en [1,2,3,4])
                detected_tags[int(tag_id)] = coins[i][0]                        #       Ajoute dans detected_tags les clés (N° du Tag) et valeurs avec les coordonnées des coins de ce Tag (plan Cam)

        return detected_tags, gray                                              # Renvoi detected_tags et l'image grisée

class ArUcoTrackingSystem:
    def __init__(self, config: Config, matrice_antidstorsion: Optional[str] = None):
        self.config = config                            # Config du système
        self.detecteur = ArUcoDetector()                # Création du détecteur de Aruco
        self.homographie = CalculHomographie(config)    # Calcule les Matrices d'Homographie       

        if matrice_antidstorsion:                       # Si la matrice anti distorsion est chargée 
            self.load_calibration(matrice_antidstorsion)   # L'applique à l'image

        self.centre_image = np.array([config.camera_largeur / 2, config.camera_longueur / 2])   # Calcule les coordonnées du centre du plan Cam
        self.facteur_parallaxe = CorrectionParallaxe(config, self.centre_image)                 # Calcule le facteur de Parallaxe

    def load_calibration(self, filepath: str):
        """
        Charge les coeficient d'anti distorsion dans "camera_matrix" et "dist_coeffs"
        """
        try:
            data = np.load(filepath)
            self.camera_matrix = data['mtx']
            self.dist_coeffs = data['dist']
            print(f"Calibration chargée depuis {filepath}")
        except Exception as e:
            print(f"Impossible de charger la calibration: {e}")

    def undistort_image(self, image: np.ndarray) -> np.ndarray:
        """
        Annule la distorsion créée par la caméra
        """
        # "hasattr" renvoi True si l'objet a 'camera_matrix' comme atribut
        if hasattr(self, 'camera_matrix') and hasattr(self, 'dist_coeffs'): 
            return cv2.undistort(image, self.camera_matrix, self.dist_coeffs)
        return image

    def carre_sur_plan_ref(self, undistorted_copie: np.ndarray, haut_gauche_mm: np.ndarray, bas_droit_mm: np.ndarray) -> np.ndarray:
        """Dessine un carré sur le plan de référence en utilisant les coordonnées de deux coins opposés"""
        if self.homographie.H_ref_to_cam is None:
            return undistorted_copie

        # Calculer les deux autres coins du carré
        haut_droit_mm = np.array([bas_droit_mm[0], haut_gauche_mm[1]])          # Calcule les coordonnées, sur le Plan Ref, des 2 autres coins
        bas_gauche_mm = np.array([haut_gauche_mm[0], bas_droit_mm[1]])

        # Convertir les coins en pixels en utilisant l'homographie inverse
        haut_gauche_pixel = self.homographie.point_ref_to_cam(haut_gauche_mm)
        haut_droit_pixel = self.homographie.point_ref_to_cam(haut_droit_mm)
        bas_droit_pixel = self.homographie.point_ref_to_cam(bas_droit_mm)
        bas_gauche_pixel = self.homographie.point_ref_to_cam(bas_gauche_mm)

        # Si on a les 4 coins sur le plan Ref, créer un carré avec ces 4 coins
        if haut_gauche_pixel is not None and haut_droit_pixel is not None and bas_droit_pixel is not None and bas_gauche_pixel is not None:
            carre = np.array([
                haut_gauche_pixel.astype(int),
                haut_droit_pixel.astype(int),
                bas_droit_pixel.astype(int),
                bas_gauche_pixel.astype(int)
            ], dtype=np.int32)

            # Dessiner le carré
            cv2.polylines(undistorted_copie, [carre], True, (0, 0, 255), 3)

        return undistorted_copie

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict]:
        results = {
            'homographie_ok': False,
            'tag_reference': {},
            'tag_Noisette': {}
        }

        undistorted = self.undistort_image(frame)   # Redresse l'image
        undistorted_copie = undistorted.copy()      # Et en fait une copie

        detected_tags, gray = self.detecteur.detect(undistorted) # Détermine les coordonnées des 4 coins de chaque Tag Réf dans le plan Cam

        for tag_id, coins in detected_tags.items():                                 # Pour chaque N° de Tag et 4-uplets de coordonnées de coin dans le Plan Cam
            coins = coins.astype(np.int32)                                          #   Converti le tableau des coins en tableau d'entier
            cv2.polylines(undistorted_copie, [coins], True, (0, 255, 0), 2)         #   Dessine un quadrilatère, vert, d'épaisseur 2, reliant les 4 coins du Tag
            centre = np.mean(coins, axis=0).astype(int)                             #   Calcule le (x,y) du centre du Tag
            cv2.putText(undistorted_copie, str(tag_id), tuple(centre),              #   Ajoute un texte avec le N° du Tag
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        tag_reference = {tid: coins for tid, coins in detected_tags.items()         # Pour chaque (Tag,4coins) dans les Tag détectés, si le Tag est un Tag Réf, ajoute le couple dans tag_reference, coordonnées des coins dans Plan Cam
                        if tid in self.config.tag_reference}
        
        tag_Noisette = {tid: coins for tid, coins in detected_tags.items()          # Pour chaque (Tag,4coins) dans les Tag détectés, si le Tag est un Tag Noisette, ajoute le couple dans tag_Noisette, coordonnées des coins dans Plan Cam
                      if tid in self.config.tag_Noisette}

        if len(tag_reference) >= 3:                                                 # Si il y a au moins 3 Tag réf   
            homographie_ok = self.homographie.calcul_homographie(tag_reference)     #   Calcule les Matrice d'homographie
            results['homographie_ok'] = homographie_ok                              #   Enregistre l'acquittement

            if homographie_ok:                                                      #   Si Matrice d'homographie bien calculée
                ref_positions = self.homographie.position_tag_reference()           #       Enregistre les coordonnées des coins extérieurs des Tag Réf dans le plan Réf
                for tid in tag_reference:                                           #       Pour chaque tag Réf (plan Cam)
                    if tid in ref_positions:                                        #       Si le Tag Réf a une coordonnées dans le Plan Réf
                        results['tag_reference'][tid] = ref_positions[tid].tolist() # Enregistre, sous forme de liste, les coordonnées dans le Plan Réf des Tag Réf

                # Dessiner le rectangle des tags de référence
                if len(tag_reference) >= 3:
                    rectangle_tags_points = []
                    for tag_id in [23, 22, 20, 21]:
                        if tag_id in tag_reference:
                            if tag_id == 23:
                                rectangle_tags_points.append(tag_reference[tag_id][2])
                            elif tag_id == 22:
                                rectangle_tags_points.append(tag_reference[tag_id][3])
                            elif tag_id == 20:
                                rectangle_tags_points.append(tag_reference[tag_id][0])
                            elif tag_id == 21:
                                rectangle_tags_points.append(tag_reference[tag_id][1])

                    if len(rectangle_tags_points) == 4:
                        rectangle_tags_points = np.array(rectangle_tags_points, dtype=np.int32)
                        cv2.polylines(undistorted_copie, [rectangle_tags_points], True, (255, 0, 0), 2)

                # Dessiner le rectangle du plan de référence étendu
                if len(tag_reference) >= 3:
                    # Coins du plan de référence étendu en millimètres
                    # Origine en haut à droite, X positif vers la gauche, Y positif vers le bas
                    haut_droit_mm = np.array([0, 0])  # Coin supérieur droit (origine)
                    haut_gauche_mm = np.array([self.config.largeur_totale_mm, 0])  # Coin supérieur gauche
                    bas_droit_mm = np.array([0, self.config.longueur_totale_mm])  # Coin inférieur droit
                    bas_gauche_mm = np.array([self.config.largeur_totale_mm, self.config.longueur_totale_mm ])  # Coin inférieur gauche

                    # Convertir les coins en pixels en utilisant l'homographie inverse
                    haut_droit_pixel = self.homographie.point_ref_to_cam(haut_droit_mm)
                    haut_gauche_pixel = self.homographie.point_ref_to_cam(haut_gauche_mm)
                    bas_droit_pixel = self.homographie.point_ref_to_cam(bas_droit_mm)
                    bas_gauche_pixel = self.homographie.point_ref_to_cam(bas_gauche_mm)

                    if haut_droit_pixel is not None and haut_gauche_pixel is not None and bas_droit_pixel is not None and bas_gauche_pixel is not None:
                        extended_rectangle = np.array([
                            haut_droit_pixel.astype(int),  # Coin supérieur droit (origine)
                            haut_gauche_pixel.astype(int),  # Coin supérieur gauche
                            bas_gauche_pixel.astype(int),  # Coin inférieur gauche
                            bas_droit_pixel.astype(int)  # Coin inférieur droit
                        ], dtype=np.int32)

                        # Dessiner le rectangle étendu
                        cv2.polylines(undistorted_copie, [extended_rectangle], True, (0, 255, 0), 3)

                # Dessiner un carré sur le plan de référence
                if homographie_ok:
                    # Exemple : un carré de 200 mm de côté
                    # Origine en haut à droite, X positif vers la gauche, Y positif vers le bas
                    top_left_mm = np.array([200, 0])  # Coin supérieur gauche
                    bottom_right_mm = np.array([0, 200])  # Coin inférieur droit

                    undistorted_copie = self.carre_sur_plan_ref(undistorted_copie, top_left_mm, bottom_right_mm)

        if results['homographie_ok'] and tag_Noisette:                                      # Si l'homographie a été faite et que des Noisettes ont été détectées
            for tag_id, coins in tag_Noisette.items():                                      #   Pour chaque N° de Tag et 4-uplets de coins dans le plan Cam
                centre_pixel = np.mean(coins, axis=0)                                       #       Calcule les coordonnées du centre du Tag, Plan Cam

                observed_ref = self.homographie.point_cam_to_ref(centre_pixel)              #       Passe ce point centre dans le Plan Réf

                if observed_ref is not None:                                                #       Si ça a réussi
                    corrected_pixel = self.facteur_parallaxe.correct_position(centre_pixel) #           Corrige la position du point centre en prenant en compte l'effet parallaxe
                    corrected_ref = self.homographie.point_cam_to_ref(corrected_pixel)      #           Passe le point centre corrigé dans le Plan Réf

                    if corrected_ref is not None:                                           #           Si ça a réussi
                        results['tag_Noisette'][tag_id] = {                                 #               Enregistre le point non corrigé dans Plan Réf, le point corrigé dans le Plan Réf, et le centre du Tag dans Plan Cam
                            'observed_mm': observed_ref.tolist(),
                            'projected_mm': corrected_ref.tolist(),
                            'pixel_center': centre_pixel.tolist()
                        }

                        # Dessiner la projection orthogonale
                        if self.homographie.H_ref_to_cam is not None:                                   # Si l'homographie a été faite
                            corrected_pixel_camera = self.homographie.point_ref_to_cam(corrected_ref)   #   Passe le point centre corrigé du Plan Réf vers Plan Cam
                            if corrected_pixel_camera is not None:                                      #   Si ça a fonctionné
                                cv2.circle(undistorted_copie, tuple(corrected_pixel_camera.astype(int)), 10, (255, 0, 255), 2)  # Dessine le projeté du centre du tag dans le plan Cam
                                cv2.putText(undistorted_copie, f"Projection Tag {tag_id}",                  # Ajoute un texte
                                        tuple(corrected_pixel_camera.astype(int) + np.array([15, -15])),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)

                                # Dessiner une ligne entre le tag et sa projection
                                cv2.line(undistorted_copie,
                                        tuple(centre_pixel.astype(int)),
                                        tuple(corrected_pixel_camera.astype(int)),
                                        (255, 255, 0), 2)

                                # Afficher les coordonnées
                                coord_text = f"({corrected_ref[0]:.0f}, {corrected_ref[1]:.0f})"
                                cv2.putText(undistorted_copie, coord_text,
                                        tuple(corrected_pixel_camera.astype(int) + np.array([15, 30])),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Affiche un test de Status à propos de l'homographie
        status_text = f"Tags ref: {len(tag_reference)}/4 | Tags mobiles: {len(tag_Noisette)}"
        color = (0, 255, 0) if results['homographie_ok'] else (0, 0, 255)
        cv2.putText(undistorted_copie, status_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        if results['homographie_ok']:
            cv2.putText(undistorted_copie, "Homographie OK", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(undistorted_copie, "Homographie INVALIDE - besoin de 3+ tags", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        return undistorted_copie, results # Renvoi la vue sans distorsion, ainsi que tous les résultats de la détection

def main():
    config = Config() # Créé la config du système (Objet "config")

    print("=" * 60)
    print("Système de Tracking ArUco avec Projection Orthogonale")
    print("=" * 60)
    print(f"\nConfiguration:")
    print(f"  - Plan de référence: {config.largeur_totale_mm:.0f} x {config.largeur_totale_mm:.0f} mm")
    print(f"  - Rectangle tags: {config.tags_largeur_mm:.0f} x {config.tags_longueur_mm:.0f} mm")
    print(f"  - Marges: {config.ajout_horizontal_mm:.0f} x {config.ajout_vertical_mm:.0f} mm")
    print(f"  - Hauteur tags mobiles: +{config.hauteur_Noisette_mm:.0f} mm")
    print(f"  - Hauteur caméra estimée: {config.hauteur_camera_mm:.0f} mm")
    print(f"\nTags de référence (plan h=0): {config.tag_reference}")
    print(f"Tags mobiles (plan h=+{config.hauteur_Noisette_mm:.0f}mm): {config.tag_Noisette}")
    print(f"\nOrigine du repère: coin SUPÉRIEUR DROIT du plan de référence")
    print(f"  - X positif vers la GAUCHE")
    print(f"  - Y positif vers le BAS")
    print("\nAppuyez sur 'q' pour quitter")
    print("=" * 60)

    system = ArUcoTrackingSystem(config, matrice_antidstorsion='calibration_data.npz')  # Créé le système de Tracking de Tag

    # Commence la lecture de la caméra
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print("Erreur: impossible d'ouvrir la caméra")
        return

    # Initialise la fenêtre de visualisation
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.camera_largeur)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera_longueur)

    try:
        while True:
            ret, frame = cap.read()                     # Stocke l'acquittement de lecture (ret) ainsi que la Matrice de l'image capturée
            if not ret:
                print("Erreur de lecture de la caméra")
                break

            annotated, results = system.process_frame(frame)    # Stocke la copie annotée de l'image ainsi que les résulats de l'analyse

            if results['tag_Noisette']:                         # Si des Noisettes (Tag 36 et 47) ont été détectées
                print("\n--- Coordonnées des tags mobiles (projection orthogonale) ---")
                for tag_id, data in results['tag_Noisette'].items():    # Alors pour chaque (id,data) dans les Noisettes détectées
                    proj = data['projected_mm']                         #   Récupère les coordonnées
                    print(f"  Tag {tag_id}: X = {proj[0]:.1f} mm, Y = {proj[1]:.1f} mm") #  Les prints dans le Terminal

            cv2.imshow("Camera avec Plan de Référence", annotated) # Affiche la copie anotée de l'image

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
