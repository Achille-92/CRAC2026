import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, List
from collections import deque
import json

@dataclass
class Config:
    tags_largeur_mm: float = 1900.0   # Largeur entre les tags
    tags_longueur_mm: float = 900.0  # Longueur entre les tags
    ajout_horizontal_mm: float = 550.0  # Marge à gauche et à droite
    ajout_vertical_mm: float = 550.0    # Marge en haut et en bas
    tag_taille_mm: float = 100.0           # Tags de référence (20 à 23)
    tag_taille_mm_Noisette: float = 40.0     # Tags mobiles (36, 47)
    hauteur_Noisette_mm: float = 30.0      # Hauteur du plan élevé
    hauteur_camera_mm: float = 1480.0     # Hauteur estimée de la caméra
    camera_largeur: int = 1280             # Résolution de la caméra
    camera_longueur: int = 720
    tag_reference: Tuple[int, ...] = (20, 21, 22, 23)
    tag_calibration: int = 5
    tag_mobile: Tuple[int, ...] = (36, 47)

    @property
    def largeur_totale_mm(self) -> float:
        """Renvoi un entier qui correspond à la largeur totale de la Piste"""
        return self.tags_largeur_mm + 2 * self.ajout_horizontal_mm

    @property
    def longueur_totale_mm(self) -> float:
        """Renvoi un entier qui correspond à la longueur totale de la Piste"""
        return self.tags_longueur_mm + 2 * self.ajout_vertical_mm

class CalculHomographie:
    def __init__(self, config: Config):
        self.config = config
        self.H_cam_to_ref: Optional[np.ndarray] = None
        self.H_ref_to_cam: Optional[np.ndarray] = None
        self.H_cam_to_elevated: Optional[np.ndarray] = None
        self.H_elevated_to_cam: Optional[np.ndarray] = None
        self.H_ref_to_elevated: Optional[np.ndarray] = None
        self.H_elevated_to_ref: Optional[np.ndarray] = None
        self.tag_memory = {tag_id: deque(maxlen=10) for tag_id in config.tag_reference}

    def position_tag_reference(self) -> Dict[int, np.ndarray]:
        """Renvoi un dictionnaire avec comme clé les N° des Tag de référence, et comme valeur les coordonnées (x,y) des coins extérieurs des Tag dans le plan de référence"""
        cfg = self.config
        demi_taille = cfg.tag_taille_mm / 2

        # Coins extérieurs des tags de référence (basés sur les coins extérieurs des tags)
        positions = {
            22: np.array([0, 0]),  # Coin supérieur droit (origine)
            23: np.array([cfg.tags_largeur_mm, 0]),  # Coin supérieur gauche
            20: np.array([0, cfg.tags_longueur_mm]),  # Coin inférieur droit
            21: np.array([cfg.tags_largeur_mm, cfg.tags_longueur_mm]),  # Coin inférieur gauche
        }
        return positions

    def position_tag_elevated(self) -> Dict[int, np.ndarray]:
        cfg = self.config
        demi_taille = cfg.tag_taille_mm / 2
        positions = {
            22: np.array([demi_taille, demi_taille]),  # Coin supérieur droit
            23: np.array([cfg.largeur_totale_mm - demi_taille, demi_taille]),  # Coin supérieur gauche
            20: np.array([demi_taille, cfg.longueur_totale_mm - demi_taille]),  # Coin inférieur droit
            21: np.array([cfg.largeur_totale_mm - demi_taille, cfg.longueur_totale_mm - demi_taille]),  # Coin inférieur gauche
        }
        return positions

    def tag_vers_coin(self, tag_id: int) -> int:
        num_coin = {
            22: 3,  # coin bas-gauche
            23: 2,  # coin bas-droit
            21: 1,  # coin haut-droit
            20: 0,  # coin haut-gauche
        }
        return num_coin[tag_id]

    def update_tag_memory(self, detected_tags: Dict[int, np.ndarray]):
        for tag_id in self.config.tag_reference:
            if tag_id in detected_tags:
                self.tag_memory[tag_id].append(detected_tags[tag_id])

    def get_tag_memory(self, tag_id: int) -> Optional[np.ndarray]:
        if self.tag_memory[tag_id]:
            return np.mean(np.array(self.tag_memory[tag_id]), axis=0)
        return None

    def calcul_homographie(self, detected_tags: Dict[int, np.ndarray]) -> bool:
        self.update_tag_memory(detected_tags)
        position_ref = self.position_tag_reference()

        src_points = []
        dst_points = []

        for tag_id in self.config.tag_reference:
            corner_idx = self.tag_vers_coin(tag_id)
            if tag_id in detected_tags:
                coin_tag = detected_tags[tag_id]
            else:
                coin_tag = self.get_tag_memory(tag_id)
                if coin_tag is None:
                    continue

            if coin_tag is not None and tag_id in position_ref:
                src_points.append(coin_tag[corner_idx])
                dst_points.append(position_ref[tag_id])

        if len(src_points) >= 4:
            src = np.array(src_points, dtype=np.float32)
            dst = np.array(dst_points, dtype=np.float32)

            self.H_cam_to_ref, _ = cv2.findHomography(src, dst)

            if self.H_cam_to_ref is not None:
                self.H_ref_to_cam = np.linalg.inv(self.H_cam_to_ref)
                return True

        elif len(src_points) >= 3:
            return self.calcul_homographie_3(src_points, dst_points, detected_tags, position_ref)

        return False

    def calcul_homographie_3(self, src_points: List[np.ndarray], dst_points: List[np.ndarray], detected_tags: Dict[int, np.ndarray], position_ref: Dict[int, np.ndarray]) -> bool:
        tag_ref_detectes = [id for id in self.config.tag_reference if id in detected_tags]
        id_manquant = None
        for id in self.config.tag_reference:
            if id not in tag_ref_detectes:
                id_manquant = id
                break

        if id_manquant is None:
            return False

        pos_manquante = position_ref[id_manquant]
        dst_points.append(pos_manquante)

        if len(src_points) == 3:
            estimated_src = src_points[0] + src_points[2] - src_points[1]
            src_points.append(estimated_src)

        src = np.array(src_points, dtype=np.float32)
        dst = np.array(dst_points, dtype=np.float32)

        self.H_cam_to_ref, _ = cv2.findHomography(src, dst)

        if self.H_cam_to_ref is not None:
            self.H_ref_to_cam = np.linalg.inv(self.H_cam_to_ref)
            return True

        return False

    def calcul_homographie_elevated(self, calibration_points: Dict[int, np.ndarray]) -> bool:
        if len(calibration_points) != 4:
            return False

        position_elevated = self.position_tag_elevated()

        src_points = []
        dst_points = []

        for tag_id in [20, 21, 22, 23]:
            if tag_id in calibration_points and tag_id in position_elevated:
                centre_tag = np.mean(calibration_points[tag_id], axis=0)
                src_points.append(centre_tag)
                dst_points.append(position_elevated[tag_id])

        if len(src_points) == 4:
            src = np.array(src_points, dtype=np.float32)
            dst = np.array(dst_points, dtype=np.float32)

            self.H_cam_to_elevated, _ = cv2.findHomography(src, dst)

            if self.H_cam_to_elevated is not None:
                self.H_elevated_to_cam = np.linalg.inv(self.H_cam_to_elevated)

                if self.H_cam_to_ref is not None:
                    self.H_ref_to_elevated = self.H_cam_to_elevated @ np.linalg.inv(self.H_cam_to_ref)
                    self.H_elevated_to_ref = np.linalg.inv(self.H_ref_to_elevated)

                return True

        return False

    def point_cam_to_ref(self, pixel_point: np.ndarray) -> Optional[np.ndarray]:
        if self.H_cam_to_ref is None:
            return None

        pt = np.array([[pixel_point]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.H_cam_to_ref)

        return transformed[0, 0]

    def point_ref_to_cam(self, reference_point: np.ndarray) -> Optional[np.ndarray]:
        if self.H_ref_to_cam is None:
            return None

        pt = np.array([[reference_point]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.H_ref_to_cam)

        return transformed[0, 0]

    def point_cam_to_elevated(self, pixel_point: np.ndarray) -> Optional[np.ndarray]:
        if self.H_cam_to_elevated is None:
            return None

        pt = np.array([[pixel_point]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.H_cam_to_elevated)

        return transformed[0, 0]

    def point_elevated_to_cam(self, elevated_point: np.ndarray) -> Optional[np.ndarray]:
        if self.H_elevated_to_cam is None:
            return None

        pt = np.array([[elevated_point]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.H_elevated_to_cam)

        return transformed[0, 0]

    def point_ref_to_elevated(self, reference_point: np.ndarray) -> Optional[np.ndarray]:
        if self.H_ref_to_elevated is None:
            return None

        pt = np.array([[reference_point]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.H_ref_to_elevated)

        return transformed[0, 0]

    def point_elevated_to_ref(self, elevated_point: np.ndarray) -> Optional[np.ndarray]:
        if self.H_elevated_to_ref is None:
            return None

        pt = np.array([[elevated_point]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.H_elevated_to_ref)

        return transformed[0, 0]

class CorrectionParallaxe:
    def __init__(self, config: Config, centre_image: np.ndarray):
        self.config = config
        self.centre_image = centre_image
        self.facteur_parallaxe = config.hauteur_Noisette_mm / config.hauteur_camera_mm

    def correct_position(self, observed_pixel: np.ndarray) -> np.ndarray:
        displacement = observed_pixel - self.centre_image
        corrected = observed_pixel - displacement * self.facteur_parallaxe
        return corrected

class ArUcoDetector:
    def __init__(self, dictionary_type=cv2.aruco.DICT_4X4_50):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_type)
        self.detecteur = cv2.aruco.ArucoDetector(self.aruco_dict)

    def detect(self, image: np.ndarray) -> Tuple[List[Tuple[int, np.ndarray]], np.ndarray]:
        """
        Détecte les tags ArUco et retourne une liste de tuples (tag_id, coins)
        pour supporter plusieurs tags avec le même ID
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        coins, id, rejected = self.detecteur.detectMarkers(gray)

        detected_tags = []
        if id is not None:
            for i, tag_id in enumerate(id.flatten()):
                detected_tags.append((int(tag_id), coins[i][0]))

        return detected_tags, gray

class CalibrationMode:
    def __init__(self, config: Config):
        self.config = config
        self.calibration_points = {}
        self.sequence = [20, 21, 22, 23]
        self.current_index = 0
        self.calibration_file = "elevated_plane_calibration.json"

    def is_complete(self) -> bool:
        return len(self.calibration_points) == 4

    def get_current_target(self) -> Optional[int]:
        if self.current_index < len(self.sequence):
            return self.sequence[self.current_index]
        return None

    def capture_position(self, tag_corners: np.ndarray) -> bool:
        target = self.get_current_target()
        if target is not None:
            self.calibration_points[target] = tag_corners.copy()
            self.current_index += 1
            return True
        return False

    def reset(self):
        self.calibration_points = {}
        self.current_index = 0

    def save_calibration(self):
        data = {}
        for tag_id, corners in self.calibration_points.items():
            data[str(tag_id)] = corners.tolist()

        with open(self.calibration_file, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Calibration sauvegardée dans {self.calibration_file}")

    def load_calibration(self) -> bool:
        try:
            with open(self.calibration_file, 'r') as f:
                data = json.load(f)

            self.calibration_points = {}
            for tag_id_str, corners_list in data.items():
                tag_id = int(tag_id_str)
                self.calibration_points[tag_id] = np.array(corners_list, dtype=np.float32)

            self.current_index = len(self.calibration_points)
            print(f"Calibration chargée depuis {self.calibration_file}")
            return True
        except FileNotFoundError:
            print(f"Aucun fichier de calibration trouvé")
            return False
        except Exception as e:
            print(f"Erreur lors du chargement de la calibration: {e}")
            return False

class ArUcoTrackingSystem:
    def __init__(self, config: Config, matrice_antidstorsion: Optional[str] = None):
        self.config = config
        #self.pos_gm = pos_gm
        self.detecteur = ArUcoDetector()
        self.homographie = CalculHomographie(config)
        self.calibration_mode = CalibrationMode(config)
        self.mode_calibration_active = False
        self.plan_reference_calcule = False
        self.plan_elevated_calcule = False
        self.correction_parallaxe = CorrectionParallaxe(config, np.array([config.camera_largeur / 2, config.camera_longueur / 2]))

        if matrice_antidstorsion:
            self.load_calibration(matrice_antidstorsion)

        self.centre_image = np.array([config.camera_largeur / 2, config.camera_longueur / 2])

    def load_calibration(self, filepath: str):
        try:
            data = np.load(filepath)
            self.camera_matrix = data['mtx']
            self.dist_coeffs = data['dist']
            print(f"Calibration chargée depuis {filepath}")
        except Exception as e:
            print(f"Impossible de charger la calibration: {e}")

    def undistort_image(self, image: np.ndarray) -> np.ndarray:
        if hasattr(self, 'camera_matrix') and hasattr(self, 'dist_coeffs'):
            return cv2.undistort(image, self.camera_matrix, self.dist_coeffs)
        return image

    """def is_point_in_zone_gm(self, point: np.ndarray, zone_gm: Tuple[Tuple[float, float], Tuple[float, float]]) -> bool:
        coin1, coin2 = zone_gm
        x_min = min(coin1[0], coin2[0])
        x_max = max(coin1[0], coin2[0])
        y_min = min(coin1[1], coin2[1])
        y_max = max(coin1[1], coin2[1])
        
        return x_min <= point[0] <= x_max and y_min <= point[1] <= y_max"""

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict]:
        results = {
            'homographie_ok': self.plan_reference_calcule,
            'homographie_elevated_ok': self.plan_elevated_calcule,
            'tag_reference': {},
            'tag_mobile': [],
            'calibration_mode': self.mode_calibration_active,
            'calibration_complete': self.calibration_mode.is_complete(),
            #'liste_gm_libres': [],
            #'liste_gm_occupees': []
        }

        undistorted = self.undistort_image(frame)
        undistorted_copie = undistorted.copy()

        detected_tags_list, gray = self.detecteur.detect(undistorted)

        # Dessiner les contours de tous les tags détectés
        for tag_id, coins in detected_tags_list:
            coins_int = coins.astype(np.int32)
            if tag_id in self.config.tag_reference:
                cv2.polylines(undistorted_copie, [coins_int], True, (255, 0, 0), 1)
            elif tag_id in self.config.tag_mobile:
                # Définir la couleur selon le numéro du tag
                if tag_id == 36:
                    color = (255, 0, 0)  # Bleu pour tag 36
                elif tag_id == 47:
                    color = (0, 255, 255)  # Jaune pour tag 47
                else:
                    color = (255, 0, 255)  # Magenta par défaut
                cv2.polylines(undistorted_copie, [coins_int], True, color, 2)

        # Créer un dictionnaire pour les tags de référence (un seul de chaque)
        tag_reference = {}
        for tag_id, coins in detected_tags_list:
            if tag_id in self.config.tag_reference:
                if tag_id not in tag_reference:
                    tag_reference[tag_id] = coins

        if not self.plan_reference_calcule and len(tag_reference) >= 3:
            homographie_ok = self.homographie.calcul_homographie(tag_reference)
            if homographie_ok:
                self.plan_reference_calcule = True
                results['homographie_ok'] = True
                ref_positions = self.homographie.position_tag_reference()
                for tid in tag_reference:
                    if tid in ref_positions:
                        results['tag_reference'][tid] = ref_positions[tid].tolist()

        if self.plan_reference_calcule:
            # Dessiner le rectangle du plan de référence étendu (en vert)
            if len(tag_reference) >= 3:
                cfg = self.config

                haut_droit_mm = np.array([-cfg.ajout_horizontal_mm, -cfg.ajout_vertical_mm])
                haut_gauche_mm = np.array([cfg.tags_largeur_mm + cfg.ajout_horizontal_mm, -cfg.ajout_vertical_mm])
                bas_droit_mm = np.array([-cfg.ajout_horizontal_mm, cfg.tags_longueur_mm + cfg.ajout_vertical_mm])
                bas_gauche_mm = np.array([cfg.tags_largeur_mm + cfg.ajout_horizontal_mm, cfg.tags_longueur_mm + cfg.ajout_vertical_mm])

                haut_droit_pixel = self.homographie.point_ref_to_cam(haut_droit_mm)
                haut_gauche_pixel = self.homographie.point_ref_to_cam(haut_gauche_mm)
                bas_droit_pixel = self.homographie.point_ref_to_cam(bas_droit_mm)
                bas_gauche_pixel = self.homographie.point_ref_to_cam(bas_gauche_mm)

                if haut_droit_pixel is not None and haut_gauche_pixel is not None and bas_droit_pixel is not None and bas_gauche_pixel is not None:
                    extended_rectangle = np.array([
                        haut_droit_pixel.astype(int),
                        haut_gauche_pixel.astype(int),
                        bas_gauche_pixel.astype(int),
                        bas_droit_pixel.astype(int)
                    ], dtype=np.int32)

                    cv2.polylines(undistorted_copie, [extended_rectangle], True, (0, 255, 0), 3)

            # Dessiner le rectangle reliant les tags 20, 21, 22, 23 (en bleu)
            if len(tag_reference) >= 3:
                cfg = self.config

                # Positions des tags dans le plan de référence (coins extérieurs)
                tag_22_mm = np.array([0, 0])  # Coin supérieur droit
                tag_23_mm = np.array([cfg.tags_largeur_mm, 0])  # Coin supérieur gauche
                tag_20_mm = np.array([0, cfg.tags_longueur_mm])  # Coin inférieur droit
                tag_21_mm = np.array([cfg.tags_largeur_mm, cfg.tags_longueur_mm])  # Coin inférieur gauche

                # Convertir en pixels
                tag_22_pixel = self.homographie.point_ref_to_cam(tag_22_mm)
                tag_23_pixel = self.homographie.point_ref_to_cam(tag_23_mm)
                tag_20_pixel = self.homographie.point_ref_to_cam(tag_20_mm)
                tag_21_pixel = self.homographie.point_ref_to_cam(tag_21_mm)

                if tag_22_pixel is not None and tag_23_pixel is not None and tag_20_pixel is not None and tag_21_pixel is not None:
                    tags_rectangle = np.array([
                        tag_22_pixel.astype(int),
                        tag_23_pixel.astype(int),
                        tag_21_pixel.astype(int),
                        tag_20_pixel.astype(int)
                    ], dtype=np.int32)

                    #cv2.polylines(undistorted_copie, [tags_rectangle], True, (255, 0, 0), 3)  # Bleu

        if self.mode_calibration_active:
            target = self.calibration_mode.get_current_target()
            if target is not None:
                instruction_text = f"MODE CALIBRATION - Positionner Tag {self.config.tag_calibration} dans le coin {target} puis appuyer sur ESPACE"
                cv2.putText(undistorted_copie, instruction_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(undistorted_copie, f"Progression: {self.calibration_mode.current_index}/4", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            else:
                cv2.putText(undistorted_copie, "CALIBRATION TERMINEE - Appuyer sur 'S' pour sauvegarder", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        if self.calibration_mode.is_complete() and not self.plan_elevated_calcule:
            homographie_elevated_ok = self.homographie.calcul_homographie_elevated(self.calibration_mode.calibration_points)
            if homographie_elevated_ok:
                self.plan_elevated_calcule = True
                results['homographie_elevated_ok'] = True

        if self.plan_elevated_calcule:
            # Utiliser les coins des tags 36 pour dessiner le rectangle du plan élevé
            all_corners = []
            for tag_id in [20, 21, 22, 23]:
                if tag_id in self.calibration_mode.calibration_points:
                    corners = self.calibration_mode.calibration_points[tag_id]
                    all_corners.extend(corners)

            if len(all_corners) == 16:
                all_corners = np.array(all_corners, dtype=np.float32)

                center = np.mean(all_corners, axis=0)
                sorted_corners = sorted(all_corners, key=lambda corner: np.arctan2(corner[1] - center[1], corner[0] - center[0]))

                sorted_corners = np.array(sorted_corners, dtype=np.float32)
                hull = cv2.convexHull(sorted_corners.reshape(-1, 1, 2))
                if hull is not None and len(hull) >= 4:
                    elevated_rectangle = hull.squeeze().astype(int)
                    cv2.polylines(undistorted_copie, [elevated_rectangle], True, (255, 0, 255), 3)

        # Collecter tous les tags mobiles (peut avoir plusieurs fois le même ID)
        tag_mobile_list = []
        for tag_id, coins in detected_tags_list:
            if tag_id in self.config.tag_mobile:
                tag_mobile_list.append((tag_id, coins))

        if tag_mobile_list and self.plan_elevated_calcule:
            cfg = self.config
            
            # Liste pour stocker les centres des tags dans le plan de référence étendu
            centres_tags_ref = []
            
            # Afficher l'en-tête dans la console
            print("\n" + "="*80)
            print(f"Frame - {len(tag_mobile_list)} tag(s) mobile(s) détecté(s)")
            print("="*80)
            
            for idx, (tag_id, coins) in enumerate(tag_mobile_list):
                # Définir la couleur selon le numéro du tag
                if tag_id == 36:
                    color = (255, 0, 0)  # Bleu pour tag 36
                elif tag_id == 47:
                    color = (0, 255, 255)  # Jaune pour tag 47
                else:
                    color = (255, 0, 255)  # Magenta par défaut

                # Dessiner le contour du tag dans le plan de la caméra
                cv2.polylines(undistorted_copie, [coins.astype(np.int32)], True, color, 2)

                # Calculer le centre du tag dans le plan de la caméra
                centre_pixel = np.mean(coins, axis=0)

                # Convertir le centre du tag dans le plan surélevé
                pos_elevated = self.homographie.point_cam_to_elevated(centre_pixel)

                if pos_elevated is not None:
                    # Les coordonnées du plan élevé sont dans le système étendu
                    pos_ref_extended_mm = pos_elevated.copy()
                    
                    # Ajouter le centre à la liste
                    centres_tags_ref.append(pos_ref_extended_mm)
                    
                    # Pour dessiner le carré, il faut convertir vers le système du rectangle bleu
                    pos_ref_mm = pos_elevated - np.array([cfg.ajout_horizontal_mm, cfg.ajout_vertical_mm])

                    # Afficher les coordonnées dans la console
                    print(f"\nTag {tag_id} (instance {idx + 1}):")
                    print(f"  Plan élevé    : X = {pos_elevated[0]:7.1f} mm, Y = {pos_elevated[1]:7.1f} mm")
                    print(f"  Plan référence: X = {pos_ref_extended_mm[0]:7.1f} mm, Y = {pos_ref_extended_mm[1]:7.1f} mm")

                    # Dessiner le contour du tag projeté dans le plan de référence
                    demi_taille = self.config.tag_taille_mm_Noisette / 2
                    coin_hg_ref = pos_ref_mm + np.array([-demi_taille, demi_taille])
                    coin_hd_ref = pos_ref_mm + np.array([demi_taille, demi_taille])
                    coin_bd_ref = pos_ref_mm + np.array([demi_taille, -demi_taille])
                    coin_bg_ref = pos_ref_mm + np.array([-demi_taille, -demi_taille])

                    coins_ref_mm = np.array([coin_hg_ref, coin_hd_ref, coin_bd_ref, coin_bg_ref], dtype=np.float32)

                    # Convertir les coins du plan de référence en pixels dans le plan de la caméra
                    projected_corners_cam = []
                    for corner_ref in coins_ref_mm:
                        corner_cam = self.homographie.point_ref_to_cam(corner_ref)
                        if corner_cam is not None:
                            projected_corners_cam.append(corner_cam)

                    if len(projected_corners_cam) == 4:
                        projected_corners_cam = np.array(projected_corners_cam, dtype=np.int32)
                        cv2.polylines(undistorted_copie, [projected_corners_cam], True, color, 2)

                    # Ajouter à la liste des résultats
                    results['tag_mobile'].append({
                        'tag_id': tag_id,
                        'elevated_mm': pos_elevated.tolist(),
                        'ref_mm': pos_ref_extended_mm.tolist(),
                        'pixel_center': centre_pixel.tolist()
                    })
            
            # Vérifier l'occupation des zones GM
            #compteur_par_zone = {i+1: 0 for i in range(len(self.pos_gm))}
            
            """ for centre in centres_tags_ref:
                for idx, zone_gm in enumerate(self.pos_gm):
                    if self.is_point_in_zone_gm(centre, zone_gm):
                        compteur_par_zone[idx + 1] += 1"""
            
            # Mettre à jour les listes GM libres et occupées
            """liste_gm_libres = []
            liste_gm_occupees = []"""
            
            """for num_zone, count in compteur_par_zone.items():
                if count >= 1:
                    liste_gm_occupees.append(num_zone)
                else:
                    liste_gm_libres.append(num_zone)"""
            
            """results['liste_gm_libres'] = liste_gm_libres
            results['liste_gm_occupees'] = liste_gm_occupees"""
            
            """# Dessiner les zones GM avec la couleur appropriée
            if self.plan_reference_calcule:
                index_GM = 0
                for zone_gm in self.pos_gm:
                    index_GM += 1
                    coin1_mm, coin2_mm = zone_gm
                    
                    # Convertir les coordonnées des coins en tenant compte du système de référence
                    coin1_ref = np.array([coin1_mm[0] - cfg.ajout_horizontal_mm, coin1_mm[1] - cfg.ajout_vertical_mm])
                    coin2_ref = np.array([coin2_mm[0] - cfg.ajout_horizontal_mm, coin2_mm[1] - cfg.ajout_vertical_mm])
                    
                    # Créer les 4 coins du rectangle GM
                    haut_gauche_gm = np.array([min(coin1_ref[0], coin2_ref[0]), min(coin1_ref[1], coin2_ref[1])])
                    bas_droit_gm = np.array([max(coin1_ref[0], coin2_ref[0]), max(coin1_ref[1], coin2_ref[1])])
                    haut_droit_gm = np.array([haut_gauche_gm[0], bas_droit_gm[1]])
                    bas_gauche_gm = np.array([bas_droit_gm[0], haut_gauche_gm[1]])
                    
                    gm_rectangle = np.array([
                        haut_gauche_gm,
                        bas_gauche_gm,
                        bas_droit_gm,
                        haut_droit_gm
                    ], dtype=np.float32)
                    
                    # Convertir tous les coins en pixels
                    gm_corners_pixel = []
                    for corner in gm_rectangle:
                        corner_pixel = self.homographie.point_ref_to_cam(corner)
                        if corner_pixel is not None:
                            gm_corners_pixel.append(corner_pixel)
                    
                    if len(gm_corners_pixel) == 4:
                        gm_corners_pixel = np.array(gm_corners_pixel, dtype=np.int32)
                        # Déterminer la couleur selon si la zone est libre ou occupée
                        if index_GM in liste_gm_libres:
                            color = (0, 255, 0)  # Vert si libre
                        else:
                            color = (0, 0, 255)  # Rouge si occupée
                        cv2.polylines(undistorted_copie, [gm_corners_pixel], True, color, 3)"""

            """# Afficher les informations dans la console
            print("\n" + "-"*80)
            print("État des zones GM:")
            for num_zone, count in compteur_par_zone.items():
                status = "OCCUPÉE" if count >= 2 else "LIBRE"
                print(f"  Zone GM n°{num_zone}: {count} tag(s) - {status}")
            print(f"\nListe GM libres   : {liste_gm_libres}")
            print(f"Liste GM occupées : {liste_gm_occupees}")
            print("="*80)"""
        
        """# Dessiner les zones GM même sans tags mobiles (toutes en vert au démarrage)
        elif self.plan_reference_calcule and not tag_mobile_list:
            cfg = self.config
            # Toutes les zones sont libres s'il n'y a pas de tags mobiles
            results['liste_gm_libres'] = list(range(1, len(self.pos_gm) + 1))
            results['liste_gm_occupees'] = []
            
            index_GM = 0
            for zone_gm in self.pos_gm:
                index_GM += 1
                coin1_mm, coin2_mm = zone_gm
                
                coin1_ref = np.array([coin1_mm[0] - cfg.ajout_horizontal_mm, coin1_mm[1] - cfg.ajout_vertical_mm])
                coin2_ref = np.array([coin2_mm[0] - cfg.ajout_horizontal_mm, coin2_mm[1] - cfg.ajout_vertical_mm])
                
                haut_gauche_gm = np.array([min(coin1_ref[0], coin2_ref[0]), min(coin1_ref[1], coin2_ref[1])])
                bas_droit_gm = np.array([max(coin1_ref[0], coin2_ref[0]), max(coin1_ref[1], coin2_ref[1])])
                haut_droit_gm = np.array([haut_gauche_gm[0], bas_droit_gm[1]])
                bas_gauche_gm = np.array([bas_droit_gm[0], haut_gauche_gm[1]])
                
                gm_rectangle = np.array([
                    haut_gauche_gm,
                    bas_gauche_gm,
                    bas_droit_gm,
                    haut_droit_gm
                ], dtype=np.float32)
                
                gm_corners_pixel = []
                for corner in gm_rectangle:
                    corner_pixel = self.homographie.point_ref_to_cam(corner)
                    if corner_pixel is not None:
                        gm_corners_pixel.append(corner_pixel)
                
                if len(gm_corners_pixel) == 4:
                    gm_corners_pixel = np.array(gm_corners_pixel, dtype=np.int32)
                    color = (0, 255, 0)  # Vert car libre
                    cv2.polylines(undistorted_copie, [gm_corners_pixel], True, color, 3)"""

        if not self.mode_calibration_active:
            cv2.putText(undistorted_copie, "Appuyer sur 'E' pour calibrer plan eleve", (10, undistorted_copie.shape[0] - 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(undistorted_copie, "Appuyer sur 'L' pour charger calibration", (10, undistorted_copie.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return undistorted_copie, results

    def handle_calibration_capture(self, detected_tags_list: List[Tuple[int, np.ndarray]]) -> bool:
        # Chercher le tag de calibration dans la liste
        for tag_id, tag_corners in detected_tags_list:
            if tag_id == self.config.tag_calibration:
                success = self.calibration_mode.capture_position(tag_corners)
                if success:
                    target = self.calibration_mode.get_current_target()
                    if target is None:
                        print("Calibration complete! Appuyer sur 'S' pour sauvegarder.")
                    else:
                        print(f"Position capturée! Placer le tag dans le coin {target} et appuyer sur ESPACE")
                    return True
        
        print(f"Tag {self.config.tag_calibration} non détecté")
        return False

    

if __name__ == "__main__":
    config = Config()
    """
    cote_GM = 200
    Pos_GM = (
        ((0,0),(cote_GM,cote_GM)),
        ((config.largeur_totale_mm-cote_GM,0),(config.largeur_totale_mm,cote_GM))
    )"""
    
    # Initialisation des listes GM
    """Liste_GM_libres = list(range(1, len(Pos_GM) + 1))
    Liste_GM_occupees = []"""
    
    print("=" * 80)
    print("Système de Tracking ArUco avec Double Plan")
    print("=" * 80)
    print(f"\nConfiguration:")
    print(f"  - Plan de référence étendu: {config.largeur_totale_mm:.0f} x {config.longueur_totale_mm:.0f} mm")
    print(f"  - Rectangle tags: {config.tags_largeur_mm:.0f} x {config.tags_longueur_mm:.0f} mm")
    print(f"  - Marges: {config.ajout_horizontal_mm:.0f} x {config.ajout_vertical_mm:.0f} mm")
    print(f"  - Taille des tags de calibration: {config.tag_taille_mm:.0f} mm")
    print(f"  - Taille des tags mobiles: {config.tag_taille_mm_Noisette:.0f} mm")
    print(f"  - Hauteur plan élevé: +{config.hauteur_Noisette_mm:.0f} mm")
    print(f"\nTags de référence (plan h=0): {config.tag_reference}")
    print(f"Tag de calibration (plan h=+{config.hauteur_Noisette_mm:.0f}mm): {config.tag_calibration}")
    print(f"Tags mobiles (plan h=+{config.hauteur_Noisette_mm:.0f}mm): {config.tag_mobile}")
    """print(f"\nZones GM: {len(Pos_GM)} zone(s) de {cote_GM}x{cote_GM} mm")
    print(f"Liste GM initiale: {Liste_GM_libres}")"""
    print(f"\nOrigine du repère: coin SUPÉRIEUR DROIT du plan de référence et du plan surélevé")
    print(f"  - X positif vers la GAUCHE")
    print(f"  - Y positif vers le BAS")
    print("\n" + "=" * 80)
    print("INSTRUCTIONS:")
    print("  - Appuyer sur 'E' pour entrer en mode calibration du plan élevé")
    print("  - En mode calibration:")
    print("    1. Positionner le tag 36 dans les 4 coins extérieurs du rectangle")
    print("    2. Appuyer sur ESPACE à chaque position pour capturer")
    print("    3. Appuyer sur 'S' pour sauvegarder la calibration")
    print("    4. Appuyer sur 'R' pour recommencer la calibration")
    print("  - Appuyer sur 'L' pour charger une calibration existante")
    print("  - Appuyer sur 'Q' pour quitter")
    print("=" * 80)

    system = ArUcoTrackingSystem(config, matrice_antidstorsion='calibration_data.npz')

    if system.calibration_mode.load_calibration():
        system.homographie.calcul_homographie_elevated(system.calibration_mode.calibration_points)
        system.plan_elevated_calcule = True

    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print("Erreur: impossible d'ouvrir la caméra")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.camera_largeur)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera_longueur)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Erreur de lecture de la caméra")
                break

            annotated, results = system.process_frame(frame)

            if results['homographie_ok'] and not system.plan_reference_calcule:
                print("Plan de référence calculé et verrouillé.")
                system.plan_reference_calcule = True

            # Mettre à jour les listes GM à partir des résultats
            """if 'liste_gm_libres' in results and 'liste_gm_occupees' in results:
                Liste_GM_libres = results['liste_gm_libres']
                Liste_GM_occupees = results['liste_gm_occupees']"""

            cv2.imshow("Systeme de Tracking ArUco", annotated)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('e'):
                system.mode_calibration_active = True
                system.calibration_mode.reset()
                print("\nMode calibration du plan élevé activé")
                print(f"Positionner le tag {config.tag_calibration} dans le coin {system.calibration_mode.get_current_target()} et appuyer sur ESPACE")
            elif key == ord(' ') and system.mode_calibration_active:
                detected_tags_list, _ = system.detecteur.detect(system.undistort_image(frame))
                system.handle_calibration_capture(detected_tags_list)
            elif key == ord('s') and system.mode_calibration_active and system.calibration_mode.is_complete():
                system.calibration_mode.save_calibration()
                system.mode_calibration_active = False
                system.plan_elevated_calcule = True
                print("Mode calibration désactivé")
            elif key == ord('r') and system.mode_calibration_active:
                system.calibration_mode.reset()
                print("\nCalibration réinitialisée")
                print(f"Positionner le tag {config.tag_calibration} dans le coin {system.calibration_mode.get_current_target()} et appuyer sur ESPACE")
            elif key == ord('l') and not system.mode_calibration_active:
                if system.calibration_mode.load_calibration():
                    system.homographie.calcul_homographie_elevated(system.calibration_mode.calibration_points)
                    system.plan_elevated_calcule = True

    finally:
        cap.release()
        cv2.destroyAllWindows() 