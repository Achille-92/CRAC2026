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
    tag_taille_mm_robots: float = 100.0     # Tags mobiles 
    camera_largeur: int = 1920             # Résolution de la caméra
    camera_longueur: int = 1080
    tag_reference: Tuple[int, ...] = (20, 21, 22, 23)
    tag_calibration: Tuple[int, ...] = (51,71)
    tag_mobile: Tuple[int, ...] = (0,1,2,3,4,5,6,7,8,9,10)

    couleur_jaune_rgb: Tuple[int, int, int] = (247, 181, 0)
    couleur_bleu_rgb: Tuple[int, int, int] = (0, 91, 140)
    tolerance_hsv: Tuple[int, int, int] = (30, 150, 150)  # Tolérance pour HSV
    taille_min_contour: int = 12  # Taille minimale du contour en pixels
    taille_max_contour: int = 10000  # Taille maximale du contour en pixels

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
        """
        Calcule l'homographie pour le plan surélevé à partir des 4 positions calibrées.
        calibration_points contient les coins du tag 5 aux 4 positions (au-dessus des tags 20,21,22,23)
        """
        if len(calibration_points) != 4:
            return False

        position_elevated = self.position_tag_elevated()

        src_points = []
        dst_points = []

        # Pour chaque position de calibration
        for tag_id in [20, 21, 22, 23]:
            if tag_id in calibration_points and tag_id in position_elevated:
                # Calculer le centre du tag 5 dans l'image
                centre_tag = np.mean(calibration_points[tag_id], axis=0)
                src_points.append(centre_tag)
                # Position correspondante dans le plan surélevé (mêmes coordonnées que plan ref)
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


class ArUcoDetector:
    def __init__(self, dictionary_type=cv2.aruco.DICT_4X4_100):
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
        self.detecteur = ArUcoDetector()
        self.color_detector = ColorDetector(config)  # Nouveau
        self.homographie = CalculHomographie(config)
        self.calibration_mode = CalibrationMode(config)
        self.mode_calibration_active = False
        self.plan_reference_calcule = False
        self.plan_elevated_calcule = False
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
        """if hasattr(self, 'camera_matrix') and hasattr(self, 'dist_coeffs'):
            return cv2.undistort(image, self.camera_matrix, self.dist_coeffs)"""
        return image

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict]:
        results = {
            'homographie_ok': self.plan_reference_calcule,
            'homographie_elevated_ok': self.plan_elevated_calcule,
            'tag_reference': {},
            'tag_mobile': [],
            'objets_colores': [],
            'calibration_mode': self.mode_calibration_active,
            'calibration_complete': self.calibration_mode.is_complete(),
        }

        undistorted = self.undistort_image(frame)
        
        # IMPORTANT: Créer une copie pour la détection de couleurs SANS rectangles
        undistorted_pour_detection = undistorted.copy()
        
        # Créer une copie pour l'affichage AVEC rectangles
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
                instruction_text = f"MODE CALIBRATION - Positionner Tag {self.config.tag_calibration} au-dessus du tag {target} puis appuyer sur ESPACE"
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
            # MODIFICATION IMPORTANTE: Détection des couleurs sur l'image SANS rectangles
            color_results = self.color_detector.detect_colors(undistorted_pour_detection)
            
            # Compteurs pour les statistiques
            total_jaune = len(color_results['jaune'])
            total_bleu = len(color_results['bleu'])
            filtre_jaune = 0
            filtre_bleu = 0
            
            # Définir le rectangle étendu du plan SURÉLEVÉ pour le filtrage spatial
            cfg = self.config
            coins_elevated_mm = [
                np.array([0, 0]),
                np.array([cfg.largeur_totale_mm, 0]),
                np.array([cfg.largeur_totale_mm, cfg.longueur_totale_mm]),
                np.array([0, cfg.longueur_totale_mm])
            ]
            
            # Listes pour stocker les objets valides (après filtrage spatial)
            objets_jaunes_valides = []
            objets_bleus_valides = []
            
            # Traiter les objets jaunes avec filtrage spatial
            for obj in color_results['jaune']:
                centre_pixel = obj['centre_pixel']
                pos_elevated = self.homographie.point_cam_to_elevated(centre_pixel)
                
                if pos_elevated is not None:
                    if self.point_dans_polygone(pos_elevated, coins_elevated_mm):
                        obj['pos_elevated'] = pos_elevated
                        objets_jaunes_valides.append(obj)
                    else:
                        filtre_jaune += 1
            
            # Traiter les objets bleus avec filtrage spatial
            for obj in color_results['bleu']:
                centre_pixel = obj['centre_pixel']
                pos_elevated = self.homographie.point_cam_to_elevated(centre_pixel)
                
                if pos_elevated is not None:
                    if self.point_dans_polygone(pos_elevated, coins_elevated_mm):
                        obj['pos_elevated'] = pos_elevated
                        objets_bleus_valides.append(obj)
                    else:
                        filtre_bleu += 1
            
            # NOUVEAU: Analyser les paires d'objets
            objets_jaunes_valides = self._split_large_objects(objets_jaunes_valides)
            objets_bleus_valides = self._split_large_objects(objets_bleus_valides)

            # Analyser les paires d'objets
            paires_jaunes = self._analyze_color_pairs(objets_jaunes_valides, 'jaune', objets_bleus_valides)
            paires_bleues = self._analyze_color_pairs(objets_bleus_valides, 'bleu', objets_jaunes_valides)

            
            # Afficher les paires détectées
            for paire in paires_jaunes:
                self._draw_color_pair(paire, undistorted_copie, (0, 255, 255))  # Jaune
                results['objets_colores'].append(paire)
            
            for paire in paires_bleues:
                self._draw_color_pair(paire, undistorted_copie, (140, 91, 0))  # Bleu
                results['objets_colores'].append(paire)
            
            # Afficher les objets isolés (non appariés)
            for obj in objets_jaunes_valides:
                if obj.get('pos_elevated') is not None:
                    self._process_colored_object(obj, undistorted_copie, results, (0, 255, 255))
            
            for obj in objets_bleus_valides:
                if obj.get('pos_elevated') is not None:
                    self._process_colored_object(obj, undistorted_copie, results, (140, 91, 0))
            
            # Afficher les statistiques
            if total_jaune > 0 or total_bleu > 0:
                print(f"\n[Filtrage spatial - Plan SURÉLEVÉ]")
                print(f"  Jaune: {total_jaune - filtre_jaune}/{total_jaune} acceptés ({filtre_jaune} hors zone)")
                print(f"    → {len(paires_jaunes)} paire(s) détectée(s)")
                print(f"  Bleu: {total_bleu - filtre_bleu}/{total_bleu} acceptés ({filtre_bleu} hors zone)")
                print(f"    → {len(paires_bleues)} paire(s) détectée(s)")
                print(f"  Total objets/paires valides: {len(results['objets_colores'])}")
            # NOUVEAU: Analyser les paires d'objets
            paires_jaunes = self._analyze_color_pairs(objets_jaunes_valides, 'jaune', objets_bleus_valides)
            paires_bleues = self._analyze_color_pairs(objets_bleus_valides, 'bleu', objets_jaunes_valides)

            # NOUVEAU: Créer la liste Liste_noisette_xya
            Liste_noisette_xya = []

            # Ajouter les paires jaunes
            for paire in paires_jaunes:
                x = int(paire['centre_mm'][0])
                y = int(paire['centre_mm'][1])
                angle = int(paire['angle_deg'])
                couleur = 'J'
                Liste_noisette_xya.append([x, y, angle, couleur])

            # Ajouter les paires bleues
            for paire in paires_bleues:
                x = int(paire['centre_mm'][0])
                y = int(paire['centre_mm'][1])
                angle = int(paire['angle_deg'])
                couleur = 'B'
                Liste_noisette_xya.append([x, y, angle, couleur])

            # Afficher la liste dans la console
            if len(Liste_noisette_xya) > 0:
                print("\n" + "="*80)
                print("LISTE_NOISETTE_XYA:")
                print("="*80)
                for i, noisette in enumerate(Liste_noisette_xya):
                    print(f"  Noisette {i+1}: X={noisette[0]:7.1f}mm, Y={noisette[1]:7.1f}mm, Angle={noisette[2]:6.1f}°, Couleur={noisette[3]}")
                print("="*80)

            # Ajouter la liste aux résultats
            results['Liste_noisette_xya'] = Liste_noisette_xya

            # Marquer les objets utilisés dans les paires
            objets_jaunes_utilises = set()
            objets_bleus_utilises = set()
            # Dessiner le rectangle du plan élevé en utilisant tous les coins détectés (convexHull)
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
                    #cv2.polylines(undistorted_copie, [elevated_rectangle], True, (255, 0, 255), 3)  # Magenta
            
            # Dessiner le rectangle ÉTENDU du plan surélevé (en cyan)
            cfg = self.config
            
            # Les centres des tags calibrés définissent directement les coins du rectangle
            haut_droit_elev_mm = np.array([0, 0])
            haut_gauche_elev_mm = np.array([cfg.tags_largeur_mm + 2*cfg.ajout_horizontal_mm, 0])
            bas_droit_elev_mm = np.array([0, cfg.tags_longueur_mm + 2*cfg.ajout_vertical_mm])
            bas_gauche_elev_mm = np.array([cfg.tags_largeur_mm + 2*cfg.ajout_horizontal_mm, cfg.tags_longueur_mm + 2*cfg.ajout_vertical_mm])

            haut_droit_elev_pixel = self.homographie.point_elevated_to_cam(haut_droit_elev_mm)
            haut_gauche_elev_pixel = self.homographie.point_elevated_to_cam(haut_gauche_elev_mm)
            bas_droit_elev_pixel = self.homographie.point_elevated_to_cam(bas_droit_elev_mm)
            bas_gauche_elev_pixel = self.homographie.point_elevated_to_cam(bas_gauche_elev_mm)
            
            if haut_droit_elev_pixel is not None and haut_gauche_elev_pixel is not None and bas_droit_elev_pixel is not None and bas_gauche_elev_pixel is not None:
                extended_elevated_rectangle = np.array([
                    haut_droit_elev_pixel.astype(int),
                    haut_gauche_elev_pixel.astype(int),
                    bas_gauche_elev_pixel.astype(int),
                    bas_droit_elev_pixel.astype(int)
                ], dtype=np.int32)
                
                cv2.polylines(undistorted_copie, [extended_elevated_rectangle], True, (255, 255, 0), 3)  # Cyan
                    
            
        # Collecter tous les tags mobiles (peut avoir plusieurs fois le même ID)
        tag_mobile_list = []
        for tag_id, coins in detected_tags_list:
            if tag_id in self.config.tag_mobile:
                tag_mobile_list.append((tag_id, coins))

        if tag_mobile_list and self.plan_elevated_calcule:
            cfg = self.config
            
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
                    # Afficher les coordonnées dans la console
                    print(f"\nTag {tag_id} (instance {idx + 1}):")
                    print(f"  Plan élevé: X = {pos_elevated[0]:7.1f} mm, Y = {pos_elevated[1]:7.1f} mm")

                    # Pour dessiner le carré, utiliser les coordonnées du plan élevé
                    pos_ref_mm = pos_elevated.copy()

                    # Dessiner le contour du tag projeté dans le plan de référence
                    demi_taille = self.config.tag_taille_mm_robot / 2
                    coin_hg_ref = pos_ref_mm + np.array([-demi_taille, -demi_taille])
                    coin_hd_ref = pos_ref_mm + np.array([demi_taille, -demi_taille])
                    coin_bd_ref = pos_ref_mm + np.array([demi_taille, demi_taille])
                    coin_bg_ref = pos_ref_mm + np.array([-demi_taille, demi_taille])

                    coins_ref_mm = np.array([coin_hg_ref, coin_hd_ref, coin_bd_ref, coin_bg_ref], dtype=np.float32)

                    # Convertir les coins du plan de référence en pixels dans le plan de la caméra
                    projected_corners_cam = []
                    for corner_ref in coins_ref_mm:
                        corner_cam = self.homographie.point_elevated_to_cam(corner_ref)
                        if corner_cam is not None:
                            projected_corners_cam.append(corner_cam)

                    if len(projected_corners_cam) == 4:
                        projected_corners_cam = np.array(projected_corners_cam, dtype=np.int32)
                        cv2.polylines(undistorted_copie, [projected_corners_cam], True, color, 2)

                    # Ajouter à la liste des résultats
                    results['tag_mobile'].append({
                        'tag_id': tag_id,
                        'position_mm': pos_elevated.tolist(),
                        'pixel_center': centre_pixel.tolist()
                    })

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
                        print(f"Position capturée! Placer le tag au-dessus du tag {target} et appuyer sur ESPACE")
                    return True
        
        print(f"Tag {self.config.tag_calibration} non détecté")
        return False
    
    def _process_colored_object(self, obj: Dict, image: np.ndarray, results: Dict, color_bgr: Tuple):
        """Traite un objet coloré détecté et l'affiche"""
        cfg = self.config
        
        # Dessiner le contour avec une épaisseur plus visible
        cv2.drawContours(image, [obj['contour']], -1, color_bgr, 3)
        
        # Dessiner le centre avec un cercle plus grand
        centre_pixel = obj['centre_pixel']
        cv2.circle(image, tuple(centre_pixel.astype(int)), 8, color_bgr, -1)
        cv2.circle(image, tuple(centre_pixel.astype(int)), 10, (255, 255, 255), 2)  # Contour blanc
        
        # Récupérer la position dans le plan surélevé (déjà calculée)
        pos_elevated_tags = obj.get('pos_elevated')
        
        if pos_elevated_tags is not None:
            # Convertir en coordonnées du plan surélevé ÉTENDU (avec marges)
            # L'origine (0,0) du plan étendu est au coin haut-droit avec les marges
            pos_elevated_extended_mm = pos_elevated_tags
            
            # Afficher les coordonnées dans la console
            print(f"\nObjet {obj['couleur'].upper()} (aire={obj['aire']:.0f}px):")
            print(f"  Plan élevé (rectangle tags): X = {pos_elevated_tags[0]:7.1f} mm, Y = {pos_elevated_tags[1]:7.1f} mm")
            print(f"  Plan élevé ÉTENDU (avec marges): X = {pos_elevated_extended_mm[0]:7.1f} mm, Y = {pos_elevated_extended_mm[1]:7.1f} mm")
            
            # Afficher les coordonnées sur l'image avec un fond pour la lisibilité
            """text_pos = (int(centre_pixel[0]) + 15, int(centre_pixel[1]) - 15)
            text = f"{obj['couleur'].upper()}: ({pos_elevated_extended_mm[0]:.0f}, {pos_elevated_extended_mm[1]:.0f})mm"
            
            # Fond noir pour le texte
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(image, 
                        (text_pos[0] - 5, text_pos[1] - text_height - 5),
                        (text_pos[0] + text_width + 5, text_pos[1] + baseline + 5),
                        (0, 0, 0), -1)
            
            # Texte en couleur
            cv2.putText(image, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)"""
            
            # Dessiner une croix au centre pour plus de précision
            cross_size = 15
            cv2.line(image, 
                    (int(centre_pixel[0]) - cross_size, int(centre_pixel[1])),
                    (int(centre_pixel[0]) + cross_size, int(centre_pixel[1])),
                    (255, 255, 255), 2)
            cv2.line(image, 
                    (int(centre_pixel[0]), int(centre_pixel[1]) - cross_size),
                    (int(centre_pixel[0]), int(centre_pixel[1]) + cross_size),
                    (255, 255, 255), 2)
            
            # Ajouter aux résultats avec les coordonnées du plan étendu
            results['objets_colores'].append({
                'couleur': obj['couleur'],
                'elevated_extended_mm': pos_elevated_extended_mm.tolist(),  # COORDONNÉES PRINCIPALES
                'elevated_tags_mm': pos_elevated_tags.tolist(),  # Pour référence
                'pixel_center': centre_pixel.tolist(),
                'aire_pixels': obj['aire']
            })
    
    def point_dans_polygone(self, point: np.ndarray, coins_polygone_mm: List[np.ndarray]) -> bool:
        """
        Vérifie si un point (en mm) est à l'intérieur d'un polygone défini par ses coins
        Utilise l'algorithme du ray casting
        """
        x, y = point[0], point[1]
        n = len(coins_polygone_mm)
        inside = False
        
        p1x, p1y = coins_polygone_mm[0][0], coins_polygone_mm[0][1]
        for i in range(1, n + 1):
            p2x, p2y = coins_polygone_mm[i % n][0], coins_polygone_mm[i % n][1]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside
    def _split_large_objects(self, objects: List[Dict]) -> List[Dict]:
        """
        Détecte les zones de couleur dont les dimensions correspondent à 50x100mm
        (deux noisettes côte-à-côte) et les sépare en deux zones de 50x50mm.
        La détection se fait dans le plan surélevé en mm.
        """
        result = []

        for obj in objects:
            contour = obj['contour']

            # Obtenir le rectangle orienté minimum englobant le contour (en pixels)
            rect = cv2.minAreaRect(contour)
            (cx, cy), (w, h), angle = rect

            # S'assurer que w est toujours la grande dimension
            if w < h:
                w, h = h, w
                angle += 90

            # Convertir les 4 coins du rectangle en coordonnées mm dans le plan surélevé
            box_pixels = cv2.boxPoints(rect)  # 4 coins en pixels
            coins_mm = []
            for pt in box_pixels:
                pt_mm = self.homographie.point_cam_to_elevated(np.array(pt))
                if pt_mm is not None:
                    coins_mm.append(pt_mm)

            if len(coins_mm) != 4:
                result.append(obj)
                continue

            # Calculer largeur et hauteur en mm à partir des coins projetés
            coins_mm = np.array(coins_mm)
            largeur_mm = np.linalg.norm(coins_mm[1] - coins_mm[0])
            hauteur_mm = np.linalg.norm(coins_mm[2] - coins_mm[1])

            # S'assurer que largeur_mm est la grande dimension
            if largeur_mm < hauteur_mm:
                largeur_mm, hauteur_mm = hauteur_mm, largeur_mm

            # Vérifier si les dimensions correspondent à ~50x100mm (tolérance ±20mm)
            est_double = (
                70 < largeur_mm < 130 and   # grande dimension ~100mm
                30 < hauteur_mm < 70         # petite dimension ~50mm
            )

            if not est_double:
                result.append(obj)
                continue

            # La zone est double : calculer les centres des deux sous-zones en mm
            # Centre de la zone complète
            centre_mm = obj['pos_elevated']

            # Vecteur unitaire dans la direction de la grande dimension (en mm)
            # On cherche la direction en comparant les coins projetés
            v1 = coins_mm[1] - coins_mm[0]
            v2 = coins_mm[2] - coins_mm[1]
            if np.linalg.norm(v1) > np.linalg.norm(v2):
                direction = v1 / np.linalg.norm(v1)
            else:
                direction = v2 / np.linalg.norm(v2)

            # Décalage de 25mm de chaque côté du centre
            offset_mm = 25.0
            pos1_mm = centre_mm + direction * offset_mm
            pos2_mm = centre_mm - direction * offset_mm

            # Retrouver les pixels correspondants
            centre1_pixel = self.homographie.point_elevated_to_cam(pos1_mm)
            centre2_pixel = self.homographie.point_elevated_to_cam(pos2_mm)

            if centre1_pixel is None or centre2_pixel is None:
                result.append(obj)
                continue

            # Créer les deux objets séparés
            obj1 = {
                'centre_pixel': centre1_pixel,
                'contour': contour,
                'aire': obj['aire'] / 2,
                'couleur': obj['couleur'],
                'pos_elevated': pos1_mm,
            }
            obj2 = {
                'centre_pixel': centre2_pixel,
                'contour': contour,
                'aire': obj['aire'] / 2,
                'couleur': obj['couleur'],
                'pos_elevated': pos2_mm,
            }

            result.append(obj1)
            result.append(obj2)

            print(f"\n[SPLIT] Zone {obj['couleur'].upper()} séparée: "
                f"{largeur_mm:.0f}x{hauteur_mm:.0f}mm → 2 zones de ~50x50mm")

        return result
    def _analyze_color_pairs(self, objects: List[Dict], color_name: str, other_objects: List[Dict], tolerance_mm: float = 20.0) -> List[Dict]:
        """
        Analyse les objets colorés pour détecter les paires espacées d'environ 100mm
        
        Args:
            objects: Liste des objets détectés de la même couleur
            color_name: Nom de la couleur
            other_objects: Liste des objets de l'autre couleur (pour vérifier qu'il n'y a pas d'intrusion)
            tolerance_mm: Tolérance autour de 100mm (par défaut ±20mm)
        
        Returns:
            Liste des paires détectées avec centre et angle
        """
        target_distance = 100.0  # Distance cible en mm
        min_distance = target_distance - tolerance_mm
        max_distance = target_distance + tolerance_mm
        
        pairs = []
        used_indices = set()
        
        # Comparer chaque paire d'objets
        for i in range(len(objects)):
            if i in used_indices:
                continue
                
            obj1 = objects[i]
            pos1 = obj1['pos_elevated']
            
            for j in range(i + 1, len(objects)):
                if j in used_indices:
                    continue
                    
                obj2 = objects[j]
                pos2 = obj2['pos_elevated']
                
                # Calculer la distance entre les deux objets
                distance = np.linalg.norm(pos2 - pos1)
                
                # Vérifier si la distance est proche de 100mm
                if min_distance <= distance <= max_distance:
                    # NOUVEAU: Vérifier qu'il n'y a pas d'objet de l'autre couleur entre les deux
                    if self._check_interference(pos1, pos2, other_objects):
                        # Il y a une zone de l'autre couleur entre les deux -> ne pas associer
                        continue
                    
                    # Calculer le centre de la paire
                    centre = (pos1 + pos2) / 2
                    
                    # Calculer l'angle (en degrés) entre les deux zones
                    # Angle par rapport à l'axe horizontal (X)
                    delta_x = pos2[0] - pos1[0]
                    delta_y = pos2[1] - pos1[1]
                    angle_rad = np.arctan2(delta_y, delta_x)
                    angle_deg = np.degrees(angle_rad)
                    
                    # Marquer les objets comme utilisés
                    used_indices.add(i)
                    used_indices.add(j)
                    
                    pairs.append({
                        'couleur': color_name,
                        'objet1': {
                            'position_mm': pos1.tolist(),
                            'aire_pixels': obj1['aire']
                        },
                        'objet2': {
                            'position_mm': pos2.tolist(),
                            'aire_pixels': obj2['aire']
                        },
                        'centre_mm': centre.tolist(),
                        'distance_mm': float(distance),
                        'angle_deg': float(angle_deg),
                        'pixel_centers': [obj1['centre_pixel'].tolist(), obj2['centre_pixel'].tolist()]
                    })
                    
                    break  # Passer à l'objet suivant
        
        return pairs

    def _check_interference(self, pos1: np.ndarray, pos2: np.ndarray, other_objects: List[Dict]) -> bool:
        """
        Vérifie si un objet de l'autre couleur se trouve entre deux positions.
        Utilise un couloir dont la largeur est proportionnelle à la distance entre les deux zones.
        """
        if len(other_objects) == 0:
            return False

        vec = pos2 - pos1
        vec_length = np.linalg.norm(vec)

        if vec_length == 0:
            return False

        vec_unit = vec / vec_length

        # Largeur du couloir = moitié de la distance (zones font ~50mm, couloir de 50mm de large)
        margin_mm = vec_length * 1.5

        for other_obj in other_objects:
            other_pos = other_obj['pos_elevated']

            to_other = other_pos - pos1

            # Projection sur l'axe pos1->pos2
            projection_length = np.dot(to_other, vec_unit)

            # L'objet doit être STRICTEMENT entre pos1 et pos2
            # On réduit légèrement les bords pour éviter les faux positifs
            # si l'objet adverse est juste à côté de pos1 ou pos2
            marge_bord = vec_length * 0.15
            if projection_length < marge_bord or projection_length > vec_length - marge_bord:
                continue

            # Distance perpendiculaire à la ligne
            projection_point = pos1 + projection_length * vec_unit
            perpendicular_distance = np.linalg.norm(other_pos - projection_point)

            if perpendicular_distance < margin_mm:
                return True

        return False
    
    def _draw_color_pair(self, paire: Dict, image: np.ndarray, color_bgr: Tuple):
        """Dessine une paire d'objets colorés avec le centre et l'angle"""
        
        # Dessiner une ligne entre les deux objets
        for pixel_center in paire['pixel_centers']:
            cv2.circle(image, tuple(np.array(pixel_center).astype(int)), 6, color_bgr, -1)
        
        # Ligne reliant les deux objets
        pt1 = tuple(np.array(paire['pixel_centers'][0]).astype(int))
        pt2 = tuple(np.array(paire['pixel_centers'][1]).astype(int))
        cv2.line(image, pt1, pt2, color_bgr, 2)
        
        # Calculer la position du centre en pixels
        centre_mm = np.array(paire['centre_mm'])
        centre_pixel = self.homographie.point_elevated_to_cam(centre_mm)
        
        if centre_pixel is not None:
            centre_pixel_int = tuple(centre_pixel.astype(int))
            
            # Dessiner un cercle plus grand au centre
            cv2.circle(image, centre_pixel_int, 12, color_bgr, -1)
            cv2.circle(image, centre_pixel_int, 14, (255, 255, 255), 2)
            
            # Dessiner une flèche indiquant l'angle
            angle_rad = np.radians(paire['angle_deg'])
            arrow_length = 40
            end_x = int(centre_pixel[0] + arrow_length * np.cos(angle_rad))
            end_y = int(centre_pixel[1] + arrow_length * np.sin(angle_rad))
            cv2.arrowedLine(image, centre_pixel_int, (end_x, end_y), (255, 255, 255), 2, tipLength=0.3)
            
            # Afficher les informations
            """text_pos = (int(centre_pixel[0]) + 20, int(centre_pixel[1]) - 20)
            text = f"{paire['couleur'].upper()}: ({centre_mm[0]:.0f}, {centre_mm[1]:.0f})mm, {paire['angle_deg']:.1f}°"
            
            # Fond noir
            (text_width, text_height), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.rectangle(image,
                         (text_pos[0] - 5, text_pos[1] - text_height - 5),
                         (text_pos[0] + text_width + 5, text_pos[1] + baseline + 5),
                         (0, 0, 0), -1)
            
            # Texte
            cv2.putText(image, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_bgr, 2)"""
        
        # Afficher dans la console
        print(f"\nPAIRE {paire['couleur'].upper()}:")
        print(f"  Centre: X = {centre_mm[0]:7.1f} mm, Y = {centre_mm[1]:7.1f} mm")
        print(f"  Distance: {paire['distance_mm']:.1f} mm")
        print(f"  Angle: {paire['angle_deg']:.1f}°")

class ColorDetector:
    def __init__(self, config: Config):
        self.config = config
        
        # Conversion directe RGB -> HSV
        # Jaune RGB (247, 181, 0) -> approximativement HSV(37, 255, 247)
        # Bleu RGB (0, 91, 140) -> approximativement HSV(100, 255, 140)
        
        # JAUNE : Définir manuellement les plages HSV optimales
        # En OpenCV, Hue va de 0-180 (pas 0-360!)
        # Jaune pur = environ 30° en OpenCV (60°/2 car divisé par 2)
        self.jaune_lower = np.array([16, 126, 182])   # H, S, V minimums
        self.jaune_upper = np.array([26, 255, 255])   # H, S, V maximums
        
        # BLEU : Bleu cyan
        # Bleu = environ 100° en OpenCV
        self.bleu_lower = np.array([34, 139, 106])
        self.bleu_upper = np.array([135, 255, 255])
        
        print("\n=== ColorDetector Initialisé ===")
        print(f"Jaune RGB cible: {config.couleur_jaune_rgb}")
        print(f"  -> Plage HSV: {self.jaune_lower} à {self.jaune_upper}")
        print(f"Bleu RGB cible: {config.couleur_bleu_rgb}")
        print(f"  -> Plage HSV: {self.bleu_lower} à {self.bleu_upper}")
    
    def detect_colors(self, image: np.ndarray) -> Dict[str, List[Dict]]:
        """
        Détecte les objets jaunes et bleus dans l'image
        Retourne un dictionnaire avec les centres et contours des objets détectés
        """
        # Conversion en HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        results = {
            'jaune': [],
            'bleu': []
        }
        
        # Détection du jaune
        mask_jaune = cv2.inRange(hsv, self.jaune_lower, self.jaune_upper)
        pixels_jaune = np.count_nonzero(mask_jaune)
        results['jaune'] = self._process_mask(mask_jaune, 'jaune')
        
        # Détection du bleu
        mask_bleu = cv2.inRange(hsv, self.bleu_lower, self.bleu_upper)
        pixels_bleu = np.count_nonzero(mask_bleu)
        results['bleu'] = self._process_mask(mask_bleu, 'bleu')
        
        if pixels_jaune > 0 or pixels_bleu > 0:
            print(f"\n[Couleurs] Pixels: Jaune={pixels_jaune}, Bleu={pixels_bleu} | Objets: Jaune={len(results['jaune'])}, Bleu={len(results['bleu'])}")
        
        return results
    
    def _process_mask(self, mask: np.ndarray, color_name: str) -> List[Dict]:
        """Traite un masque de couleur pour extraire les contours et centres"""
        # Morphologie pour nettoyer le masque
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Trouver les contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_objects = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filtrer par taille
            if self.config.taille_min_contour < area < self.config.taille_max_contour:
                # Calculer le centre
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    detected_objects.append({
                        'centre_pixel': np.array([cx, cy]),
                        'contour': contour,
                        'aire': area,
                        'couleur': color_name
                    })
        
        return detected_objects
    
    def calibrate_interactive(self, image: np.ndarray):
        """
        Mode interactif pour calibrer les plages HSV
        Affiche les masques en temps réel avec des trackbars
        """
        cv2.namedWindow('Calibration HSV')
        cv2.namedWindow('Image Originale')
        
        # Créer les trackbars pour le JAUNE
        cv2.createTrackbar('Jaune H min', 'Calibration HSV', self.jaune_lower[0], 180, lambda x: None)
        cv2.createTrackbar('Jaune H max', 'Calibration HSV', self.jaune_upper[0], 180, lambda x: None)
        cv2.createTrackbar('Jaune S min', 'Calibration HSV', self.jaune_lower[1], 255, lambda x: None)
        cv2.createTrackbar('Jaune S max', 'Calibration HSV', self.jaune_upper[1], 255, lambda x: None)
        cv2.createTrackbar('Jaune V min', 'Calibration HSV', self.jaune_lower[2], 255, lambda x: None)
        cv2.createTrackbar('Jaune V max', 'Calibration HSV', self.jaune_upper[2], 255, lambda x: None)
        
        # Créer les trackbars pour le BLEU
        cv2.createTrackbar('Bleu H min', 'Calibration HSV', self.bleu_lower[0], 180, lambda x: None)
        cv2.createTrackbar('Bleu H max', 'Calibration HSV', self.bleu_upper[0], 180, lambda x: None)
        cv2.createTrackbar('Bleu S min', 'Calibration HSV', self.bleu_lower[1], 255, lambda x: None)
        cv2.createTrackbar('Bleu S max', 'Calibration HSV', self.bleu_upper[1], 255, lambda x: None)
        cv2.createTrackbar('Bleu V min', 'Calibration HSV', self.bleu_lower[2], 255, lambda x: None)
        cv2.createTrackbar('Bleu V max', 'Calibration HSV', self.bleu_upper[2], 255, lambda x: None)
        
        print("\n=== MODE CALIBRATION COULEURS ===")
        print("Ajustez les trackbars pour voir les masques en temps réel")
        print("Appuyez sur 'S' pour sauvegarder les valeurs")
        print("Appuyez sur 'Q' pour quitter le mode calibration")
        
        while True:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Lire les valeurs des trackbars
            j_h_min = cv2.getTrackbarPos('Jaune H min', 'Calibration HSV')
            j_h_max = cv2.getTrackbarPos('Jaune H max', 'Calibration HSV')
            j_s_min = cv2.getTrackbarPos('Jaune S min', 'Calibration HSV')
            j_s_max = cv2.getTrackbarPos('Jaune S max', 'Calibration HSV')
            j_v_min = cv2.getTrackbarPos('Jaune V min', 'Calibration HSV')
            j_v_max = cv2.getTrackbarPos('Jaune V max', 'Calibration HSV')
            
            b_h_min = cv2.getTrackbarPos('Bleu H min', 'Calibration HSV')
            b_h_max = cv2.getTrackbarPos('Bleu H max', 'Calibration HSV')
            b_s_min = cv2.getTrackbarPos('Bleu S min', 'Calibration HSV')
            b_s_max = cv2.getTrackbarPos('Bleu S max', 'Calibration HSV')
            b_v_min = cv2.getTrackbarPos('Bleu V min', 'Calibration HSV')
            b_v_max = cv2.getTrackbarPos('Bleu V max', 'Calibration HSV')
            
            # Créer les masques
            jaune_lower_temp = np.array([j_h_min, j_s_min, j_v_min])
            jaune_upper_temp = np.array([j_h_max, j_s_max, j_v_max])
            bleu_lower_temp = np.array([b_h_min, b_s_min, b_v_min])
            bleu_upper_temp = np.array([b_h_max, b_s_max, b_v_max])
            
            mask_jaune = cv2.inRange(hsv, jaune_lower_temp, jaune_upper_temp)
            mask_bleu = cv2.inRange(hsv, bleu_lower_temp, bleu_upper_temp)
            
            # Combiner les masques pour visualisation
            combined = np.zeros_like(image)
            combined[:, :, 2] = mask_jaune  # Rouge pour jaune
            combined[:, :, 0] = mask_bleu   # Bleu pour bleu
            
            cv2.imshow('Image Originale', image)
            cv2.imshow('Calibration HSV', combined)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                self.jaune_lower = jaune_lower_temp
                self.jaune_upper = jaune_upper_temp
                self.bleu_lower = bleu_lower_temp
                self.bleu_upper = bleu_upper_temp
                print(f"\nValeurs sauvegardées:")
                print(f"  Jaune: {self.jaune_lower} à {self.jaune_upper}")
                print(f"  Bleu: {self.bleu_lower} à {self.bleu_upper}")
        
        cv2.destroyWindow('Calibration HSV')
        cv2.destroyWindow('Image Originale')


if __name__ == "__main__":
    config = Config()
    
    """print("=" * 80)
    print("Système de Tracking ArUco avec Double Plan")
    print("=" * 80)
    print(f"\nConfiguration:")
    print(f"  - Plan de référence étendu: {config.largeur_totale_mm:.0f} x {config.longueur_totale_mm:.0f} mm")
    print(f"  - Rectangle tags: {config.tags_largeur_mm:.0f} x {config.tags_longueur_mm:.0f} mm")
    print(f"  - Marges: {config.ajout_horizontal_mm:.0f} x {config.ajout_vertical_mm:.0f} mm")
    print(f"  - Taille des tags de calibration: {config.tag_taille_mm:.0f} mm")
    print(f"  - Taille des tags mobiles: {config.tag_taille_mm_robot:.0f} mm")
    print(f"  - Hauteur plan élevé: +{config.hauteur_robot_mm:.0f} mm")
    print(f"\nTags de référence (plan h=0): {config.tag_reference}")
    print(f"Tag de calibration (plan h=+{config.hauteur_robot_mm:.0f}mm): {config.tag_calibration}")
    print(f"Tags mobiles (plan h=+{config.hauteur_robot_mm:.0f}mm): {config.tag_mobile}")
    print(f"\nOrigine du repère: coin SUPÉRIEUR DROIT")
    print(f"  - X positif vers la GAUCHE")
    print(f"  - Y positif vers le BAS")
    print(f"\nLes plans de référence et surélevé utilisent les mêmes coordonnées X,Y")
    print(f"Seule la hauteur Z diffère de +{config.hauteur_robot_mm:.0f}mm")
    print("\n" + "=" * 80)
    print("INSTRUCTIONS:")
    print("  - Appuyer sur 'E' pour entrer en mode calibration du plan élevé")
    print("  - En mode calibration:")
    print(f"    1. Positionner le tag {config.tag_calibration} exactement au-dessus de chaque tag de référence")
    print("       (successivement au-dessus des tags 20, 21, 22, 23)")
    print("    2. Appuyer sur ESPACE à chaque position pour capturer")
    print("    3. Appuyer sur 'S' pour sauvegarder la calibration")
    print("    4. Appuyer sur 'R' pour recommencer la calibration")
    print("  - Appuyer sur 'L' pour charger une calibration existante")
    print("  - Appuyer sur 'Q' pour quitter")
    print("\n" + "=" * 80)
    print("VISUALISATION:")
    print("  - VERT : Rectangle étendu du plan de référence")
    print("  - BLEU : Rectangle entre les tags de référence (20-23)")
    print("  - MAGENTA : Rectangle entre les positions calibrées du plan surélevé")
    print("  - CYAN : Rectangle étendu du plan surélevé")
    print("=" * 80)"""

    system = ArUcoTrackingSystem(config, matrice_antidstorsion='calibration_data_HR_camM.npz')

    if system.calibration_mode.load_calibration():
        system.homographie.calcul_homographie_elevated(system.calibration_mode.calibration_points)
        system.plan_elevated_calcule = True

    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print("Erreur: impossible d'ouvrir la caméra")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.camera_largeur)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera_longueur)

    # Vérifiez la résolution effective
    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"Résolution effective: {actual_width}x{actual_height}")

    try:
        while True:
            cv2.namedWindow("Systeme de Tracking ArUco", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Systeme de Tracking ArUco", 1280, 720)  # Taille de la fenêtre d'affichage

            ret, frame = cap.read()
            if not ret:
                print("Erreur de lecture de la caméra")
                break

            annotated, results = system.process_frame(frame)
            Liste_noisette_xya = results['Liste_noisette_xya']
            print(Liste_noisette_xya) 
            
            if hasattr(system, 'show_debug') and system.show_debug:
                system.color_detector.show_debug_masks(frame)

            if results['homographie_ok'] and not system.plan_reference_calcule:
                print("Plan de référence calculé et verrouillé.")
                system.plan_reference_calcule = True

            cv2.imshow("Systeme de Tracking ArUco", annotated)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('e'):
                system.mode_calibration_active = True
                system.calibration_mode.reset()
                print("\nMode calibration du plan élevé activé")
                print(f"Positionner le tag {config.tag_calibration} au-dessus du tag {system.calibration_mode.get_current_target()} et appuyer sur ESPACE")
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
                print(f"Positionner le tag {config.tag_calibration} au-dessus du tag {system.calibration_mode.get_current_target()} et appuyer sur ESPACE")
            elif key == ord('l') and not system.mode_calibration_active:
                if system.calibration_mode.load_calibration():
                    system.homographie.calcul_homographie_elevated(system.calibration_mode.calibration_points)
                    system.plan_elevated_calcule = True

            elif key == ord('c'):
                # Mode calibration interactive des couleurs
                print("\n=== Entrée en mode calibration couleurs ===")
                system.color_detector.calibrate_interactive(frame)

            elif key == ord('d'):
                # Activer/désactiver le mode debug
                if not hasattr(system, 'show_debug'):
                    system.show_debug = False
                system.show_debug = not system.show_debug
                if system.show_debug:
                    print("\nMode debug des masques de couleur ACTIVÉ")
                else:
                    print("\nMode debug des masques de couleur DÉSACTIVÉ")
                    cv2.destroyWindow("Masque Jaune")
                    cv2.destroyWindow("Masque Bleu")
                    cv2.destroyWindow("Masques Combinés (Bleu=Bleu, Jaune=Rouge)")
    finally:
        cap.release()
        cv2.destroyAllWindows()
