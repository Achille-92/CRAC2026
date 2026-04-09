import cv2
import numpy as np
from dataclasses import dataclass, field
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
    tag_taille_mm_robot: float = 100.0     # Tags mobiles
    hauteur_camera_mm: float = 1480.0     # Hauteur estimée de la caméra
    camera_largeur: int = 1920             # Résolution de la caméra
    camera_longueur: int = 1080
    tag_reference: Tuple[int, ...] = (20, 21, 22, 23)
    tag_calibration_Noisette: int = 0
    tag_calibration_robot: int = 0
    tag_robot: Tuple[int, ...] = ()
    tag_ennemi: Tuple[int, ...] = ()
    tag_mobile: Tuple[int, ...] = (0, 0)

    couleur_jaune_rgb: Tuple[int, int, int] = (247, 181, 0)
    couleur_bleu_rgb: Tuple[int, int, int] = (0, 91, 140)
    tolerance_hsv: Tuple[int, int, int] = (30, 150, 150)
    taille_min_contour: int = 12
    taille_max_contour: int = 10000

    @property
    def largeur_totale_mm(self) -> float:
        return self.tags_largeur_mm + 2 * self.ajout_horizontal_mm

    @property
    def longueur_totale_mm(self) -> float:
        return self.tags_longueur_mm + 2 * self.ajout_vertical_mm


# ─────────────────────────────────────────────────────────────────────────────
# GESTIONNAIRE DE BOUTONS
# ─────────────────────────────────────────────────────────────────────────────

class Button_A:
    def __init__(self, x, y, w, h, text, color=(100, 100, 100), text_color=(255, 255, 255)):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.text = text
        self.color = color
        self.text_color = text_color
        self.hovered = False
        
    def draw(self, img):
        color = tuple(min(c + 30, 255) for c in self.color) if self.hovered else self.color
        cv2.rectangle(img, (self.x, self.y), (self.x + self.w, self.y + self.h), color, -1)
        cv2.rectangle(img, (self.x, self.y), (self.x + self.w, self.y + self.h), (255, 255, 255), 2)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        text_size = cv2.getTextSize(self.text, font, font_scale, thickness)[0]
        text_x = self.x + (self.w - text_size[0]) // 2
        text_y = self.y + (self.h + text_size[1]) // 2
        cv2.putText(img, self.text, (text_x, text_y), font, font_scale, self.text_color, thickness)
    
    def is_clicked(self, x, y):
        return self.x <= x <= self.x + self.w and self.y <= y <= self.y + self.h
    
    def update_hover(self, x, y):
        self.hovered = self.is_clicked(x, y)


class ButtonManager:
    def __init__(self, window_name):
        self.window_name = window_name
        self.buttons = []
        self.last_click = None
        cv2.setMouseCallback(window_name, self._mouse_callback)
    
    def _mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEMOVE:
            for button in self.buttons:
                button.update_hover(x, y)
        elif event == cv2.EVENT_LBUTTONDOWN:
            for button in self.buttons:
                if button.is_clicked(x, y):
                    self.last_click = button.text
    
    def add_button(self, button):
        self.buttons.append(button)
    
    def draw_all(self, img):
        for button in self.buttons:
            button.draw(img)
    
    def get_last_click(self):
        click = self.last_click
        self.last_click = None
        return click


def appliquer_couleur(config: Config, couleur: str) -> Config:
    """
    Configure les variables de tags selon la couleur de l'équipe.
    couleur == 'B' : équipe Bleue
    couleur == 'J' : équipe Jaune
    """
    if couleur == "B":
        config.tag_calibration_Noisette = 51
        config.tag_calibration_robot    = 52
        config.tag_robot                = (1, 2, 3, 4, 5)
        config.tag_ennemi               = (6, 7, 8, 9, 10)
    elif couleur == "J":
        config.tag_calibration_Noisette = 71
        config.tag_calibration_robot    = 72
        config.tag_robot                = (6, 7, 8, 9, 10)
        config.tag_ennemi               = (1, 2, 3, 4, 5)
    else:
        raise ValueError(f"Couleur '{couleur}' invalide. Valeurs acceptées : 'B' ou 'J'.")
    return config


# ─────────────────────────────────────────────────────────────────────────────
# HOMOGRAPHIE
# ─────────────────────────────────────────────────────────────────────────────

class CalculHomographie:
    def __init__(self, config: Config):
        self.config = config
        # Plan de référence (sol)
        self.H_cam_to_ref: Optional[np.ndarray] = None
        self.H_ref_to_cam: Optional[np.ndarray] = None
        # Plan surélevé Noisette
        self.H_cam_to_elevated: Optional[np.ndarray] = None
        self.H_elevated_to_cam: Optional[np.ndarray] = None
        self.H_ref_to_elevated: Optional[np.ndarray] = None
        self.H_elevated_to_ref: Optional[np.ndarray] = None
        # Plan surélevé Robot
        self.H_cam_to_robot: Optional[np.ndarray] = None
        self.H_robot_to_cam: Optional[np.ndarray] = None

        self.tag_memory = {tag_id: deque(maxlen=10) for tag_id in config.tag_reference}

    # ── Positions de référence ────────────────────────────────────────────────

    def position_tag_reference(self) -> Dict[int, np.ndarray]:
        cfg = self.config
        positions = {
            22: np.array([0, 0]),
            23: np.array([cfg.tags_largeur_mm, 0]),
            20: np.array([0, cfg.tags_longueur_mm]),
            21: np.array([cfg.tags_largeur_mm, cfg.tags_longueur_mm]),
        }
        return positions

    def position_tag_elevated(self) -> Dict[int, np.ndarray]:
        cfg = self.config
        demi_taille = cfg.tag_taille_mm / 2
        positions = {
            22: np.array([demi_taille, demi_taille]),
            23: np.array([cfg.largeur_totale_mm - demi_taille, demi_taille]),
            20: np.array([demi_taille, cfg.longueur_totale_mm - demi_taille]),
            21: np.array([cfg.largeur_totale_mm - demi_taille, cfg.longueur_totale_mm - demi_taille]),
        }
        return positions

    def position_tag_robot(self) -> Dict[int, np.ndarray]:
        """
        Positions d'étalonnage pour le plan Robot.
        Tag 22: coin haut-droit  (50, 50)
        Tag 23: coin haut-gauche (2950, 50)
        Tag 20: coin bas-droit   (50, 1500)  <- MODIFIÉ
        Tag 21: coin bas-gauche  (2950, 1500) <- MODIFIÉ
        """
        cfg = self.config
        demi_taille = cfg.tag_taille_mm / 2
        positions = {
            22: np.array([demi_taille, demi_taille]),                    # (50, 50)
            23: np.array([cfg.largeur_totale_mm - demi_taille, demi_taille]),  # (2950, 50)
            20: np.array([demi_taille, 1500.0]),                         # (50, 1500)
            21: np.array([cfg.largeur_totale_mm - demi_taille, 1500.0]), # (2950, 1500)
        }
        return positions

    def tag_vers_coin(self, tag_id: int) -> int:
        num_coin = {22: 3, 23: 2, 21: 1, 20: 0}
        return num_coin[tag_id]

    # ── Mémoire des tags ──────────────────────────────────────────────────────

    def update_tag_memory(self, detected_tags: Dict[int, np.ndarray]):
        for tag_id in self.config.tag_reference:
            if tag_id in detected_tags:
                self.tag_memory[tag_id].append(detected_tags[tag_id])

    def get_tag_memory(self, tag_id: int) -> Optional[np.ndarray]:
        if self.tag_memory[tag_id]:
            return np.mean(np.array(self.tag_memory[tag_id]), axis=0)
        return None

    # ── Homographie plan sol ──────────────────────────────────────────────────

    def calcul_homographie(self, detected_tags: Dict[int, np.ndarray]) -> bool:
        self.update_tag_memory(detected_tags)
        position_ref = self.position_tag_reference()
        src_points, dst_points = [], []

        for tag_id in self.config.tag_reference:
            corner_idx = self.tag_vers_coin(tag_id)
            coin_tag = detected_tags.get(tag_id)
            if coin_tag is None:
                coin_tag = self.get_tag_memory(tag_id)
            if coin_tag is None:
                continue
            if tag_id in position_ref:
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

    def calcul_homographie_3(self, src_points, dst_points, detected_tags, position_ref) -> bool:
        id_manquant = next((id for id in self.config.tag_reference if id not in detected_tags), None)
        if id_manquant is None:
            return False

        dst_points.append(position_ref[id_manquant])
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

    # ── Homographie plan Noisette ─────────────────────────────────────────────

    def calcul_homographie_elevated(self, calibration_points: Dict[int, np.ndarray]) -> bool:
        if len(calibration_points) != 4:
            return False

        position_elevated = self.position_tag_elevated()
        src_points, dst_points = [], []

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

    # ── Homographie plan Robot ────────────────────────────────────────────────

    def calcul_homographie_robot(self, calibration_points: Dict[int, np.ndarray]) -> bool:
        """
        Calcule l'homographie pour le plan robot à partir des 4 positions calibrées.
        Utilise position_tag_robot() avec les coordonnées spécifiques au plan robot.
        """
        if len(calibration_points) != 4:
            return False

        position_robot = self.position_tag_robot()  # <- MODIFIÉ
        src_points, dst_points = [], []

        for tag_id in [20, 21, 22, 23]:
            if tag_id in calibration_points and tag_id in position_robot:
                centre_tag = np.mean(calibration_points[tag_id], axis=0)
                src_points.append(centre_tag)
                dst_points.append(position_robot[tag_id])  # <- MODIFIÉ

        if len(src_points) == 4:
            src = np.array(src_points, dtype=np.float32)
            dst = np.array(dst_points, dtype=np.float32)
            self.H_cam_to_robot, _ = cv2.findHomography(src, dst)

            if self.H_cam_to_robot is not None:
                self.H_robot_to_cam = np.linalg.inv(self.H_cam_to_robot)
                return True

        return False

    # ── Transformations de points ─────────────────────────────────────────────

    def _transform(self, point: np.ndarray, H: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if H is None:
            return None
        pt = np.array([[point]], dtype=np.float32)
        return cv2.perspectiveTransform(pt, H)[0, 0]

    def point_cam_to_ref(self, p):      return self._transform(p, self.H_cam_to_ref)
    def point_ref_to_cam(self, p):      return self._transform(p, self.H_ref_to_cam)
    def point_cam_to_elevated(self, p): return self._transform(p, self.H_cam_to_elevated)
    def point_elevated_to_cam(self, p): return self._transform(p, self.H_elevated_to_cam)
    def point_ref_to_elevated(self, p): return self._transform(p, self.H_ref_to_elevated)
    def point_elevated_to_ref(self, p): return self._transform(p, self.H_elevated_to_ref)
    def point_cam_to_robot(self, p):    return self._transform(p, self.H_cam_to_robot)
    def point_robot_to_cam(self, p):    return self._transform(p, self.H_robot_to_cam)


# ─────────────────────────────────────────────────────────────────────────────
# DÉTECTEUR ARUCO
# ─────────────────────────────────────────────────────────────────────────────

class ArUcoDetector:
    def __init__(self, dictionary_type=cv2.aruco.DICT_4X4_100):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_type)
        self.detecteur = cv2.aruco.ArucoDetector(self.aruco_dict)

    def detect(self, image: np.ndarray) -> Tuple[List[Tuple[int, np.ndarray]], np.ndarray]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        coins, id, _ = self.detecteur.detectMarkers(gray)
        detected_tags = []
        if id is not None:
            for i, tag_id in enumerate(id.flatten()):
                detected_tags.append((int(tag_id), coins[i][0]))
        return detected_tags, gray


# ─────────────────────────────────────────────────────────────────────────────
# CALIBRATION PLAN NOISETTE
# ─────────────────────────────────────────────────────────────────────────────

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
        data = {str(k): v.tolist() for k, v in self.calibration_points.items()}
        with open(self.calibration_file, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Calibration Noisette sauvegardée dans {self.calibration_file}")

    def load_calibration(self) -> bool:
        try:
            with open(self.calibration_file, 'r') as f:
                data = json.load(f)
            self.calibration_points = {int(k): np.array(v, dtype=np.float32) for k, v in data.items()}
            self.current_index = len(self.calibration_points)
            print(f"Calibration Noisette chargée depuis {self.calibration_file}")
            return True
        except FileNotFoundError:
            print("Aucun fichier de calibration Noisette trouvé")
            return False
        except Exception as e:
            print(f"Erreur lors du chargement de la calibration Noisette: {e}")
            return False


# ─────────────────────────────────────────────────────────────────────────────
# CALIBRATION PLAN ROBOT
# ─────────────────────────────────────────────────────────────────────────────

class CalibrationModeRobot:
    def __init__(self, config: Config):
        self.config = config
        self.calibration_points = {}
        self.sequence = [20, 21, 22, 23]
        self.current_index = 0
        self.calibration_file = "elevated_plane_calibration_robot.json"

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
        data = {str(k): v.tolist() for k, v in self.calibration_points.items()}
        with open(self.calibration_file, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"Calibration Robot sauvegardée dans {self.calibration_file}")

    def load_calibration(self) -> bool:
        try:
            with open(self.calibration_file, 'r') as f:
                data = json.load(f)
            self.calibration_points = {int(k): np.array(v, dtype=np.float32) for k, v in data.items()}
            self.current_index = len(self.calibration_points)
            print(f"Calibration Robot chargée depuis {self.calibration_file}")
            return True
        except FileNotFoundError:
            print("Aucun fichier de calibration Robot trouvé")
            return False
        except Exception as e:
            print(f"Erreur lors du chargement de la calibration Robot: {e}")
            return False


# ─────────────────────────────────────────────────────────────────────────────
# SYSTÈME DE TRACKING PRINCIPAL
# ─────────────────────────────────────────────────────────────────────────────

class ArUcoTrackingSystem:
    def __init__(self, config: Config, matrice_antidstorsion: Optional[str] = None):
        self.config = config
        self.detecteur = ArUcoDetector()
        self.color_detector = ColorDetector(config)
        self.homographie = CalculHomographie(config)

        # Plan Noisette
        self.calibration_mode = CalibrationMode(config)
        self.mode_calibration_active = False
        self.plan_elevated_calcule = False

        # Plan Robot
        self.calibration_mode_robot = CalibrationModeRobot(config)
        self.mode_calibration_robot_active = False
        self.plan_robot_calcule = False

        self.plan_reference_calcule = False

        # Mémoire des objets colorés détectés (filtre passe-bas)
        self.color_memory_jaune = {}  # {id: {'obj': dict, 'frames_missing': int, 'last_seen': int}}
        self.color_memory_bleu = {}
        self.color_object_id_counter = 0
        self.max_frames_missing = 2  # Nombre de frames avant suppression
        self.distance_threshold_mm = 50.0  # Distance max pour considérer 2 objets identiques
        self.frame_counter = 0

        # Mémoire des paires (noisettes) détectées (filtre passe-bas)
        self.pair_memory_jaune = {}  # {id: {'pair': dict, 'frames_missing': int, 'last_seen': int}}
        self.pair_memory_bleu = {}
        self.pair_id_counter = 0
        self.max_frames_missing_pairs = 2  # Nombre de frames avant suppression d'une paire
        self.pair_distance_threshold_mm = 50.0  # Distance max pour considérer 2 paires identiques

        if matrice_antidstorsion:
            self.load_calibration(matrice_antidstorsion)

        self.centre_image = np.array([config.camera_largeur / 2, config.camera_longueur / 2])

    def _draw_text_with_background(self, image, text, position, font=cv2.FONT_HERSHEY_SIMPLEX, 
                                font_scale=0.6, text_color=(0, 255, 255), thickness=2,
                                bg_color=(255, 255, 255), padding=5):
        """
        Dessine du texte avec un fond rectangulaire pour améliorer la lisibilité.
        
        Args:
            image: Image sur laquelle dessiner
            text: Texte à afficher
            position: Tuple (x, y) de la position du texte
            font: Police de caractères OpenCV
            font_scale: Taille de la police
            text_color: Couleur du texte (BGR)
            thickness: Épaisseur du texte
            bg_color: Couleur du fond (BGR) - blanc par défaut
            padding: Espace entre le texte et le bord du rectangle (pixels)
        """
        x, y = position
        
        # Obtenir la taille du texte
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        
        # Calculer les coordonnées du rectangle de fond
        rect_x1 = x - padding
        rect_y1 = y - text_height - padding
        rect_x2 = x + text_width + padding
        rect_y2 = y + baseline + padding
        
        # Dessiner le rectangle de fond blanc
        cv2.rectangle(image, (rect_x1, rect_y1), (rect_x2, rect_y2), bg_color, -1)
        
        # Dessiner le texte par-dessus
        cv2.putText(image, text, (x, y), font, font_scale, text_color, thickness)

    def load_calibration(self, filepath: str):
        try:
            data = np.load(filepath)
            self.camera_matrix = data['mtx']
            self.dist_coeffs = data['dist']
            print(f"Calibration caméra chargée depuis {filepath}")
        except Exception as e:
            print(f"Impossible de charger la calibration caméra: {e}")

    def undistort_image(self, image: np.ndarray) -> np.ndarray:
        return image

    # ── Traitement d'une frame ────────────────────────────────────────────────

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict]:
        results = {
            'homographie_ok': self.plan_reference_calcule,
            'homographie_elevated_ok': self.plan_elevated_calcule,
            'homographie_robot_ok': self.plan_robot_calcule,
            'tag_reference': {},
            'tag_mobile': [],
            'tag_robot_detectes': [],
            'tag_ennemi_detectes': [],
            'objets_colores': [],
            'Liste_noisette_xya': [],
            'Liste_robots_xy': [],
            'calibration_mode': self.mode_calibration_active,
            'calibration_complete': self.calibration_mode.is_complete(),
        }

        undistorted = self.undistort_image(frame)
        undistorted_pour_detection = undistorted.copy()
        undistorted_copie = undistorted.copy()

        detected_tags_list, gray = self.detecteur.detect(undistorted)

        # ── Dessiner les contours des tags détectés ───────────────────────────
        for tag_id, coins in detected_tags_list:
            coins_int = coins.astype(np.int32)
            if tag_id in self.config.tag_reference:
                cv2.polylines(undistorted_copie, [coins_int], True, (255, 0, 0), 1)
            elif tag_id in self.config.tag_mobile:
                color = (255, 0, 0) if tag_id == 36 else (0, 255, 255) if tag_id == 47 else (255, 0, 255)
                cv2.polylines(undistorted_copie, [coins_int], True, color, 2)

        # ── Homographie plan sol ──────────────────────────────────────────────
        tag_reference = {}
        for tag_id, coins in detected_tags_list:
            if tag_id in self.config.tag_reference and tag_id not in tag_reference:
                tag_reference[tag_id] = coins

        if not self.plan_reference_calcule and len(tag_reference) >= 3:
            if self.homographie.calcul_homographie(tag_reference):
                self.plan_reference_calcule = True
                results['homographie_ok'] = True
                ref_positions = self.homographie.position_tag_reference()
                for tid in tag_reference:
                    if tid in ref_positions:
                        results['tag_reference'][tid] = ref_positions[tid].tolist()

        if self.plan_reference_calcule:
            cfg = self.config
            # Rectangle étendu (vert)
            coins_ref = [
                np.array([-cfg.ajout_horizontal_mm, -cfg.ajout_vertical_mm]),
                np.array([cfg.tags_largeur_mm + cfg.ajout_horizontal_mm, -cfg.ajout_vertical_mm]),
                np.array([cfg.tags_largeur_mm + cfg.ajout_horizontal_mm, cfg.tags_longueur_mm + cfg.ajout_vertical_mm]),
                np.array([-cfg.ajout_horizontal_mm, cfg.tags_longueur_mm + cfg.ajout_vertical_mm]),
            ]
            pixels = [self.homographie.point_ref_to_cam(p) for p in coins_ref]
            if all(p is not None for p in pixels):
                rect = np.array([p.astype(int) for p in pixels], dtype=np.int32)
                cv2.polylines(undistorted_copie, [rect], True, (0, 255, 0), 3)

        # ── Affichage des instructions de calibration ─────────────────────────
        if self.mode_calibration_active:
            target = self.calibration_mode.get_current_target()
            if target is not None:
                self._draw_text_with_background(
                    undistorted_copie,
                    f"[NOISETTE] Tag {self.config.tag_calibration_Noisette} au-dessus du tag {target} puis ESPACE",
                    (10, 30), text_color=(0, 255, 255))
                self._draw_text_with_background(
                    undistorted_copie,
                    f"Progression: {self.calibration_mode.current_index}/4",
                    (10, 60), text_color=(0, 255, 255))
            else:
                self._draw_text_with_background(
                    undistorted_copie,
                    "CALIBRATION NOISETTE TERMINEE - Appuyer sur 'S' pour sauvegarder",
                    (10, 30), text_color=(0, 255, 0))

        if self.mode_calibration_robot_active:
            target = self.calibration_mode_robot.get_current_target()
            if target is not None:
                self._draw_text_with_background(
                    undistorted_copie,
                    f"[ROBOT] Tag {self.config.tag_calibration_robot} au-dessus du tag {target} puis ESPACE",
                    (10, 90), text_color=(0, 200, 255))
                self._draw_text_with_background(
                    undistorted_copie,
                    f"Progression robot: {self.calibration_mode_robot.current_index}/4",
                    (10, 120), text_color=(0, 200, 255))
            else:
                self._draw_text_with_background(
                    undistorted_copie,
                    "CALIBRATION ROBOT TERMINEE - Appuyer sur 'P' pour sauvegarder",
                    (10, 90), text_color=(0, 255, 0))

        # ── Calcul des homographies surélevées si calibration complète ────────
        if self.calibration_mode.is_complete() and not self.plan_elevated_calcule:
            if self.homographie.calcul_homographie_elevated(self.calibration_mode.calibration_points):
                self.plan_elevated_calcule = True
                results['homographie_elevated_ok'] = True

        if self.calibration_mode_robot.is_complete() and not self.plan_robot_calcule:
            if self.homographie.calcul_homographie_robot(self.calibration_mode_robot.calibration_points):
                self.plan_robot_calcule = True
                results['homographie_robot_ok'] = True

        # ── Détection des couleurs (plan Noisette) ────────────────────────────
        if self.plan_elevated_calcule:
            self.frame_counter += 1  # Incrémenter le compteur de frames
            
            color_results = self.color_detector.detect_colors(undistorted_pour_detection)

            cfg = self.config
            coins_elevated_mm = [
                np.array([0, 0]),
                np.array([cfg.largeur_totale_mm, 0]),
                np.array([cfg.largeur_totale_mm, cfg.longueur_totale_mm]),
                np.array([0, cfg.longueur_totale_mm])
            ]

            objets_jaunes_valides, objets_bleus_valides = [], []
            filtre_jaune, filtre_bleu = 0, 0

            for obj in color_results['jaune']:
                pos_elevated = self.homographie.point_cam_to_elevated(obj['centre_pixel'])
                if pos_elevated is not None and self.point_dans_polygone(pos_elevated, coins_elevated_mm):
                    obj['pos_elevated'] = pos_elevated
                    objets_jaunes_valides.append(obj)
                else:
                    filtre_jaune += 1

            for obj in color_results['bleu']:
                pos_elevated = self.homographie.point_cam_to_elevated(obj['centre_pixel'])
                if pos_elevated is not None and self.point_dans_polygone(pos_elevated, coins_elevated_mm):
                    obj['pos_elevated'] = pos_elevated
                    objets_bleus_valides.append(obj)
                else:
                    filtre_bleu += 1

            objets_jaunes_valides = self._split_large_objects(objets_jaunes_valides)
            objets_bleus_valides  = self._split_large_objects(objets_bleus_valides)

            # Appliquer le filtre passe-bas (mémoire)
            objets_jaunes_avec_memoire = self._update_color_memory(objets_jaunes_valides, self.color_memory_jaune, 'jaune')
            objets_bleus_avec_memoire = self._update_color_memory(objets_bleus_valides, self.color_memory_bleu, 'bleu')

            # ANALYSE DE PAIRES : Créer les Noisettes à partir des zones de couleur
            paires_jaunes = self._analyze_color_pairs(objets_jaunes_avec_memoire, 'jaune', objets_bleus_avec_memoire)
            paires_bleues = self._analyze_color_pairs(objets_bleus_avec_memoire, 'bleu', objets_jaunes_avec_memoire)

            # Appliquer le filtre passe-bas sur les paires (noisettes)
            paires_jaunes_avec_memoire = self._update_pair_memory(paires_jaunes, self.pair_memory_jaune, 'jaune')
            paires_bleues_avec_memoire = self._update_pair_memory(paires_bleues, self.pair_memory_bleu, 'bleu')

            # Dessiner les paires (Noisettes)
            for paire in paires_jaunes_avec_memoire:
                self._draw_color_pair(paire, undistorted_copie, (0, 255, 255))
                results['objets_colores'].append(paire)
            for paire in paires_bleues_avec_memoire:
                self._draw_color_pair(paire, undistorted_copie, (140, 91, 0))
                results['objets_colores'].append(paire)

            # Affichage uniquement des objets individuels (avec mémoire)
            for obj in objets_jaunes_avec_memoire:
                if obj.get('pos_elevated') is not None:
                    self._process_colored_object(obj, undistorted_copie, results, (0, 255, 255))
            for obj in objets_bleus_avec_memoire:
                if obj.get('pos_elevated') is not None:
                    self._process_colored_object(obj, undistorted_copie, results, (140, 91, 0))

            # Construction de Liste_noisette_xya
            Liste_noisette_xya = []
            for paire in paires_jaunes_avec_memoire:
                Liste_noisette_xya.append([
                    int(paire['centre_mm'][0]),
                    int(paire['centre_mm'][1]),
                    int(paire['angle_deg']),
                    'J'
                ])
            for paire in paires_bleues_avec_memoire:
                Liste_noisette_xya.append([
                    int(paire['centre_mm'][0]),
                    int(paire['centre_mm'][1]),
                    int(paire['angle_deg']),
                    'B'
                ])
            results['Liste_noisette_xya'] = Liste_noisette_xya

            # Rectangle étendu plan surélevé Noisette (cyan)
            cfg = self.config
            coins_elev = [
                np.array([0, 0]),
                np.array([cfg.largeur_totale_mm, 0]),
                np.array([cfg.largeur_totale_mm, cfg.longueur_totale_mm]),
                np.array([0, cfg.longueur_totale_mm]),
            ]
            pixels_elev = [self.homographie.point_elevated_to_cam(p) for p in coins_elev]
            if all(p is not None for p in pixels_elev):
                rect_elev = np.array([p.astype(int) for p in pixels_elev], dtype=np.int32)
                cv2.polylines(undistorted_copie, [rect_elev], True, (255, 255, 0), 3)

        # ── Dessin du rectangle étendu du plan Robot (orange) ────────────────
        if self.plan_robot_calcule:
            cfg = self.config
            # Les coins du plan Robot : haut complet (y=0), bas tronqué (y=1500)
            coins_robot = [
                np.array([0, 0]),                        # Coin haut-droit
                np.array([cfg.largeur_totale_mm, 0]),   # Coin haut-gauche
                np.array([cfg.largeur_totale_mm, 1500.0]), # Coin bas-gauche (tronqué à y=1500)
                np.array([0, 1500.0]),                   # Coin bas-droit (tronqué à y=1500)
            ]
            pixels_robot = [self.homographie.point_robot_to_cam(p) for p in coins_robot]
            if all(p is not None for p in pixels_robot):
                rect_robot = np.array([p.astype(int) for p in pixels_robot], dtype=np.int32)
                cv2.polylines(undistorted_copie, [rect_robot], True, (0, 165, 255), 3)  # Orange

        # ── Détection des tags robot et ennemi (plan Robot) ───────────────────
        if self.plan_robot_calcule:
            cfg = self.config
            # Zone de filtrage : rectangle complet en largeur, tronqué à y=1500 en hauteur
            for tag_id, coins in detected_tags_list:
                est_robot  = tag_id in self.config.tag_robot
                est_ennemi = tag_id in self.config.tag_ennemi
                if not (est_robot or est_ennemi):
                    continue

                centre_pixel = np.mean(coins, axis=0)
                pos_robot = self.homographie.point_cam_to_robot(centre_pixel)

                if pos_robot is None:
                    continue

                # Calcul de l'angle du tag dans le plan robot
                # Transformer les 4 coins du tag dans le plan robot
                coins_robot_plan = []
                for coin_pixel in coins:
                    coin_mm = self.homographie.point_cam_to_robot(coin_pixel)
                    if coin_mm is not None:
                        coins_robot_plan.append(coin_mm)
                
                # Calculer l'angle si les 4 coins sont disponibles
                angle_deg = 0.0
                if len(coins_robot_plan) == 4:
                    # Vecteur du coin 0 vers coin 1 (côté du tag)
                    vec_x = coins_robot_plan[1][0] - coins_robot_plan[0][0]
                    vec_y = coins_robot_plan[1][1] - coins_robot_plan[0][1]
                    # Angle en degrés par rapport à l'axe X horizontal
                    angle_rad = np.arctan2(vec_y, vec_x)
                    angle_deg = np.degrees(angle_rad)

                # Vert pour nos robots, rouge pour les ennemis
                color_display = (0, 255, 0) if est_robot else (0, 0, 255)
                label = "ROBOT" if est_robot else "ENNEMI"

                # Contour et centre
                cv2.polylines(undistorted_copie, [coins.astype(np.int32)], True, color_display, 2)
                cv2.circle(undistorted_copie, tuple(centre_pixel.astype(int)), 6, color_display, -1)

                # Carré projeté
                demi = self.config.tag_taille_mm_robot / 2
                corners_robot_mm = np.array([
                    pos_robot + np.array([-demi, -demi]),
                    pos_robot + np.array([ demi, -demi]),
                    pos_robot + np.array([ demi,  demi]),
                    pos_robot + np.array([-demi,  demi]),
                ], dtype=np.float32)

                projected = [self.homographie.point_robot_to_cam(c) for c in corners_robot_mm]
                if all(p is not None for p in projected):
                    cv2.polylines(undistorted_copie,
                                  [np.array(projected, dtype=np.int32)],
                                  True, color_display, 2)
                
                # Dessiner une flèche indiquant l'orientation du tag
                if len(coins_robot_plan) == 4:
                    arrow_length = 40
                    end_x = int(centre_pixel[0] + arrow_length * np.cos(angle_rad))
                    end_y = int(centre_pixel[1] + arrow_length * np.sin(angle_rad))
                    cv2.arrowedLine(undistorted_copie, 
                                   tuple(centre_pixel.astype(int)), 
                                   (end_x, end_y), 
                                   color_display, 3, tipLength=0.3)

                entry = {
                    'tag_id': tag_id,
                    'position_mm': pos_robot.tolist(),
                    'pixel_center': centre_pixel.tolist(),
                    'angle_deg': float(angle_deg),
                }
                if est_robot:
                    results['tag_robot_detectes'].append(entry)
                else:
                    results['tag_ennemi_detectes'].append(entry)

                #print(f"{label} Tag {tag_id}: X={pos_robot[0]:.1f}mm, Y={pos_robot[1]:.1f}mm, Angle={angle_deg:.1f}°")

            # Construction de Liste_robots_xy
            # Format : [tag_id, X_mm, Y_mm, angle_deg, 'R' ou 'E']
            Liste_robots_xy = []
            for entry in results['tag_robot_detectes']:
                Liste_robots_xy.append([
                    entry['tag_id'],
                    int(entry['position_mm'][0]),
                    int(entry['position_mm'][1]),
                    int(entry['angle_deg']),
                    'R'
                ])
            for entry in results['tag_ennemi_detectes']:
                Liste_robots_xy.append([
                    entry['tag_id'],
                    int(entry['position_mm'][0]),
                    int(entry['position_mm'][1]),
                    int(entry['angle_deg']),
                    'E'
                ])
            results['Liste_robots_xy'] = Liste_robots_xy

        # ── Tags mobiles (tag_mobile, plan Noisette) ──────────────────────────
        tag_mobile_list = [(tid, c) for tid, c in detected_tags_list if tid in self.config.tag_mobile]

        if tag_mobile_list and self.plan_elevated_calcule:
            for idx, (tag_id, coins) in enumerate(tag_mobile_list):
                color = (255, 0, 0) if tag_id == 36 else (0, 255, 255) if tag_id == 47 else (255, 0, 255)
                cv2.polylines(undistorted_copie, [coins.astype(np.int32)], True, color, 2)

                centre_pixel = np.mean(coins, axis=0)
                pos_elevated = self.homographie.point_cam_to_elevated(centre_pixel)

                if pos_elevated is not None:
                    demi_taille = self.config.tag_taille_mm_robot / 2
                    coins_mm = np.array([
                        pos_elevated + np.array([-demi_taille, -demi_taille]),
                        pos_elevated + np.array([ demi_taille, -demi_taille]),
                        pos_elevated + np.array([ demi_taille,  demi_taille]),
                        pos_elevated + np.array([-demi_taille,  demi_taille]),
                    ], dtype=np.float32)

                    projected = [self.homographie.point_elevated_to_cam(c) for c in coins_mm]
                    if all(p is not None for p in projected):
                        cv2.polylines(undistorted_copie,
                                      [np.array(projected, dtype=np.int32)],
                                      True, color, 2)

                    results['tag_mobile'].append({
                        'tag_id': tag_id,
                        'position_mm': pos_elevated.tolist(),
                        'pixel_center': centre_pixel.tolist()
                    })

        # ── Aide à l'écran ────────────────────────────────────────────────────
        if not self.mode_calibration_active and not self.mode_calibration_robot_active:
            cv2.putText(undistorted_copie,
                "'E' plan noisette | 'R' plan robot | 'L' charger | 'Q' quitter",
                (10, undistorted_copie.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return undistorted_copie, results

    # ── Capture de calibration ────────────────────────────────────────────────

    def handle_calibration_capture(self, detected_tags_list: List[Tuple[int, np.ndarray]]) -> bool:
        for tag_id, tag_corners in detected_tags_list:
            if tag_id == self.config.tag_calibration_Noisette:
                success = self.calibration_mode.capture_position(tag_corners)
                if success:
                    target = self.calibration_mode.get_current_target()
                    if target is None:
                        print("Calibration Noisette complète ! Appuyer sur 'S' pour sauvegarder.")
                    else:
                        print(f"[Noisette] Position capturée ! Placer le tag au-dessus du tag {target} et appuyer sur ESPACE")
                    return True
        print(f"Tag {self.config.tag_calibration_Noisette} non détecté")
        return False

    def handle_calibration_capture_robot(self, detected_tags_list: List[Tuple[int, np.ndarray]]) -> bool:
        for tag_id, tag_corners in detected_tags_list:
            if tag_id == self.config.tag_calibration_robot:
                success = self.calibration_mode_robot.capture_position(tag_corners)
                if success:
                    target = self.calibration_mode_robot.get_current_target()
                    if target is None:
                        print("Calibration Robot complète ! Appuyer sur 'P' pour sauvegarder.")
                    else:
                        print(f"[Robot] Position capturée ! Placer le tag au-dessus du tag {target} et appuyer sur ESPACE")
                    return True
        print(f"Tag {self.config.tag_calibration_robot} non détecté")
        return False

    # ── Méthodes internes ─────────────────────────────────────────────────────

    def _process_colored_object(self, obj: Dict, image: np.ndarray, results: Dict, color_bgr: Tuple):
        cv2.drawContours(image, [obj['contour']], -1, color_bgr, 3)
        centre_pixel = obj['centre_pixel']
        cv2.circle(image, tuple(centre_pixel.astype(int)), 8, color_bgr, -1)
        cv2.circle(image, tuple(centre_pixel.astype(int)), 10, (255, 255, 255), 2)

        pos_elevated_tags = obj.get('pos_elevated')
        if pos_elevated_tags is not None:
            cross_size = 15
            cx, cy = int(centre_pixel[0]), int(centre_pixel[1])
            cv2.line(image, (cx - cross_size, cy), (cx + cross_size, cy), (255, 255, 255), 2)
            cv2.line(image, (cx, cy - cross_size), (cx, cy + cross_size), (255, 255, 255), 2)

            results['objets_colores'].append({
                'couleur': obj['couleur'],
                'elevated_extended_mm': pos_elevated_tags.tolist(),
                'elevated_tags_mm': pos_elevated_tags.tolist(),
                'pixel_center': centre_pixel.tolist(),
                'aire_pixels': obj['aire']
            })

    def point_dans_polygone(self, point: np.ndarray, coins_polygone_mm: List[np.ndarray]) -> bool:
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

    def _update_color_memory(self, detected_objects: List[Dict], color_memory: Dict, color_name: str) -> List[Dict]:
        """
        Met à jour la mémoire des objets colorés avec un filtre passe-bas.
        Conserve les objets même s'ils ne sont plus détectés pendant quelques frames.
        
        Args:
            detected_objects: Liste des objets détectés dans la frame actuelle
            color_memory: Dictionnaire de mémoire (self.color_memory_jaune ou self.color_memory_bleu)
            color_name: Nom de la couleur ('jaune' ou 'bleu')
            
        Returns:
            Liste des objets à afficher (détectés + mémoire récente)
        """
        current_frame = self.frame_counter
        matched_ids = set()
        result_objects = []
        
        # Filtrer les objets détectés pour ne garder que ceux avec pos_elevated valide
        valid_detected = [obj for obj in detected_objects if obj.get('pos_elevated') is not None]
        
        #print(f"[{color_name}] Frame {current_frame}: {len(valid_detected)} objets détectés, {len(color_memory)} en mémoire")
        
        # 1. Associer les objets détectés aux objets en mémoire
        for obj in valid_detected:
            pos_current = obj['pos_elevated']
            best_match_id = None
            best_distance = self.distance_threshold_mm
            
            # Chercher l'objet le plus proche en mémoire
            for obj_id, mem_data in color_memory.items():
                mem_obj = mem_data['obj']
                if mem_obj.get('pos_elevated') is None:
                    continue
                    
                pos_memory = mem_obj['pos_elevated']
                distance = np.linalg.norm(pos_current - pos_memory)
                
                if distance < best_distance:
                    best_distance = distance
                    best_match_id = obj_id
            
            if best_match_id is not None:
                # Objet existant : mettre à jour
                color_memory[best_match_id]['obj'] = obj
                color_memory[best_match_id]['frames_missing'] = 0
                color_memory[best_match_id]['last_seen'] = current_frame
                matched_ids.add(best_match_id)
                result_objects.append(obj)
                #print(f"  - Objet ID={best_match_id} mis à jour (distance={best_distance:.1f}mm)")
            else:
                # Nouvel objet : ajouter à la mémoire
                new_id = self.color_object_id_counter
                self.color_object_id_counter += 1
                color_memory[new_id] = {
                    'obj': obj,
                    'frames_missing': 0,
                    'last_seen': current_frame
                }
                matched_ids.add(new_id)
                result_objects.append(obj)
                #print(f"  - Nouvel objet ID={new_id} créé à [{pos_current[0]:.0f}, {pos_current[1]:.0f}]mm")
        
        # 2. Incrémenter le compteur de frames manquantes pour les objets non détectés
        ids_to_remove = []
        for obj_id, mem_data in color_memory.items():
            if obj_id not in matched_ids:
                mem_data['frames_missing'] += 1
                
                # Si l'objet n'a pas été vu depuis trop longtemps, le supprimer
                if mem_data['frames_missing'] > self.max_frames_missing:
                    ids_to_remove.append(obj_id)
                    #print(f"  - Objet ID={obj_id} supprimé (absent depuis {mem_data['frames_missing']} frames)")
                else:
                    # Conserver l'objet en mémoire
                    result_objects.append(mem_data['obj'])
                    #print(f"  - Objet ID={obj_id} conservé en mémoire (absent {mem_data['frames_missing']}/{self.max_frames_missing})")
        
        # 3. Supprimer les objets trop anciens
        for obj_id in ids_to_remove:
            del color_memory[obj_id]
        
        #print(f"  → Total objets affichés: {len(result_objects)}")
        return result_objects

    def _update_pair_memory(self, detected_pairs: List[Dict], pair_memory: Dict, color_name: str) -> List[Dict]:
        """
        Met à jour la mémoire des paires (noisettes) avec un filtre passe-bas.
        Conserve les paires même si elles ne sont plus détectées pendant quelques frames.
        
        Args:
            detected_pairs: Liste des paires détectées dans la frame actuelle
            pair_memory: Dictionnaire de mémoire (self.pair_memory_jaune ou self.pair_memory_bleu)
            color_name: Nom de la couleur ('jaune' ou 'bleu')
            
        Returns:
            Liste des paires à afficher (détectées + mémoire récente)
        """
        current_frame = self.frame_counter
        matched_ids = set()
        result_pairs = []
        
        #print(f"[{color_name} PAIRES] Frame {current_frame}: {len(detected_pairs)} paires détectées, {len(pair_memory)} en mémoire")
        
        # 1. Associer les paires détectées aux paires en mémoire
        for pair in detected_pairs:
            centre_current = np.array(pair['centre_mm'])
            best_match_id = None
            best_distance = self.pair_distance_threshold_mm
            
            # Chercher la paire la plus proche en mémoire
            for pair_id, mem_data in pair_memory.items():
                mem_pair = mem_data['pair']
                centre_memory = np.array(mem_pair['centre_mm'])
                distance = np.linalg.norm(centre_current - centre_memory)
                
                if distance < best_distance:
                    best_distance = distance
                    best_match_id = pair_id
            
            if best_match_id is not None:
                # Paire existante : mettre à jour
                pair_memory[best_match_id]['pair'] = pair
                pair_memory[best_match_id]['frames_missing'] = 0
                pair_memory[best_match_id]['last_seen'] = current_frame
                matched_ids.add(best_match_id)
                result_pairs.append(pair)
                #print(f"  - Paire ID={best_match_id} mise à jour (distance={best_distance:.1f}mm)")
            else:
                # Nouvelle paire : ajouter à la mémoire
                new_id = self.pair_id_counter
                self.pair_id_counter += 1
                pair_memory[new_id] = {
                    'pair': pair,
                    'frames_missing': 0,
                    'last_seen': current_frame
                }
                matched_ids.add(new_id)
                result_pairs.append(pair)
                #print(f"  - Nouvelle paire ID={new_id} créée à [{centre_current[0]:.0f}, {centre_current[1]:.0f}]mm")
        
        # 2. Incrémenter le compteur de frames manquantes pour les paires non détectées
        ids_to_remove = []
        for pair_id, mem_data in pair_memory.items():
            if pair_id not in matched_ids:
                mem_data['frames_missing'] += 1
                
                # Si la paire n'a pas été vue depuis trop longtemps, la supprimer
                if mem_data['frames_missing'] > self.max_frames_missing_pairs:
                    ids_to_remove.append(pair_id)
                    #print(f"  - Paire ID={pair_id} supprimée (absente depuis {mem_data['frames_missing']} frames)")
                else:
                    # Conserver la paire en mémoire
                    result_pairs.append(mem_data['pair'])
                    #print(f"  - Paire ID={pair_id} conservée en mémoire (absente {mem_data['frames_missing']}/{self.max_frames_missing_pairs})")
        
        # 3. Supprimer les paires trop anciennes
        for pair_id in ids_to_remove:
            del pair_memory[pair_id]
        
        #print(f"  → Total paires affichées: {len(result_pairs)}")
        return result_pairs

    def _split_large_objects(self, objects: List[Dict]) -> List[Dict]:
        result = []
        for obj in objects:
            contour = obj['contour']
            rect = cv2.minAreaRect(contour)
            (cx, cy), (w, h), angle = rect
            if w < h:
                w, h = h, w
                angle += 90

            box_pixels = cv2.boxPoints(rect)
            coins_mm = []
            for pt in box_pixels:
                pt_mm = self.homographie.point_cam_to_elevated(np.array(pt))
                if pt_mm is not None:
                    coins_mm.append(pt_mm)

            if len(coins_mm) != 4:
                result.append(obj)
                continue

            coins_mm = np.array(coins_mm)
            largeur_mm = np.linalg.norm(coins_mm[1] - coins_mm[0])
            hauteur_mm = np.linalg.norm(coins_mm[2] - coins_mm[1])
            if largeur_mm < hauteur_mm:
                largeur_mm, hauteur_mm = hauteur_mm, largeur_mm

            # Déterminer si c'est un groupe de noisettes collées
            # Les noisettes sont espacées de 50mm chacune
            # 1 noisette seule : ~50mm de large
            # 2 noisettes côte à côte : ~100mm de large (50mm + 50mm)
            # 3 noisettes alignées : ~150mm de large (50mm + 50mm + 50mm)
            # 4 noisettes alignées : ~200mm de large (50mm + 50mm + 50mm + 50mm)
            
            nb_noisettes = 0
            
            if 30 < hauteur_mm < 70:  # Hauteur cohérente avec des noisettes
                if 70 < largeur_mm < 130:  # Environ 100mm = 2 noisettes
                    nb_noisettes = 2
                    #print(f"  → Détecté : Groupe de 2 noisettes (largeur={largeur_mm:.1f}mm)")
                elif 130 < largeur_mm < 180:  # Environ 150mm = 3 noisettes
                    nb_noisettes = 3
                    #print(f"  → Détecté : Groupe de 3 noisettes (largeur={largeur_mm:.1f}mm)")
                elif 180 < largeur_mm < 230:  # Environ 200mm = 4 noisettes
                    nb_noisettes = 4
                    #print(f"  → Détecté : Groupe de 4 noisettes (largeur={largeur_mm:.1f}mm)")
            
            # Si ce n'est pas un groupe de noisettes collées, garder tel quel
            if nb_noisettes == 0:
                result.append(obj)
                continue

            # Calculer la direction principale (axe le plus long du rectangle)
            centre_mm = obj['pos_elevated']
            v1 = coins_mm[1] - coins_mm[0]
            v2 = coins_mm[2] - coins_mm[1]
            direction = v1 / np.linalg.norm(v1) if np.linalg.norm(v1) > np.linalg.norm(v2) else v2 / np.linalg.norm(v2)

            # Créer les positions pour chaque noisette
            # Les noisettes individuelles font 50mm de large et sont espacées de 50mm entre leurs centres
            # Pour 2 noisettes : centres à -25, +25 (espacement total de 50mm)
            # Pour 3 noisettes : centres à -50, 0, +50 (espacements de 50mm)
            # Pour 4 noisettes : centres à -75, -25, +25, +75 (espacements de 50mm)
            
            if nb_noisettes == 2:
                offsets = [-25.0, 25.0]
            elif nb_noisettes == 3:
                offsets = [-50.0, 0.0, 50.0]
            elif nb_noisettes == 4:
                offsets = [-75.0, -25.0, 25.0, 75.0]
            else:
                offsets = []
            
            # Créer un objet pour chaque noisette
            aire_par_noisette = obj['aire'] / nb_noisettes
            
            for offset in offsets:
                pos_mm = centre_mm + direction * offset
                centre_pixel = self.homographie.point_elevated_to_cam(pos_mm)
                
                if centre_pixel is not None:
                    result.append({
                        'centre_pixel': centre_pixel,
                        'contour': contour,
                        'aire': aire_par_noisette,
                        'couleur': obj['couleur'],
                        'pos_elevated': pos_mm
                    })

        return result

    def _analyze_color_pairs(self, objects, color_name, other_objects, tolerance_mm=20.0):
        target_distance = 100.0
        min_distance = target_distance - tolerance_mm
        max_distance = target_distance + tolerance_mm
        pairs, used_indices = [], set()

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
                distance = np.linalg.norm(pos2 - pos1)

                if min_distance <= distance <= max_distance:
                    # NOUVELLE VÉRIFICATION : Pas d'objet de même couleur entre obj1 et obj2
                    if self._check_same_color_between(pos1, pos2, objects, i, j, color_name):
                        continue  # Il y a un objet de même couleur entre les deux → pas de paire
                    
                    # Vérification originale : pas d'interférence avec l'autre couleur
                    if self._check_interference(pos1, pos2, other_objects):
                        continue
                    
                    centre = (pos1 + pos2) / 2
                    delta_x, delta_y = pos2[0] - pos1[0], pos2[1] - pos1[1]
                    angle_deg = np.degrees(np.arctan2(delta_y, delta_x))
                    used_indices.add(i)
                    used_indices.add(j)
                    pairs.append({
                        'couleur': color_name,
                        'objet1': {'position_mm': pos1.tolist(), 'aire_pixels': obj1['aire']},
                        'objet2': {'position_mm': pos2.tolist(), 'aire_pixels': obj2['aire']},
                        'centre_mm': centre.tolist(),
                        'distance_mm': float(distance),
                        'angle_deg': float(angle_deg),
                        'pixel_centers': [obj1['centre_pixel'].tolist(), obj2['centre_pixel'].tolist()]
                    })
                    break

        # NOUVELLE ÉTAPE : Détecter les configurations partielles (3 objets sur 4 détectés)
        partial_pairs = self._detect_partial_configurations(objects, used_indices, color_name, other_objects)
        pairs.extend(partial_pairs)

        return pairs

    def _detect_partial_configurations(self, objects: List[Dict], used_indices: set, 
                                       color_name: str, other_objects: List[Dict]) -> List[Dict]:
        """
        Détecte les configurations partielles où seulement 3 des 4 zones de couleur sont détectées
        pour un groupe de 2 noisettes côte à côte.
        
        Configuration attendue pour 2 noisettes côte à côte :
        ●─50mm─●     ●─50mm─●
        Noisette1    Noisette2
        (100mm d'écart entre les noisettes)
        
        Configurations partielles possibles (3 objets détectés) :
        - Cas A : Manque obj4 → ●─50─●─100─●  
        - Cas B : Manque obj3 → ●─100─●─50─●
        - Cas C : Manque obj2 → ●─150─●
        - Cas D : Manque obj1 → ●─150─●
        
        Args:
            objects: Liste de tous les objets de la couleur
            used_indices: Indices des objets déjà utilisés en paires
            color_name: Nom de la couleur
            other_objects: Objets de l'autre couleur (pour vérifier les interférences)
            
        Returns:
            Liste des paires détectées à partir de configurations partielles
        """
        partial_pairs = []
        remaining_objects = [obj for idx, obj in enumerate(objects) if idx not in used_indices]
        
        if len(remaining_objects) < 3:
            return partial_pairs
        
        # Parcourir tous les triplets d'objets non utilisés
        for i in range(len(remaining_objects)):
            for j in range(i + 1, len(remaining_objects)):
                for k in range(j + 1, len(remaining_objects)):
                    obj1 = remaining_objects[i]
                    obj2 = remaining_objects[j]
                    obj3 = remaining_objects[k]
                    
                    pos1 = obj1['pos_elevated']
                    pos2 = obj2['pos_elevated']
                    pos3 = obj3['pos_elevated']
                    
                    # Calculer les distances entre les 3 objets
                    d12 = np.linalg.norm(pos2 - pos1)
                    d23 = np.linalg.norm(pos3 - pos2)
                    d13 = np.linalg.norm(pos3 - pos1)
                    
                    # Vérifier si les 3 objets sont approximativement alignés
                    if not self._are_aligned(pos1, pos2, pos3, max_deviation_mm=30.0):
                        continue
                    
                    # CAS A : Configuration ●─50─●─100─● (manque obj4 à droite)
                    # Distances : 50mm, 100mm, 150mm
                    if (40 < d12 < 60 and 90 < d23 < 110 and 140 < d13 < 160):
                        # Les 3 objets forment une configuration partielle
                        # Noisette 1 = obj1 + obj2, Noisette 2 = obj3 + [obj4 manquant]
                        centre_noisette1 = (pos1 + pos2) / 2
                        centre_noisette2 = pos3 + (pos3 - pos2)  # Extrapoler la position de obj4
                        
                        centre_paire = (centre_noisette1 + centre_noisette2) / 2
                        delta = centre_noisette2 - centre_noisette1
                        angle_deg = np.degrees(np.arctan2(delta[1], delta[0]))
                        
                        partial_pairs.append({
                            'couleur': color_name,
                            'objet1': {'position_mm': centre_noisette1.tolist(), 'aire_pixels': (obj1['aire'] + obj2['aire']) / 2},
                            'objet2': {'position_mm': centre_noisette2.tolist(), 'aire_pixels': obj3['aire']},
                            'centre_mm': centre_paire.tolist(),
                            'distance_mm': float(np.linalg.norm(delta)),
                            'angle_deg': float(angle_deg),
                            'pixel_centers': [obj1['centre_pixel'].tolist(), obj3['centre_pixel'].tolist()],
                            'partial': True,  # Marqueur pour indiquer que c'est une configuration partielle
                            'missing': 'obj4'
                        })
                        #print(f"  ✓ [{color_name}] Configuration partielle détectée (CAS A - manque obj4)")
                        return partial_pairs
                    
                    # CAS B : Configuration ●─100─●─50─● (manque obj3 au milieu-droite)
                    # Distances : 100mm, 50mm, 150mm
                    if (90 < d12 < 110 and 40 < d23 < 60 and 140 < d13 < 160):
                        centre_noisette1 = pos1 + (pos2 - pos1) / 2  # Centre entre obj1 et position extrapolée
                        centre_noisette2 = (pos2 + pos3) / 2
                        
                        centre_paire = (centre_noisette1 + centre_noisette2) / 2
                        delta = centre_noisette2 - centre_noisette1
                        angle_deg = np.degrees(np.arctan2(delta[1], delta[0]))
                        
                        partial_pairs.append({
                            'couleur': color_name,
                            'objet1': {'position_mm': centre_noisette1.tolist(), 'aire_pixels': obj1['aire']},
                            'objet2': {'position_mm': centre_noisette2.tolist(), 'aire_pixels': (obj2['aire'] + obj3['aire']) / 2},
                            'centre_mm': centre_paire.tolist(),
                            'distance_mm': float(np.linalg.norm(delta)),
                            'angle_deg': float(angle_deg),
                            'pixel_centers': [obj1['centre_pixel'].tolist(), obj3['centre_pixel'].tolist()],
                            'partial': True,
                            'missing': 'obj3'
                        })
                        #print(f"  ✓ [{color_name}] Configuration partielle détectée (CAS B - manque obj3)")
                        return partial_pairs
                    
                    # CAS C/D : Configuration ●─150─● (manque obj2 ou obj4)
                    # Distance : 150mm entre obj1 et obj2, obj3 est un autre objet non lié
                    # On cherche 2 objets espacés de ~150mm
                    
        # Recherche de paires espacées de 150mm (2 noisettes sans 2 zones intermédiaires)
        for i in range(len(remaining_objects)):
            for j in range(i + 1, len(remaining_objects)):
                obj1 = remaining_objects[i]
                obj2 = remaining_objects[j]
                pos1 = obj1['pos_elevated']
                pos2 = obj2['pos_elevated']
                distance = np.linalg.norm(pos2 - pos1)
                
                # Configuration ●─150mm─● (2 noisettes avec 2 zones manquantes au milieu)
                if 140 < distance < 160:
                    # Vérifier qu'il n'y a pas d'autre objet entre les deux
                    has_object_between = False
                    for k in range(len(remaining_objects)):
                        if k == i or k == j:
                            continue
                        obj_k = remaining_objects[k]
                        pos_k = obj_k['pos_elevated']
                        
                        vec = pos2 - pos1
                        vec_unit = vec / np.linalg.norm(vec)
                        to_k = pos_k - pos1
                        proj = np.dot(to_k, vec_unit)
                        
                        if 0 < proj < distance:
                            perp_dist = np.linalg.norm(pos_k - (pos1 + proj * vec_unit))
                            if perp_dist < 40:
                                has_object_between = True
                                break
                    
                    if not has_object_between:
                        centre_paire = (pos1 + pos2) / 2
                        delta = pos2 - pos1
                        angle_deg = np.degrees(np.arctan2(delta[1], delta[0]))
                        
                        partial_pairs.append({
                            'couleur': color_name,
                            'objet1': {'position_mm': pos1.tolist(), 'aire_pixels': obj1['aire']},
                            'objet2': {'position_mm': pos2.tolist(), 'aire_pixels': obj2['aire']},
                            'centre_mm': centre_paire.tolist(),
                            'distance_mm': float(distance),
                            'angle_deg': float(angle_deg),
                            'pixel_centers': [obj1['centre_pixel'].tolist(), obj2['centre_pixel'].tolist()],
                            'partial': True,
                            'missing': 'obj2_and_obj3'
                        })
                        #print(f"  ✓ [{color_name}] Configuration partielle détectée (CAS C/D - manque 2 zones centrales)")
                        return partial_pairs
        
        return partial_pairs

    def _are_aligned(self, pos1: np.ndarray, pos2: np.ndarray, pos3: np.ndarray, 
                     max_deviation_mm: float = 30.0) -> bool:
        """
        Vérifie si 3 points sont approximativement alignés.
        
        Args:
            pos1, pos2, pos3: Positions des 3 objets
            max_deviation_mm: Déviation maximale perpendiculaire tolérée
            
        Returns:
            True si les 3 points sont alignés
        """
        # Vecteur de pos1 vers pos3
        vec = pos3 - pos1
        vec_length = np.linalg.norm(vec)
        
        if vec_length == 0:
            return False
        
        vec_unit = vec / vec_length
        
        # Projection de pos2 sur l'axe pos1→pos3
        to_pos2 = pos2 - pos1
        projection_length = np.dot(to_pos2, vec_unit)
        projection_point = pos1 + projection_length * vec_unit
        
        # Distance perpendiculaire
        perpendicular_distance = np.linalg.norm(pos2 - projection_point)
        
        return perpendicular_distance < max_deviation_mm

    def _check_same_color_between(self, pos1: np.ndarray, pos2: np.ndarray, same_color_objects: List[Dict], 
                                    idx1: int, idx2: int, color_name: str, margin_mm: float = 40.0) -> bool:
        """
        Vérifie s'il existe un objet de même couleur entre pos1 et pos2.
        
        Args:
            pos1: Position du premier objet
            pos2: Position du deuxième objet
            same_color_objects: Liste de tous les objets de la même couleur
            idx1: Index du premier objet dans la liste
            idx2: Index du deuxième objet dans la liste
            color_name: Nom de la couleur (pour les logs)
            margin_mm: Distance maximale perpendiculaire pour considérer qu'un objet est "entre" les deux
            
        Returns:
            True s'il y a au moins un objet entre pos1 et pos2
        """
        # Vecteur directeur entre pos1 et pos2
        vec = pos2 - pos1
        vec_length = np.linalg.norm(vec)
        
        if vec_length == 0:
            return False
        
        vec_unit = vec / vec_length
        
        # Parcourir tous les objets de même couleur (sauf pos1 et pos2)
        for k, other_obj in enumerate(same_color_objects):
            # Ignorer les objets pos1 et pos2 eux-mêmes
            if k == idx1 or k == idx2:
                continue
            
            other_pos = other_obj['pos_elevated']
            
            # Vecteur de pos1 vers l'autre objet
            to_other = other_pos - pos1
            
            # Projection sur l'axe pos1→pos2
            projection_length = np.dot(to_other, vec_unit)
            
            # L'objet doit être entre pos1 et pos2 (pas avant ni après)
            if projection_length <= 0 or projection_length >= vec_length:
                continue
            
            # Point projeté sur l'axe pos1→pos2
            projection_point = pos1 + projection_length * vec_unit
            
            # Distance perpendiculaire entre l'objet et l'axe pos1→pos2
            perpendicular_distance = np.linalg.norm(other_pos - projection_point)
            
            # Si l'objet est proche de l'axe (< margin_mm), il est "entre" les deux
            if perpendicular_distance < margin_mm:
                #print(f"  ⚠️ [{color_name}] Objet intermédiaire détecté entre objets {idx1} et {idx2} "
                      #f"(projection={projection_length:.1f}mm, distance_perp={perpendicular_distance:.1f}mm)")
                return True
        
        return False

    def _check_interference(self, pos1, pos2, other_objects):
        if not other_objects:
            return False
        vec = pos2 - pos1
        vec_length = np.linalg.norm(vec)
        if vec_length == 0:
            return False
        vec_unit = vec / vec_length
        margin_mm = vec_length * 1.5

        for other_obj in other_objects:
            other_pos = other_obj['pos_elevated']
            to_other = other_pos - pos1
            projection_length = np.dot(to_other, vec_unit)
            marge_bord = vec_length * 0.15
            if projection_length < marge_bord or projection_length > vec_length - marge_bord:
                continue
            projection_point = pos1 + projection_length * vec_unit
            if np.linalg.norm(other_pos - projection_point) < margin_mm:
                return True
        return False

    def _draw_color_pair(self, paire, image, color_bgr):
        for pixel_center in paire['pixel_centers']:
            cv2.circle(image, tuple(np.array(pixel_center).astype(int)), 6, color_bgr, -1)
        pt1 = tuple(np.array(paire['pixel_centers'][0]).astype(int))
        pt2 = tuple(np.array(paire['pixel_centers'][1]).astype(int))
        cv2.line(image, pt1, pt2, color_bgr, 2)

        centre_mm = np.array(paire['centre_mm'])
        centre_pixel = self.homographie.point_elevated_to_cam(centre_mm)
        if centre_pixel is not None:
            centre_pixel_int = tuple(centre_pixel.astype(int))
            cv2.circle(image, centre_pixel_int, 12, color_bgr, -1)
            cv2.circle(image, centre_pixel_int, 14, (255, 255, 255), 2)
            angle_rad = np.radians(paire['angle_deg'])
            end_x = int(centre_pixel[0] + 40 * np.cos(angle_rad))
            end_y = int(centre_pixel[1] + 40 * np.sin(angle_rad))
            cv2.arrowedLine(image, centre_pixel_int, (end_x, end_y), (255, 255, 255), 2, tipLength=0.3)


# ─────────────────────────────────────────────────────────────────────────────
# DÉTECTEUR DE COULEURS
# ─────────────────────────────────────────────────────────────────────────────

class ColorDetector:
    def __init__(self, config: Config):
        self.config = config
        self.calibration_file = "color_calibration.json"
        
        # Valeurs par défaut
        default_jaune_lower = [17, 105, 205]
        default_jaune_upper = [36, 255, 255]
        default_bleu_lower = [47, 105, 101]
        default_bleu_upper = [135, 255, 255]
        
        # Charger depuis le fichier JSON si disponible
        if not self.load_calibration():
            # Utiliser les valeurs par défaut si le chargement échoue
            self.jaune_lower = np.array(default_jaune_lower)
            self.jaune_upper = np.array(default_jaune_upper)
            self.bleu_lower = np.array(default_bleu_lower)
            self.bleu_upper = np.array(default_bleu_upper)

        print("\n=== ColorDetector Initialisé ===")
        print(f"Jaune RGB cible: {config.couleur_jaune_rgb}")
        print(f"  -> Plage HSV: {self.jaune_lower} à {self.jaune_upper}")
        print(f"Bleu RGB cible: {config.couleur_bleu_rgb}")
        print(f"  -> Plage HSV: {self.bleu_lower} à {self.bleu_upper}")

    def load_calibration(self) -> bool:
        """Charge les valeurs HSV depuis le fichier JSON"""
        try:
            with open(self.calibration_file, 'r') as f:
                data = json.load(f)
            
            self.jaune_lower = np.array(data['jaune_lower'])
            self.jaune_upper = np.array(data['jaune_upper'])
            self.bleu_lower = np.array(data['bleu_lower'])
            self.bleu_upper = np.array(data['bleu_upper'])
            
            print(f"Calibration couleurs chargée depuis {self.calibration_file}")
            return True
        except FileNotFoundError:
            print(f"Aucun fichier de calibration couleurs trouvé, utilisation des valeurs par défaut")
            return False
        except Exception as e:
            print(f"Erreur lors du chargement de la calibration couleurs: {e}")
            return False
    
    def save_calibration(self):
        """Sauvegarde les valeurs HSV dans le fichier JSON"""
        try:
            data = {
                'jaune_lower': self.jaune_lower.tolist(),
                'jaune_upper': self.jaune_upper.tolist(),
                'bleu_lower': self.bleu_lower.tolist(),
                'bleu_upper': self.bleu_upper.tolist()
            }
            
            with open(self.calibration_file, 'w') as f:
                json.dump(data, f, indent=2)
            
            print(f"Calibration couleurs sauvegardée dans {self.calibration_file}")
            return True
        except Exception as e:
            print(f"Erreur lors de la sauvegarde de la calibration couleurs: {e}")
            return False

    def detect_colors(self, image: np.ndarray) -> Dict[str, List[Dict]]:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask_jaune = cv2.inRange(hsv, self.jaune_lower, self.jaune_upper)
        mask_bleu  = cv2.inRange(hsv, self.bleu_lower,  self.bleu_upper)
        return {
            'jaune': self._process_mask(mask_jaune, 'jaune'),
            'bleu':  self._process_mask(mask_bleu,  'bleu'),
        }

    def _process_mask(self, mask: np.ndarray, color_name: str) -> List[Dict]:
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_objects = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.config.taille_min_contour < area < self.config.taille_max_contour:
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
        # Réduire la taille de l'image pour l'affichage
        scale_factor = 0.5
        display_width = int(image.shape[1] * scale_factor)
        display_height = int(image.shape[0] * scale_factor)

        cv2.namedWindow('Calibration HSV')
        cv2.namedWindow('Image Originale')

        # Redimensionner les fenêtres
        cv2.resizeWindow('Calibration HSV', display_width, display_height)
        cv2.resizeWindow('Image Originale', display_width, display_height)

        # Position et dimensions du bouton "Sauvegarder" (en haut à gauche)
        button_x = 10
        button_y = 10
        button_width = 120
        button_height = 30
        button_color = (0, 150, 0)  # Vert

        # Variable pour indiquer si la sauvegarde est terminée
        saved = False

        # Fonction pour dessiner le bouton
        def draw_save_button(img):
            cv2.rectangle(img, (button_x, button_y), (button_x + button_width, button_y + button_height), button_color, -1)
            cv2.putText(img, "Sauvegarder", (button_x + 10, button_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Fonction de callback pour gérer les clics sur le bouton
        def mouse_callback(event, x, y, flags, param):
            nonlocal saved
            if event == cv2.EVENT_LBUTTONDOWN:
                if button_x <= x <= button_x + button_width and button_y <= y <= button_y + button_height:
                    # Sauvegarder les valeurs des sliders
                    j_lower = np.array([cv2.getTrackbarPos('Jaune H min', 'Calibration HSV'),
                                        cv2.getTrackbarPos('Jaune S min', 'Calibration HSV'),
                                        cv2.getTrackbarPos('Jaune V min', 'Calibration HSV')])
                    j_upper = self.jaune_upper
                    b_lower = np.array([cv2.getTrackbarPos('Bleu H min', 'Calibration HSV'),
                                        cv2.getTrackbarPos('Bleu S min', 'Calibration HSV'),
                                        cv2.getTrackbarPos('Bleu V min', 'Calibration HSV')])
                    b_upper = self.bleu_upper

                    self.jaune_lower, self.jaune_upper = j_lower, j_upper
                    self.bleu_lower,  self.bleu_upper  = b_lower, b_upper
                    
                    # Sauvegarder dans le fichier JSON
                    self.save_calibration()
                    
                    print(f"Valeurs sauvegardées — Jaune: {j_lower}→{j_upper} | Bleu: {b_lower}→{b_upper}")
                    saved = True  # Indiquer que la sauvegarde est terminée
                    self.stop_requested = True

        # Associer la fonction de callback à la fenêtre
        cv2.setMouseCallback('Calibration HSV', mouse_callback)

        # Créer les sliders
        for label, val, maxval in [
            ('Jaune H min', self.jaune_lower[0], 180),
            ('Jaune S min', self.jaune_lower[1], 255), 
            ('Jaune V min', self.jaune_lower[2], 255), 
            ('Bleu H min',  self.bleu_lower[0],  180), 
            ('Bleu S min',  self.bleu_lower[1],  255), 
            ('Bleu V min',  self.bleu_lower[2],  255), 
        ]:
            cv2.createTrackbar(label, 'Calibration HSV', val, maxval, lambda x: None)

        print("\n=== MODE CALIBRATION COULEURS ===")
        print("Ajustez les trackbars | Cliquez sur 'Sauvegarder' ou 'Q' pour quitter")

        while not saved:  # Boucle jusqu'à ce que la sauvegarde soit terminée
            # Redimensionner l'image pour l'affichage
            image_display = cv2.resize(image, (display_width, display_height))
            hsv = cv2.cvtColor(image_display, cv2.COLOR_BGR2HSV)

            # Récupérer les valeurs des sliders
            j_lower = np.array([cv2.getTrackbarPos('Jaune H min', 'Calibration HSV'),
                                cv2.getTrackbarPos('Jaune S min', 'Calibration HSV'),
                                cv2.getTrackbarPos('Jaune V min', 'Calibration HSV')])
            j_upper = self.jaune_upper
            b_lower = np.array([cv2.getTrackbarPos('Bleu H min', 'Calibration HSV'),
                                cv2.getTrackbarPos('Bleu S min', 'Calibration HSV'),
                                cv2.getTrackbarPos('Bleu V min', 'Calibration HSV')])
            b_upper = self.bleu_upper
            # Créer une image noire pour afficher les masques et le bouton
            combined = np.zeros_like(image_display)
            combined[:, :, 2] = cv2.inRange(hsv, j_lower, j_upper)  # Jaune en rouge (canal R)
            combined[:, :, 0] = cv2.inRange(hsv, b_lower, b_upper)   # Bleu en bleu (canal B)

            # Dessiner le bouton "Sauvegarder"
            draw_save_button(combined)

            cv2.imshow('Image Originale', image_display)
            cv2.imshow('Calibration HSV', combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        # Fermer les fenêtres après la sauvegarde ou la sortie
        cv2.destroyWindow('Calibration HSV')
        cv2.destroyWindow('Image Originale')


# ─────────────────────────────────────────────────────────────────────────────
# POINT D'ENTRÉE
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":

    couleur = "B"  # ← Changer ici : "B" pour Bleu, "J" pour Jaune

    config = Config()
    config = appliquer_couleur(config, couleur)

    print(f"Équipe configurée : {'Bleue' if couleur == 'B' else 'Jaune'}")
    print(f"  tag_calibration_Noisette : {config.tag_calibration_Noisette}")
    print(f"  tag_calibration_robot    : {config.tag_calibration_robot}")
    print(f"  tag_robot                : {config.tag_robot}")
    print(f"  tag_ennemi               : {config.tag_ennemi}")

    system = ArUcoTrackingSystem(config, matrice_antidstorsion='calibration_data_HR_camM.npz')

    # Chargement automatique des deux calibrations au démarrage
    if system.calibration_mode.load_calibration():
        system.homographie.calcul_homographie_elevated(system.calibration_mode.calibration_points)
        system.plan_elevated_calcule = True

    if system.calibration_mode_robot.load_calibration():
        system.homographie.calcul_homographie_robot(system.calibration_mode_robot.calibration_points)
        system.plan_robot_calcule = True

    cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("Erreur: impossible d'ouvrir la caméra")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  config.camera_largeur)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera_longueur)
    print(f"Résolution effective: {cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}")

    # Créer la fenêtre et le gestionnaire de boutons
    window_name = "Systeme de Tracking ArUco"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    
    button_manager = ButtonManager(window_name)
    
    # Ajouter les boutons (position en bas de l'écran redimensionné)
    btn_y = 680  # Position Y des boutons
    btn_h = 35   # Hauteur des boutons
    btn_spacing = 5
    
    buttons_config = [
        (10, "Plan Noisette", (0, 100, 150)),
        (150, "Plan Robot", (0, 150, 100)),
        (280, "Capturer", (150, 100, 0)),
        (400, "Sauvegarder", (0, 150, 150)),
        (550, "Chargement", (100, 0, 150)),
        (690, "Couleurs", (150, 150, 0)),
        (820, "Debug", (100, 100, 0)),
        (930, "Quitter", (150, 0, 0)),
    ]
    
    for btn_x, btn_text, btn_color in buttons_config:
        button_manager.add_button(Button_A(btn_x, btn_y, 130, btn_h, btn_text, btn_color))

    try:
        while True:

            ret, frame = cap.read()
            if not ret:
                print("Erreur de lecture de la caméra")
                break

            annotated, results = system.process_frame(frame)
            Liste_noisette_xya = results['Liste_noisette_xya']
            Liste_robots_xy    = results['Liste_robots_xy']
            print(Liste_noisette_xya)
            print(Liste_robots_xy)

            if hasattr(system, 'show_debug') and system.show_debug:
                system.color_detector.show_debug_masks(frame)

            if results['homographie_ok'] and not system.plan_reference_calcule:
                print("Plan de référence calculé et verrouillé.")
                system.plan_reference_calcule = True

            # Dessiner les boutons sur l'image
            button_manager.draw_all(annotated)
            
            cv2.imshow(window_name, annotated)
            
            # Gérer les clics de boutons
            button_click = button_manager.get_last_click()
            
            if button_click == "Plan Noisette":
                system.mode_calibration_active = True
                system.calibration_mode.reset()
                print("\nMode calibration NOISETTE activé")
                print(f"Positionner le tag {config.tag_calibration_Noisette} au-dessus du tag "
                      f"{system.calibration_mode.get_current_target()} et appuyer sur CAPTURER")
            
            elif button_click == "Plan Robot":
                system.mode_calibration_robot_active = True
                system.calibration_mode_robot.reset()
                print("\nMode calibration ROBOT activé")
                print(f"Positionner le tag {config.tag_calibration_robot} au-dessus du tag "
                      f"{system.calibration_mode_robot.get_current_target()} et appuyer sur CAPTURER")
            
            elif button_click == "Capturer":
                detected_tags_list, _ = system.detecteur.detect(system.undistort_image(frame))
                if system.mode_calibration_active:
                    system.handle_calibration_capture(detected_tags_list)
                elif system.mode_calibration_robot_active:
                    system.handle_calibration_capture_robot(detected_tags_list)
                else:
                    print("Aucune calibration active. Cliquez d'abord sur 'Plan Noisette' ou 'Plan Robot'")
            
            elif button_click == "Sauvegarder":
                if system.mode_calibration_active and system.calibration_mode.is_complete():
                    system.calibration_mode.save_calibration()
                    system.mode_calibration_active = False
                    system.plan_elevated_calcule = True
                    print("Calibration NOISETTE sauvegardée.")
                elif system.mode_calibration_robot_active and system.calibration_mode_robot.is_complete():
                    system.calibration_mode_robot.save_calibration()
                    system.mode_calibration_robot_active = False
                    system.plan_robot_calcule = True
                    print("Calibration ROBOT sauvegardée.")
                else:
                    print("Aucune calibration complète à sauvegarder")
            
            elif button_click == "Chargement":
                if not system.mode_calibration_active and not system.mode_calibration_robot_active:
                    if system.calibration_mode.load_calibration():
                        system.homographie.calcul_homographie_elevated(system.calibration_mode.calibration_points)
                        system.plan_elevated_calcule = True
                    if system.calibration_mode_robot.load_calibration():
                        system.homographie.calcul_homographie_robot(system.calibration_mode_robot.calibration_points)
                        system.plan_robot_calcule = True
                else:
                    print("Impossible de charger pendant une calibration active")
            
            elif button_click == "Couleurs":
                print("\n=== Entrée en mode calibration couleurs ===")
                system.color_detector.calibrate_interactive(frame)
            
            elif button_click == "Debug":
                if not hasattr(system, 'show_debug'):
                    system.show_debug = False
                system.show_debug = not system.show_debug
                if system.show_debug:
                    print("\nMode debug ACTIVÉ")
                else:
                    print("\nMode debug DÉSACTIVÉ")
                    for win in ["Masque Jaune", "Masque Bleu", "Masques Combinés (Bleu=Bleu, Jaune=Rouge)"]:
                        cv2.destroyWindow(win)
            
            elif button_click == "Quitter":
                break
            
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()