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
        font_scale = 0.65
        thickness = 2
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
            20: np.array([demi_taille, 1500-demi_taille]),
            21: np.array([cfg.largeur_totale_mm - demi_taille, 1500-demi_taille]),
        }
        return positions

    def position_tag_robot(self) -> Dict[int, np.ndarray]:
        cfg = self.config
        demi_taille = cfg.tag_taille_mm / 2
        positions = {
            22: np.array([demi_taille, demi_taille]),
            23: np.array([cfg.largeur_totale_mm - demi_taille, demi_taille]),
            20: np.array([demi_taille, 1500-demi_taille]),
            21: np.array([cfg.largeur_totale_mm - demi_taille, 1500.0-demi_taille]),
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
        if len(calibration_points) != 4:
            return False

        position_robot = self.position_tag_robot()
        src_points, dst_points = [], []

        for tag_id in [20, 21, 22, 23]:
            if tag_id in calibration_points and tag_id in position_robot:
                centre_tag = np.mean(calibration_points[tag_id], axis=0)
                src_points.append(centre_tag)
                dst_points.append(position_robot[tag_id])

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
    def __init__(self, config: Config, sequence: List[int] = None):
        self.config = config
        self.calibration_points = {}
        self.sequence = sequence if sequence is not None else [20, 21, 22, 23]
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
    def __init__(self, config: Config, sequence: List[int] = None):
        self.config = config
        self.calibration_points = {}
        self.sequence = sequence if sequence is not None else [20, 21, 22, 23]
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
    def __init__(self, config: Config, matrice_antidstorsion: Optional[str] = None, 
             calibration_sequence: List[int] = None):
        self.config = config
        self.detecteur = ArUcoDetector()
        self.color_detector = ColorDetector(config)
        self.homographie = CalculHomographie(config)

        # Plan Noisette
        self.calibration_mode = CalibrationMode(config, sequence=calibration_sequence)
        self.mode_calibration_active = False
        self.plan_elevated_calcule = False

        # Plan Robot
        self.calibration_mode_robot = CalibrationModeRobot(config, sequence=calibration_sequence)
        self.mode_calibration_robot_active = False
        self.plan_robot_calcule = False

        self.plan_reference_calcule = False

        # Mémoire des objets colorés détectés (filtre passe-bas)
        self.color_memory_jaune = {}
        self.color_memory_bleu = {}
        self.color_object_id_counter = 0
        self.max_frames_missing = 2
        self.distance_threshold_mm = 50.0
        self.frame_counter = 0

        if matrice_antidstorsion:
            self.load_calibration(matrice_antidstorsion)

        self.centre_image = np.array([config.camera_largeur / 2, config.camera_longueur / 2])

    def _draw_text_with_background(self, image, text, position, font=cv2.FONT_HERSHEY_SIMPLEX, 
                                font_scale=0.6, text_color=(0, 255, 255), thickness=2,
                                bg_color=(255, 255, 255), padding=5):
        x, y = position
        (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)
        rect_x1 = x - padding
        rect_y1 = y - text_height - padding
        rect_x2 = x + text_width + padding
        rect_y2 = y + baseline + padding
        cv2.rectangle(image, (rect_x1, rect_y1), (rect_x2, rect_y2), bg_color, -1)
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
                    (10, 30), text_color=(0, 0, 0))
                self._draw_text_with_background(
                    undistorted_copie,
                    f"Progression: {self.calibration_mode.current_index}/4",
                    (10, 60), text_color=(0, 0, 0))
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
                    (10, 90), text_color=(0, 0, 0))
                self._draw_text_with_background(
                    undistorted_copie,
                    f"Progression robot: {self.calibration_mode_robot.current_index}/4",
                    (10, 120), text_color=(0, 0, 0))
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

        # ── Détection des couleurs (plan de référence) ────────────────────────
        if self.plan_reference_calcule:
            self.frame_counter += 1

            color_results = self.color_detector.detect_colors(undistorted_pour_detection)

            cfg = self.config
            # Offset pour que (-550, -550) devienne (0, 0)
            offset = np.array([cfg.ajout_horizontal_mm, cfg.ajout_vertical_mm])

            # Zone de filtrage dans le plan décalé : (0,0) à (largeur_totale, longueur_totale)
            coins_ref_mm = [
                np.array([0, 0]),
                np.array([cfg.largeur_totale_mm, 0]),
                np.array([cfg.largeur_totale_mm, cfg.longueur_totale_mm]),
                np.array([0, cfg.longueur_totale_mm]),
            ]

            objets_jaunes_valides, objets_bleus_valides = [], []

            for obj in color_results['jaune']:
                pos_ref = self.homographie.point_cam_to_ref(obj['centre_pixel'])
                if pos_ref is not None:
                    pos_ref = pos_ref + offset
                    if self.point_dans_polygone(pos_ref, coins_ref_mm):
                        obj['pos_ref'] = pos_ref
                        objets_jaunes_valides.append(obj)

            for obj in color_results['bleu']:
                pos_ref = self.homographie.point_cam_to_ref(obj['centre_pixel'])
                if pos_ref is not None:
                    pos_ref = pos_ref + offset
                    if self.point_dans_polygone(pos_ref, coins_ref_mm):
                        obj['pos_ref'] = pos_ref
                        objets_bleus_valides.append(obj)

            # Appliquer le filtre passe-bas (mémoire)
            objets_jaunes_avec_memoire = self._update_color_memory(objets_jaunes_valides, self.color_memory_jaune, 'jaune')
            objets_bleus_avec_memoire = self._update_color_memory(objets_bleus_valides, self.color_memory_bleu, 'bleu')

            # Affichage des objets individuels avec coordonnées
            for obj in objets_jaunes_avec_memoire:
                if obj.get('pos_ref') is not None:
                    self._process_colored_object(obj, undistorted_copie, results, (0, 255, 255))
            for obj in objets_bleus_avec_memoire:
                if obj.get('pos_ref') is not None:
                    self._process_colored_object(obj, undistorted_copie, results, (140, 91, 0))

        # ── Dessin du rectangle étendu du plan Robot (orange) ────────────────
        if self.plan_robot_calcule:
            cfg = self.config
            y_max_zone_robot = 1450.0
            coins_robot = [
                np.array([0, 0]),
                np.array([cfg.largeur_totale_mm, 0]),
                np.array([cfg.largeur_totale_mm, y_max_zone_robot]),
                np.array([0, y_max_zone_robot]),
            ]
            pixels_robot = [self.homographie.point_robot_to_cam(p) for p in coins_robot]
            if all(p is not None for p in pixels_robot):
                rect_robot = np.array([p.astype(int) for p in pixels_robot], dtype=np.int32)
                cv2.polylines(undistorted_copie, [rect_robot], True, (0, 165, 255), 3)

        # ── Détection des tags robot et ennemi (plan Robot) ───────────────────
        if self.plan_robot_calcule:
            for tag_id, coins in detected_tags_list:
                est_robot  = tag_id in self.config.tag_robot
                est_ennemi = tag_id in self.config.tag_ennemi
                if not (est_robot or est_ennemi):
                    continue

                centre_pixel = np.mean(coins, axis=0)
                pos_robot = self.homographie.point_cam_to_robot(centre_pixel)

                if pos_robot is None:
                    continue

                coins_robot_plan = []
                for coin_pixel in coins:
                    coin_mm = self.homographie.point_cam_to_robot(coin_pixel)
                    if coin_mm is not None:
                        coins_robot_plan.append(coin_mm)

                angle_deg = 0.0
                angle_rad = 0.0
                if len(coins_robot_plan) == 4:
                    vec_x = coins_robot_plan[1][0] - coins_robot_plan[0][0]
                    vec_y = coins_robot_plan[1][1] - coins_robot_plan[0][1]
                    angle_rad = np.arctan2(vec_y, vec_x)
                    angle_deg = np.degrees(angle_rad)

                color_display = (0, 255, 0) if est_robot else (0, 0, 255)

                cv2.polylines(undistorted_copie, [coins.astype(np.int32)], True, color_display, 2)
                cv2.circle(undistorted_copie, tuple(centre_pixel.astype(int)), 6, color_display, -1)

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
        """Dessine l'objet coloré et affiche ses coordonnées dans le plan de référence."""
        cv2.drawContours(image, [obj['contour']], -1, color_bgr, 3)
        centre_pixel = obj['centre_pixel']
        cv2.circle(image, tuple(centre_pixel.astype(int)), 8, color_bgr, -1)
        cv2.circle(image, tuple(centre_pixel.astype(int)), 10, (255, 255, 255), 2)

        pos_ref = obj.get('pos_ref')
        if pos_ref is not None:
            # Croix blanche au centre
            cross_size = 15
            cx, cy = int(centre_pixel[0]), int(centre_pixel[1])
            cv2.line(image, (cx - cross_size, cy), (cx + cross_size, cy), (255, 255, 255), 2)
            cv2.line(image, (cx, cy - cross_size), (cx, cy + cross_size), (255, 255, 255), 2)

            # Afficher les coordonnées en mm à côté de la zone
            coord_text = f"({int(pos_ref[0])}, {int(pos_ref[1])})"
            self._draw_text_with_background(
                image, coord_text,
                (cx + 15, cy - 10),
                font_scale=0.45, thickness=1,
                text_color=color_bgr, bg_color=(255, 255, 255), padding=3)

            results['objets_colores'].append({
                'couleur': obj['couleur'],
                'position_ref_mm': pos_ref.tolist(),
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
        """
        current_frame = self.frame_counter
        matched_ids = set()
        result_objects = []

        valid_detected = [obj for obj in detected_objects if obj.get('pos_ref') is not None]

        # 1. Associer les objets détectés aux objets en mémoire
        for obj in valid_detected:
            pos_current = obj['pos_ref']
            best_match_id = None
            best_distance = self.distance_threshold_mm

            for obj_id, mem_data in color_memory.items():
                mem_obj = mem_data['obj']
                if mem_obj.get('pos_ref') is None:
                    continue
                pos_memory = mem_obj['pos_ref']
                distance = np.linalg.norm(pos_current - pos_memory)
                if distance < best_distance:
                    best_distance = distance
                    best_match_id = obj_id

            if best_match_id is not None:
                color_memory[best_match_id]['obj'] = obj
                color_memory[best_match_id]['frames_missing'] = 0
                color_memory[best_match_id]['last_seen'] = current_frame
                matched_ids.add(best_match_id)
                result_objects.append(obj)
            else:
                new_id = self.color_object_id_counter
                self.color_object_id_counter += 1
                color_memory[new_id] = {
                    'obj': obj,
                    'frames_missing': 0,
                    'last_seen': current_frame
                }
                matched_ids.add(new_id)
                result_objects.append(obj)

        # 2. Incrémenter le compteur pour les objets non détectés
        ids_to_remove = []
        for obj_id, mem_data in color_memory.items():
            if obj_id not in matched_ids:
                mem_data['frames_missing'] += 1
                if mem_data['frames_missing'] > self.max_frames_missing:
                    ids_to_remove.append(obj_id)
                else:
                    result_objects.append(mem_data['obj'])

        # 3. Supprimer les objets trop anciens
        for obj_id in ids_to_remove:
            del color_memory[obj_id]

        return result_objects


# ─────────────────────────────────────────────────────────────────────────────
# DÉTECTEUR DE COULEURS
# ─────────────────────────────────────────────────────────────────────────────

class ColorDetector:
    def __init__(self, config: Config):
        self.config = config
        self.calibration_file = "color_calibration.json"
        
        default_jaune_lower = [17, 105, 205]
        default_jaune_upper = [36, 255, 255]
        default_bleu_lower = [47, 105, 101]
        default_bleu_upper = [135, 255, 255]
        
        if not self.load_calibration():
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
        scale_factor = 0.5
        display_width = int(image.shape[1] * scale_factor)
        display_height = int(image.shape[0] * scale_factor)

        cv2.namedWindow('Calibration HSV')
        cv2.namedWindow('Image Originale')
        cv2.resizeWindow('Calibration HSV', display_width, display_height)
        cv2.resizeWindow('Image Originale', display_width, display_height)

        button_x, button_y, button_width, button_height = 10, 10, 120, 30
        button_color = (0, 150, 0)
        saved = False

        def draw_save_button(img):
            cv2.rectangle(img, (button_x, button_y), (button_x + button_width, button_y + button_height), button_color, -1)
            cv2.putText(img, "Sauvegarder", (button_x + 10, button_y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        def mouse_callback(event, x, y, flags, param):
            nonlocal saved
            if event == cv2.EVENT_LBUTTONDOWN:
                if button_x <= x <= button_x + button_width and button_y <= y <= button_y + button_height:
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
                    self.save_calibration()
                    print(f"Valeurs sauvegardées — Jaune: {j_lower}→{j_upper} | Bleu: {b_lower}→{b_upper}")
                    saved = True
                    self.stop_requested = True

        cv2.setMouseCallback('Calibration HSV', mouse_callback)

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

        while not saved:
            image_display = cv2.resize(image, (display_width, display_height))
            hsv = cv2.cvtColor(image_display, cv2.COLOR_BGR2HSV)

            j_lower = np.array([cv2.getTrackbarPos('Jaune H min', 'Calibration HSV'),
                                cv2.getTrackbarPos('Jaune S min', 'Calibration HSV'),
                                cv2.getTrackbarPos('Jaune V min', 'Calibration HSV')])
            j_upper = self.jaune_upper
            b_lower = np.array([cv2.getTrackbarPos('Bleu H min', 'Calibration HSV'),
                                cv2.getTrackbarPos('Bleu S min', 'Calibration HSV'),
                                cv2.getTrackbarPos('Bleu V min', 'Calibration HSV')])
            b_upper = self.bleu_upper

            combined = np.zeros_like(image_display)
            combined[:, :, 2] = cv2.inRange(hsv, j_lower, j_upper)
            combined[:, :, 0] = cv2.inRange(hsv, b_lower, b_upper)
            draw_save_button(combined)

            cv2.imshow('Image Originale', image_display)
            cv2.imshow('Calibration HSV', combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyWindow('Calibration HSV')
        cv2.destroyWindow('Image Originale')


# ─────────────────────────────────────────────────────────────────────────────
# POINT D'ENTRÉE
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":

    couleur = "B"  # ← Changer ici : "B" pour Bleu, "J" pour Jaune

    config = Config()
    config = appliquer_couleur(config, couleur)

    if couleur == "B":
        calibration_sequence = [20, 22, 23, 21]
    elif couleur == "J":
        calibration_sequence = [21, 23, 22, 20]
    else:
        calibration_sequence = [20, 21, 22, 23]

    print(f"Équipe configurée : {'Bleue' if couleur == 'B' else 'Jaune'}")
    print(f"  tag_calibration_Noisette : {config.tag_calibration_Noisette}")
    print(f"  tag_calibration_robot    : {config.tag_calibration_robot}")
    print(f"  tag_robot                : {config.tag_robot}")
    print(f"  tag_ennemi               : {config.tag_ennemi}")

    system = ArUcoTrackingSystem(config, 
                             matrice_antidstorsion='calibration_data_HR_vraiecam.npz',
                             calibration_sequence=calibration_sequence)
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

    window_name = "Systeme de Tracking ArUco"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1280, 720)
    
    button_manager = ButtonManager(window_name)
    
    btn_y = 680
    btn_h = 35
    
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

    zone_Noisette = [
        [(0,1000),(350,1400)],
        [(0,200),(350,600)],
        [(2650,1000),(3000,1400)],
        [(2650,200),(3000,600)],
        
        [(1350,625),(1500,975)],
        [(1650,625),(2050,975)],
        [(900,0),(1300,350)],
        [(1700,0),(2200,350)],
    ]
    
    try:
        while True:

            ret, frame = cap.read()
            if not ret:
                print("Erreur de lecture de la caméra")
                break

            annotated, results = system.process_frame(frame)
            Liste_robots_xy = results['Liste_robots_xy']
           
            objet_zone_noisette = [
                [],[],[],[],[],[],[],[]
            ]
            code_couleur_noisette = ["N","N","N","N","N","N","N","N"]

            for i in range(len(zone_Noisette)):
                for j in range(len(results['objets_colores'])):
                    if (zone_Noisette[i][0][0] <= results['objets_colores'][j]['position_ref_mm'][0] <= zone_Noisette[i][1][0]) and (zone_Noisette[i][0][1] <= results['objets_colores'][j]['position_ref_mm'][1] <= zone_Noisette[i][1][1]):
                        objet_zone_noisette[i].append(results['objets_colores'][j])

                print(f"Zone N°{i} : {objet_zone_noisette[i]}")

    
            for i in range(len(objet_zone_noisette)):
                nbr_jaune = 0
                nbr_bleu = 0
                for objet in objet_zone_noisette[i]:
                    if objet['couleur'] == 'jaune':
                        nbr_jaune += 1
                    elif objet['couleur'] == 'bleu':
                        nbr_bleu += 1
                if nbr_jaune == 2 and nbr_bleu == 4:
                    code_couleur_noisette[i] = "C"
                if nbr_jaune == 4 and nbr_bleu == 2:
                    code_couleur_noisette[i] = "D"
                if nbr_jaune == 2 and nbr_bleu == 2:
                    bleux = [objet for objet in objet_zone_noisette[i] if objet['couleur'] == 'bleu']
                    jaunes = [objet for objet in objet_zone_noisette[i] if objet['couleur'] == 'jaune']
                    if 0<=i<=3:
                        if (bleux[0]['position_ref_mm'][1] > jaunes[0]['position_ref_mm'][1]):
                            code_couleur_noisette[i] = "A"
                        else:
                            code_couleur_noisette[i] = "B"
                    else:
                        if (bleux[0]['position_ref_mm'][0] < jaunes[0]['position_ref_mm'][0]):
                            code_couleur_noisette[i] = "A"
                        else:
                            code_couleur_noisette[i] = "B"

                if nbr_jaune == 4 and nbr_bleu == 4:
                    bleux = [objet for objet in objet_zone_noisette[i] if objet['couleur'] == 'bleu']
                    jaunes = [objet for objet in objet_zone_noisette[i] if objet['couleur'] == 'jaune']
                    x_plus_petit = 3000
                    y_plus_petit = 2000
                    couleur_plus_petite = ""
                    if 0<=i<=3:
                        for objet in objet_zone_noisette[i]:
                            if objet['position_ref_mm'][1] < y_plus_petit:
                                y_plus_petit = objet['position_ref_mm'][1]
                                couleur_plus_petite = objet['couleur']
                        if couleur_plus_petite == "bleu":
                            code_couleur_noisette[i] = "F"
                        else:
                            code_couleur_noisette[i] = "E"
                    else:
                        for objet in objet_zone_noisette[i]:
                            if objet['position_ref_mm'][0] < x_plus_petit:
                                x_plus_petit = objet['position_ref_mm'][0]
                                couleur_plus_petite = objet['couleur']
                        if couleur_plus_petite == "bleu":
                            code_couleur_noisette[i] = "E"
                        else:
                            code_couleur_noisette[i] = "F"
                            
                print(f"Code de la zone N°{i} : {code_couleur_noisette[i]}")

            #print(Liste_robots_xy)

            if hasattr(system, 'show_debug') and system.show_debug:
                system.color_detector.show_debug_masks(frame)

            if results['homographie_ok'] and not system.plan_reference_calcule:
                print("Plan de référence calculé et verrouillé.")
                system.plan_reference_calcule = True

            button_manager.draw_all(annotated)
            cv2.imshow(window_name, annotated)
            
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