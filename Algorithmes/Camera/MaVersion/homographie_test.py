import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, List
from collections import deque
import json

@dataclass
class Config:
    tags_largeur_mm: float = 910.0
    tags_longueur_mm: float = 450.0
    ajout_horizontal_mm: float = 303.0
    ajout_vertical_mm: float = 225.0
    tag_taille_mm: float = 100.0
    tag_taille_mm_Noisette: float = 40.0
    hauteur_Noisette_mm: float = 30.0  # ou 330.0 pour le test avec 330mm
    hauteur_camera_mm: float = 1480.0
    camera_largeur: int = 1280
    camera_longueur: int = 720
    tag_reference: Tuple[int, ...] = (20, 21, 22, 23)
    tag_calibration: int = 36
    tag_mobile: Tuple[int, ...] = (36, 47)

    @property
    def largeur_totale_mm(self) -> float:
        return self.tags_largeur_mm + 2 * self.ajout_horizontal_mm

    @property
    def longueur_totale_mm(self) -> float:
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
        cfg = self.config
        positions = {
            22: np.array([cfg.ajout_horizontal_mm, cfg.ajout_vertical_mm]),
            23: np.array([cfg.ajout_horizontal_mm + cfg.tags_largeur_mm, cfg.ajout_vertical_mm]),
            20: np.array([cfg.ajout_horizontal_mm, cfg.ajout_vertical_mm + cfg.tags_longueur_mm]),
            21: np.array([cfg.ajout_horizontal_mm + cfg.tags_largeur_mm, cfg.ajout_vertical_mm + cfg.tags_longueur_mm]),
        }
        return positions

    def tag_vers_coin(self, tag_id: int) -> int:
        num_coin = {
            22: 3,
            23: 2,
            21: 1,
            20: 0,
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

        position_ref = self.position_tag_reference()

        src_points = []
        dst_points = []

        for tag_id in [20, 21, 22, 23]:
            if tag_id in calibration_points and tag_id in position_ref:
                centre_tag = np.mean(calibration_points[tag_id], axis=0)
                src_points.append(centre_tag)
                dst_points.append(position_ref[tag_id])

        if len(src_points) == 4:
            src = np.array(src_points, dtype=np.float32)
            dst = np.array(dst_points, dtype=np.float32)

            self.H_cam_to_elevated, _ = cv2.findHomography(src, dst)

            if self.H_cam_to_elevated is not None:
                self.H_elevated_to_cam = np.linalg.inv(self.H_cam_to_elevated)

                if self.H_cam_to_ref is not None:
                    self.H_elevated_to_ref = self.H_cam_to_ref @ self.H_elevated_to_cam
                    self.H_ref_to_elevated = np.linalg.inv(self.H_elevated_to_ref)

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
    def __init__(self, dictionary_type=cv2.aruco.DICT_4X4_50):
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dictionary_type)
        self.detecteur = cv2.aruco.ArucoDetector(self.aruco_dict)

    def detect(self, image: np.ndarray) -> Tuple[Dict[int, np.ndarray], np.ndarray]:
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        coins, id, rejected = self.detecteur.detectMarkers(gray)

        detected_tags = {}
        if id is not None:
            for i, tag_id in enumerate(id.flatten()):
                detected_tags[int(tag_id)] = coins[i][0]

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
        self.homographie = CalculHomographie(config)
        self.calibration_mode = CalibrationMode(config)
        self.mode_calibration_active = False

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

    def apply_height_offset(self, pixel_point: np.ndarray, center: np.ndarray) -> np.ndarray:
        """
        Applique un décalage en fonction de la hauteur du plan élevé
        """
        # Calculer le facteur de mise à l'échelle en fonction de la hauteur
        scale_factor = self.config.hauteur_camera_mm / (self.config.hauteur_camera_mm - self.config.hauteur_Noisette_mm)
        # Calculer le déplacement par rapport au centre
        offset = pixel_point - center
        # Appliquer le facteur de mise à l'échelle
        offset_scaled = offset * scale_factor
        # Retourner le point décalé
        return center + offset_scaled

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, Dict]:
        results = {
            'homographie_ok': False,
            'homographie_elevated_ok': False,
            'tag_reference': {},
            'tag_mobile': {},
            'calibration_mode': self.mode_calibration_active,
            'calibration_complete': self.calibration_mode.is_complete()
        }

        undistorted = self.undistort_image(frame)
        undistorted_copie = undistorted.copy()

        detected_tags, gray = self.detecteur.detect(undistorted)

        for tag_id, coins in detected_tags.items():
            coins_int = coins.astype(np.int32)
            if tag_id in self.config.tag_reference:
                cv2.polylines(undistorted_copie, [coins_int], True, (255, 0, 0), 1)
            elif tag_id in self.config.tag_mobile:
                if self.mode_calibration_active and tag_id == self.config.tag_calibration:
                    cv2.polylines(undistorted_copie, [coins_int], True, (0, 255, 255), 3)
                else:
                    cv2.polylines(undistorted_copie, [coins_int], True, (255, 0, 255), 2)

        tag_reference = {tid: coins for tid, coins in detected_tags.items()
                        if tid in self.config.tag_reference}

        tag_mobile = {tid: coins for tid, coins in detected_tags.items()
                     if tid in self.config.tag_mobile}

        if len(tag_reference) >= 3:
            homographie_ok = self.homographie.calcul_homographie(tag_reference)
            results['homographie_ok'] = homographie_ok

            if homographie_ok:
                ref_positions = self.homographie.position_tag_reference()
                for tid in tag_reference:
                    if tid in ref_positions:
                        results['tag_reference'][tid] = ref_positions[tid].tolist()

                # Dessiner le rectangle du plan de référence étendu
                haut_droit_mm = np.array([0, 0])
                haut_gauche_mm = np.array([self.config.largeur_totale_mm, 0])
                bas_droit_mm = np.array([0, self.config.longueur_totale_mm])
                bas_gauche_mm = np.array([self.config.largeur_totale_mm, self.config.longueur_totale_mm])

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

        if results['homographie_ok'] and self.calibration_mode.is_complete():
            homographie_elevated_ok = self.homographie.calcul_homographie_elevated(self.calibration_mode.calibration_points)
            results['homographie_elevated_ok'] = homographie_elevated_ok

            if homographie_elevated_ok:
                # Dessiner le rectangle du plan élevé
                # Coins du plan élevé en millimètres (identiques au plan de référence)
                haut_droit_mm = np.array([0, 0])
                haut_gauche_mm = np.array([self.config.largeur_totale_mm, 0])
                bas_droit_mm = np.array([0, self.config.longueur_totale_mm])
                bas_gauche_mm = np.array([self.config.largeur_totale_mm, self.config.longueur_totale_mm])

                # Convertir les coins du plan de référence en pixels
                haut_droit_pixel = self.homographie.point_ref_to_cam(haut_droit_mm)
                haut_gauche_pixel = self.homographie.point_ref_to_cam(haut_gauche_mm)
                bas_droit_pixel = self.homographie.point_ref_to_cam(bas_droit_mm)
                bas_gauche_pixel = self.homographie.point_ref_to_cam(bas_gauche_mm)

                # Calculer le centre du rectangle de référence
                center_pixel = np.mean([haut_droit_pixel, haut_gauche_pixel, bas_droit_pixel, bas_gauche_pixel], axis=0)

                # Appliquer un décalage pour simuler la hauteur
                if haut_droit_pixel is not None:
                    haut_droit_pixel_elev = self.apply_height_offset(haut_droit_pixel, center_pixel)
                if haut_gauche_pixel is not None:
                    haut_gauche_pixel_elev = self.apply_height_offset(haut_gauche_pixel, center_pixel)
                if bas_droit_pixel is not None:
                    bas_droit_pixel_elev = self.apply_height_offset(bas_droit_pixel, center_pixel)
                if bas_gauche_pixel is not None:
                    bas_gauche_pixel_elev = self.apply_height_offset(bas_gauche_pixel, center_pixel)

                if haut_droit_pixel_elev is not None and haut_gauche_pixel_elev is not None and bas_droit_pixel_elev is not None and bas_gauche_pixel_elev is not None:
                    elevated_rectangle = np.array([
                        haut_droit_pixel_elev.astype(int),
                        haut_gauche_pixel_elev.astype(int),
                        bas_gauche_pixel_elev.astype(int),
                        bas_droit_pixel_elev.astype(int)
                    ], dtype=np.int32)

                    # Dessiner le rectangle du plan élevé en violet
                    cv2.polylines(undistorted_copie, [elevated_rectangle], True, (255, 0, 255), 3)

                if tag_mobile:
                    for tag_id, coins in tag_mobile.items():
                        # Dessiner le carré autour du tag mobile
                        cv2.polylines(undistorted_copie, [coins.astype(np.int32)], True, (255, 0, 255), 2)

                        # Projeter le tag sur le plan de référence
                        centre_pixel = np.mean(coins, axis=0)
                        pos_elevated = self.homographie.point_cam_to_elevated(centre_pixel)

                        if pos_elevated is not None:
                            demi_taille = self.config.tag_taille_mm_Noisette / 2

                            coin_hg_elev = pos_elevated + np.array([-demi_taille, -demi_taille])
                            coin_hd_elev = pos_elevated + np.array([demi_taille, -demi_taille])
                            coin_bd_elev = pos_elevated + np.array([demi_taille, demi_taille])
                            coin_bg_elev = pos_elevated + np.array([-demi_taille, demi_taille])

                            coins_elev_mm = [coin_hg_elev, coin_hd_elev, coin_bd_elev, coin_bg_elev]

                            coins_projetes_ref_mm = []
                            for coin_elev_mm in coins_elev_mm:
                                coin_ref_mm = self.homographie.point_elevated_to_ref(coin_elev_mm)
                                if coin_ref_mm is not None:
                                    coins_projetes_ref_mm.append(coin_ref_mm)

                            if len(coins_projetes_ref_mm) == 4:
                                centre_projete_ref_mm = np.mean(coins_projetes_ref_mm, axis=0)

                                # Convertir les coins projetés en pixels pour les dessiner
                                projected_corners_cam = []
                                for corner_ref in coins_projetes_ref_mm:
                                    corner_cam = self.homographie.point_ref_to_cam(corner_ref)
                                    if corner_cam is not None:
                                        projected_corners_cam.append(corner_cam)

                                if len(projected_corners_cam) == 4:
                                    projected_corners_cam = np.array(projected_corners_cam, dtype=np.int32)
                                    # Dessiner le rectangle projeté orthogonalement
                                    cv2.polylines(undistorted_copie, [projected_corners_cam], True, (0, 255, 255), 2)

                                    # Afficher les coordonnées
                                    coord_text = f"({centre_projete_ref_mm[0]:.0f}, {centre_projete_ref_mm[1]:.0f})"
                                    center_cam = self.homographie.point_ref_to_cam(centre_projete_ref_mm)
                                    if center_cam is not None:
                                        cv2.putText(undistorted_copie, coord_text,
                                                tuple(center_cam.astype(int) + np.array([15, 30])),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                                results['tag_mobile'][tag_id] = {
                                    'elevated_mm': pos_elevated.tolist(),
                                    'ref_mm': centre_projete_ref_mm.tolist(),
                                    'pixel_center': centre_pixel.tolist()
                                }

        if self.mode_calibration_active:
            target = self.calibration_mode.get_current_target()
            if target is not None:
                instruction_text = f"MODE CALIBRATION - Positionner Tag {self.config.tag_calibration} sur Tag {target} puis appuyer sur ESPACE"
                cv2.putText(undistorted_copie, instruction_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                cv2.putText(undistorted_copie, f"Progression: {self.calibration_mode.current_index}/4", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            else:
                cv2.putText(undistorted_copie, "CALIBRATION TERMINEE - Appuyer sur 'S' pour sauvegarder", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            status_text = f"Tags ref: {len(tag_reference)}/4 | Tags mobiles: {len(tag_mobile)}"
            color = (0, 255, 0) if results['homographie_ok'] else (0, 0, 255)
            cv2.putText(undistorted_copie, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

            if results['homographie_ok']:
                cv2.putText(undistorted_copie, "Homographie Plan Ref OK", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(undistorted_copie, "Homographie INVALIDE - besoin de 3+ tags", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            if self.calibration_mode.is_complete() and results['homographie_elevated_ok']:
                cv2.putText(undistorted_copie, "Homographie Plan Eleve OK", (10, 90),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if tag_mobile:
                    y_pos = 120
                    for tag_id in tag_mobile:
                        if tag_id in results['tag_mobile']:
                            data = results['tag_mobile'][tag_id]
                            pos_elevated = data['elevated_mm']
                            pos_ref = data['ref_mm']

                            cv2.putText(undistorted_copie, f"Tag {tag_id} Plan Eleve: ({pos_elevated[0]:.0f}, {pos_elevated[1]:.0f})", (10, y_pos),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)
                            y_pos += 25
                            cv2.putText(undistorted_copie, f"Tag {tag_id} Plan Ref: ({pos_ref[0]:.0f}, {pos_ref[1]:.0f})", (10, y_pos),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            y_pos += 25

            cv2.putText(undistorted_copie, "Appuyer sur 'C' pour calibrer plan eleve", (10, undistorted_copie.shape[0] - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(undistorted_copie, "Appuyer sur 'L' pour charger calibration", (10, undistorted_copie.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        return undistorted_copie, results

    def handle_calibration_capture(self, detected_tags: Dict[int, np.ndarray]) -> bool:
        if self.config.tag_calibration in detected_tags:
            tag_corners = detected_tags[self.config.tag_calibration]
            success = self.calibration_mode.capture_position(tag_corners)
            if success:
                target = self.calibration_mode.get_current_target()
                if target is None:
                    print("Calibration complete! Appuyer sur 'S' pour sauvegarder ou 'C' pour recommencer")
                else:
                    print(f"Position capturée! Placer le tag sur la position {target} et appuyer sur ESPACE")
                return True
        else:
            print(f"Tag {self.config.tag_calibration} non détecté")
        return False

def main():
    config = Config()
    config.hauteur_Noisette_mm = 330.0  # Changer cette valeur pour 330mm

    print("=" * 80)
    print("Système de Tracking ArUco avec Double Plan")
    print("=" * 80)
    print(f"\nConfiguration:")
    print(f"  - Plan de référence: {config.largeur_totale_mm:.0f} x {config.longueur_totale_mm:.0f} mm")
    print(f"  - Rectangle tags: {config.tags_largeur_mm:.0f} x {config.tags_longueur_mm:.0f} mm")
    print(f"  - Marges: {config.ajout_horizontal_mm:.0f} x {config.ajout_vertical_mm:.0f} mm")
    print(f"  - Hauteur plan élevé: +{config.hauteur_Noisette_mm:.0f} mm")
    print(f"\nTags de référence (plan h=0): {config.tag_reference}")
    print(f"Tag de calibration (plan h=+{config.hauteur_Noisette_mm:.0f}mm): {config.tag_calibration}")
    print(f"Tags mobiles (plan h=+{config.hauteur_Noisette_mm:.0f}mm): {config.tag_mobile}")
    print(f"\nOrigine du repère: coin SUPÉRIEUR DROIT du plan de référence")
    print(f"  - X positif vers la GAUCHE")
    print(f"  - Y positif vers le BAS")
    print("\n" + "=" * 80)
    print("INSTRUCTIONS:")
    print("  - Appuyer sur 'C' pour entrer en mode calibration du plan élevé")
    print("  - En mode calibration:")
    print("    1. Positionner le tag 36 sur chaque position des tags de référence (20, 21, 22, 23)")
    print("    2. Appuyer sur ESPACE à chaque position pour capturer")
    print("    3. Appuyer sur 'S' pour sauvegarder la calibration")
    print("    4. Appuyer sur 'R' pour recommencer la calibration")
    print("  - Appuyer sur 'L' pour charger une calibration existante")
    print("  - Appuyer sur 'Q' pour quitter")
    print("=" * 80)

    system = ArUcoTrackingSystem(config, matrice_antidstorsion='calibration_data.npz')

    if system.calibration_mode.load_calibration():
        system.homographie.calcul_homographie_elevated(system.calibration_mode.calibration_points)

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

    if not cap.isOpened():
        print("Erreur: impossible d'ouvrir la caméra")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.camera_largeur)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.camera_longueur)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Erreur de lecture de la caméra")
                break

            annotated, results = system.process_frame(frame)

            cv2.imshow("Systeme de Tracking ArUco", annotated)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('c'):
                system.mode_calibration_active = True
                system.calibration_mode.reset()
                print("\nMode calibration activé")
                print(f"Positionner le tag {config.tag_calibration} sur la position du tag {system.calibration_mode.get_current_target()}")
            elif key == ord(' ') and system.mode_calibration_active:
                detected_tags, _ = system.detecteur.detect(system.undistort_image(frame))
                system.handle_calibration_capture(detected_tags)
            elif key == ord('s') and system.mode_calibration_active and system.calibration_mode.is_complete():
                system.calibration_mode.save_calibration()
                system.mode_calibration_active = False
                system.homographie.calcul_homographie_elevated(system.calibration_mode.calibration_points)
                print("Mode calibration désactivé")
            elif key == ord('r') and system.mode_calibration_active:
                system.calibration_mode.reset()
                print("\nCalibration réinitialisée")
                print(f"Positionner le tag {config.tag_calibration} sur la position du tag {system.calibration_mode.get_current_target()}")
            elif key == ord('l') and not system.mode_calibration_active:
                if system.calibration_mode.load_calibration():
                    system.homographie.calcul_homographie_elevated(system.calibration_mode.calibration_points)

    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
