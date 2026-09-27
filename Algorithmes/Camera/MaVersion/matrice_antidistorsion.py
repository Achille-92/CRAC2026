import cv2
import numpy as np

# Charger les paramètres de calibration
data = np.load('calibration_data.npz')
mtx, dist = data['mtx'], data['dist']

# Paramètres de la caméra
CAMERA_INDEX = 0

# Initialisation de la caméra
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    print("Erreur : impossible d'ouvrir la caméra")
    exit()

# Configuration de la résolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Création du dictionnaire et du détecteur ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(aruco_dict)

# NOUVELLES DIMENSIONS du VRAI rectangle en mm
total_width_mm = 1430   # Largeur totale
total_height_mm = 900   # Hauteur totale

# Dimensions du rectangle formé par les tags en mm
tags_width_mm = 1232  # 123.2 cm
tags_height_mm = 544  # 54.4 cm

# Taille des tags ArUco en mm
tag_size_mm = 100  # 10 cm

# Calculer les marges réelles
margin_horizontal = (total_width_mm - tags_width_mm) / 2  # Marges gauche/droite
margin_vertical = (total_height_mm - tags_height_mm) / 2  # Marges haut/bas

print(f"Dimensions du rectangle des tags (coins extérieurs): {tags_width_mm}mm x {tags_height_mm}mm")
print(f"Marges calculées: {margin_horizontal}mm (gauche/droite), {margin_vertical}mm (haut/bas)")
print(f"Dimensions du VRAI rectangle total: {total_width_mm}mm x {total_height_mm}mm")

# Dimensions de l'affichage du rectangle transformé (en pixels)
display_width = 1000
display_height = int(display_width * total_height_mm / total_width_mm)

print(f"Dimensions d'affichage: {display_width}px x {display_height}px")

# Coordonnées de destination pour l'homographie (rectangle redressé)
# Les coins extérieurs des tags dans l'espace transformé
scale_factor = display_width / total_width_mm

dst_tags_coords = np.float32([
    [((margin_horizontal-100) + tags_width_mm) * scale_factor, (margin_vertical+70) * scale_factor],  # Tag 22
    [(margin_horizontal+100) * scale_factor, (margin_vertical+70) * scale_factor],                    # Tag 23
    [(margin_horizontal+100) * scale_factor, ((margin_vertical-70) + tags_height_mm) * scale_factor], # Tag 21
    [((margin_horizontal-100) + tags_width_mm) * scale_factor, ((margin_vertical-70) + tags_height_mm) * scale_factor]  # Tag 20
])

# Dictionnaire pour associer les IDs des tags à leurs coordonnées de destination
tag_id_to_dst = {
    22: dst_tags_coords[0],
    23: dst_tags_coords[1],
    21: dst_tags_coords[2],
    20: dst_tags_coords[3]
}

homography_matrix = None
calibration_done = False
last_valid_homography = None  # Pour mémoriser la dernière homographie valide

print("\nNouveau système de coordonnées:")
print("- Origine (0, 0) : Coin supérieur DROIT du vrai rectangle")
print("- Point opposé (1430, 900) : Coin inférieur GAUCHE du vrai rectangle")
print("\nInstructions:")
print("- Placez les 4 tags ArUco (20, 21, 22, 23) dans le champ de vision")
print("- Appuyez sur 'c' pour calibrer l'homographie")
print("- L'homographie continuera même avec moins de 4 tags après calibration")
print("- Appuyez sur 'q' pour quitter")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Impossible de lire l'image de la caméra")
        break

    # Correction de la distorsion
    h, w = frame.shape[:2]
    undistorted_frame = cv2.undistort(frame, mtx, dist, None, mtx)

    # Conversion en niveaux de gris
    gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)

    # Détection des marqueurs Aruco
    corners, ids, rejected = detector.detectMarkers(gray)

    display_frame = undistorted_frame.copy()

    if ids is not None:
        # Dessiner les marqueurs détectés
        cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)

        # Vérifier si les 4 tags requis sont présents
        required_ids = {20, 21, 22, 23}
        detected_ids = set(ids.flatten())

        # Compter combien de tags de calibration sont visibles
        calibration_tags_detected = required_ids.intersection(detected_ids)
        num_calibration_tags = len(calibration_tags_detected)

        if num_calibration_tags >= 3:  # Au moins 3 tags sur 4
            # Créer un dictionnaire pour stocker les coins des tags
            tag_corners_dict = {}

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id in required_ids:
                    # Les coins du marqueur (dans l'ordre: haut-gauche, haut-droit, bas-droit, bas-gauche)
                    marker_corners = corners[i][0]
                    tag_corners_dict[marker_id] = marker_corners
                    
                    # Calculer le centre pour l'affichage de l'ID
                    center = np.mean(marker_corners, axis=0)
                    cv2.circle(display_frame, tuple(center.astype(int)), 5, (0, 255, 0), -1)
                    cv2.putText(display_frame, f"ID:{marker_id}", 
                              tuple(center.astype(int) + np.array([10, -10])),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Dessiner le rectangle formé par les coins EXTÉRIEURS des tags visibles
            if len(tag_corners_dict) >= 3:
                # Préparer les listes de points source et destination
                src_points = []
                dst_points = []

                for tag_id in sorted(tag_corners_dict.keys()):
                    # Coin extérieur selon l'ID du tag
                    if tag_id == 22:
                        src_points.append(tag_corners_dict[22][3])  # coin EXTÉRIEUR
                    elif tag_id == 23:
                        src_points.append(tag_corners_dict[23][2])  # coin EXTÉRIEUR
                    elif tag_id == 21:
                        src_points.append(tag_corners_dict[21][1])  # coin EXTÉRIEUR
                    elif tag_id == 20:
                        src_points.append(tag_corners_dict[20][0])  # coin EXTÉRIEUR
                    
                    dst_points.append(tag_id_to_dst[tag_id])

                src_coords_available = np.float32(src_points)
                dst_coords_available = np.float32(dst_points)

                # Dessiner les tags visibles
                if len(tag_corners_dict) == 4:
                    rectangle_tags_points = np.array([
                        tag_corners_dict[23][2],  # Tag 23
                        tag_corners_dict[22][3],  # Tag 22
                        tag_corners_dict[20][0],  # Tag 20
                        tag_corners_dict[21][1]   # Tag 21
                    ], dtype=np.int32)
                    cv2.polylines(display_frame, [rectangle_tags_points], True, (255, 0, 0), 2)
                    cv2.putText(display_frame, "Rectangle des tags (4/4)", 
                              tuple(rectangle_tags_points[0] - np.array([0, 10])),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                else:
                    # Dessiner les points des tags visibles
                    for point in src_coords_available:
                        cv2.circle(display_frame, tuple(point.astype(int)), 8, (255, 0, 0), -1)
                    cv2.putText(display_frame, f"Tags visibles: {num_calibration_tags}/4", 
                              (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)

                # Calculer ou mettre à jour l'homographie
                if len(src_points) >= 4:
                    # On a assez de points pour calculer une homographie
                    H_temp, _ = cv2.findHomography(src_coords_available, dst_coords_available)
                    if H_temp is not None:
                        homography_matrix = H_temp
                        last_valid_homography = H_temp
                    
                    # Calculer l'homographie inverse pour le rectangle vert
                    H_tags_to_camera, _ = cv2.findHomography(dst_coords_available, src_coords_available)
                    
                    if H_tags_to_camera is not None:
                        # Calculer les 4 coins du VRAI rectangle dans l'espace caméra
                        real_corners_transformed = np.float32([
                            [0, 0],
                            [display_width, 0],
                            [display_width, display_height],
                            [0, display_height]
                        ])

                        real_corners_camera = cv2.perspectiveTransform(
                            real_corners_transformed.reshape(-1, 1, 2), 
                            H_tags_to_camera
                        ).reshape(-1, 2)

                        # Dessiner le VRAI rectangle étendu
                        real_rect_camera = real_corners_camera.astype(np.int32)
                        cv2.polylines(display_frame, [real_rect_camera], True, (0, 255, 0), 3)
                        cv2.putText(display_frame, "VRAI rectangle complet", 
                                  tuple(real_rect_camera[0] - np.array([0, 20])),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                elif len(src_points) == 3 and last_valid_homography is not None:
                    # Mode 3 tags : on utilise la dernière homographie valide
                    homography_matrix = last_valid_homography
                    cv2.putText(display_frame, "Mode 3 tags (derniere calibration)", 
                              (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 165, 0), 2)

                # Afficher la vue transformée si calibré
                if calibration_done and homography_matrix is not None:
                    # Appliquer la transformation
                    warped = cv2.warpPerspective(undistorted_frame, homography_matrix, 
                                                 (display_width, display_height))

                    # Dessiner le VRAI rectangle complet
                    cv2.rectangle(warped, (0, 0), (display_width-1, display_height-1), 
                                (0, 255, 0), 3)

                    # Détecter TOUS les marqueurs ArUco dans la vue transformée
                    warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
                    warped_corners, warped_ids, _ = detector.detectMarkers(warped_gray)

                    if warped_ids is not None:
                        cv2.aruco.drawDetectedMarkers(warped, warped_corners, warped_ids)

                        for i, marker_id in enumerate(warped_ids.flatten()):
                            marker_corners_warped = warped_corners[i][0]
                            center_px = np.mean(marker_corners_warped, axis=0)

                            x_old_mm = (center_px[0] / display_width) * total_width_mm
                            y_old_mm = (center_px[1] / display_height) * total_height_mm

                            x_new_mm = total_width_mm - x_old_mm
                            y_new_mm = y_old_mm

                            coord_text = f"ID {marker_id}: ({x_new_mm:.1f}, {y_new_mm:.1f})"
                            print(coord_text)
                            
                            cv2.circle(warped, tuple(center_px.astype(int)), 5, (0, 0, 255), -1)
                            cv2.putText(warped, f"({x_new_mm:.0f},{y_new_mm:.0f})", 
                                      tuple(center_px.astype(int) + np.array([10, -10])),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    # Dessiner l'origine
                    origin_new_px = np.array([display_width, 0])
                    cv2.circle(warped, tuple(origin_new_px.astype(int)), 8, (255, 0, 255), -1)

                    # Afficher le statut
                    if len(src_points) < 4:
                        cv2.putText(warped, f"Mode {num_calibration_tags} tags", 
                                  (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 165, 0), 2)

                    cv2.imshow("VRAI Rectangle Complet (Homographie)", warped)

                cv2.putText(display_frame, f"{num_calibration_tags}/4 tags - Appuyez sur 'c' pour calibrer", 
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            missing = required_ids - detected_ids
            cv2.putText(display_frame, f"Tags manquants: {missing} (besoin de 3 minimum)", 
                      (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Si on a déjà une homographie valide, on continue à l'utiliser
            if calibration_done and last_valid_homography is not None:
                homography_matrix = last_valid_homography
                warped = cv2.warpPerspective(undistorted_frame, homography_matrix, 
                                             (display_width, display_height))
                cv2.rectangle(warped, (0, 0), (display_width-1, display_height-1), (0, 0, 255), 3)
                cv2.putText(warped, "MODE DEGRADE: Derniere calibration", 
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                cv2.imshow("VRAI Rectangle Complet (Homographie)", warped)

    else:
        # Aucun tag détecté
        if calibration_done and last_valid_homography is not None:
            # Continuer avec la dernière homographie
            homography_matrix = last_valid_homography
            warped = cv2.warpPerspective(undistorted_frame, homography_matrix, 
                                         (display_width, display_height))
            cv2.rectangle(warped, (0, 0), (display_width-1, display_height-1), (0, 0, 255), 3)
            cv2.putText(warped, "AUCUN TAG: Derniere calibration", 
                      (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.imshow("VRAI Rectangle Complet (Homographie)", warped)
        
        cv2.putText(display_frame, "Aucun tag detecte", 
                  (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("Détection ArUco", display_frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('c'):
        if ids is not None:
            required_ids = {20, 21, 22, 23}
            detected_ids = set(ids.flatten())
            calibration_tags = required_ids.intersection(detected_ids)
            
            if len(calibration_tags) >= 4:
                tag_corners_dict = {}
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id in required_ids:
                        marker_corners = corners[i][0]
                        tag_corners_dict[marker_id] = marker_corners

                src_coords = np.float32([
                    tag_corners_dict[22][3],
                    tag_corners_dict[23][2],
                    tag_corners_dict[21][1],
                    tag_corners_dict[20][0]
                ])

                homography_matrix, _ = cv2.findHomography(src_coords, dst_tags_coords)
                last_valid_homography = homography_matrix
                calibration_done = True
                print("\n=== Homographie calibrée avec succès! ===")
                print("Matrice d'homographie:")
                print(homography_matrix)
                print(f"\nLe système peut maintenant fonctionner avec moins de 4 tags")
            else:
                print(f"Calibration nécessite les 4 tags. Actuellement: {len(calibration_tags)}/4")
        else:
            print("Aucun tag détecté")

    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
