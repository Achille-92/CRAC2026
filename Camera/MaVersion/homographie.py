import cv2
import numpy as np

# Charger les paramètres de calibration
data = np.load('calibration_data.npz')
mtx, dist = data['mtx'], data['dist']

# Paramètres de la caméra
CAMERA_INDEX = 1  # 0 = caméra intégrée, 1 = webcam USB

# Initialisation de la caméra
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    print("Erreur : impossible d'ouvrir la caméra")
    exit()

# Configuration de la résolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)

# Création du dictionnaire et du détecteur ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(aruco_dict)

# Coordonnées réelles des 4 tags Aruco sur la piste (en mm)
# Les tags sont placés aux coins rapprochés de la piste
# Exemple : (x, y) en partant du coin inférieur gauche de la piste
aruco_real_coords = np.float32([
    [0, 2000],  # Coin supérieur gauche (ID 20)
    [0, 0],    # Coin inférieur gauche (ID 21)
    [3000, 0],       # Coin inférieur droit (ID 22)
    [3000, 2000]     # Coin supérieur droit (ID 23)
])

# Liste pour stocker les coins des marqueurs détectés
detected_corners = []

print("Placez les 4 tags Aruco (IDs 20, 21, 22, 23) dans le champ de vision de la caméra. Appuyez sur 'c' pour capturer les positions, puis 'q' pour quitter.")

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

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(undistorted_frame, corners, ids)

        # Tri des marqueurs par ID
        sorted_indices = np.argsort(ids.flatten())
        sorted_corners = corners[sorted_indices]
        sorted_ids = ids[sorted_indices]

        # Vérification que les IDs 20, 21, 22 et 23 sont présents
        required_ids = {20, 21, 22, 23}
        if set(sorted_ids.flatten()) == required_ids:
            # Calcul des centres des marqueurs
            centers = []
            for i in range(len(sorted_corners)):
                center = np.mean(sorted_corners[i][0], axis=0)
                centers.append(center)

            # Affichage des centres
            for center in centers:
                cv2.circle(undistorted_frame, tuple(center.astype(int)), 5, (0, 0, 255), -1)

    cv2.imshow("Calibration Homographie", undistorted_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('c'):
        if ids is not None and set(ids.flatten()) == required_ids:
            detected_corners = np.float32(centers)
            print("Positions des tags capturées.")
        else:
            print("Il faut exactement les tags Aruco avec les IDs 20, 21, 22 et 23 pour effectuer la calibration.")
    elif key == ord('q'):
        break

# Calcul de la matrice d'homographie
if len(detected_corners) == 4:
    homography_matrix, _ = cv2.findHomography(detected_corners, aruco_real_coords)
    print("Matrice d'homographie calculée :\n", homography_matrix)
else:
    print("Impossible de calculer la matrice d'homographie : les tags Aruco avec les IDs 20, 21, 22 et 23 sont requis.")
    exit()

# Boucle principale pour détecter les tags et afficher leurs coordonnées
print("Appuyez sur 'q' pour quitter.")

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

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(undistorted_frame, corners, ids)

        # Calcul des centres des marqueurs
        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]
            center = np.mean(marker_corners, axis=0)
            center = np.array([center], dtype=np.float32)

            # Transformation des coordonnées avec la matrice d'homographie
            transformed_center = cv2.perspectiveTransform(center, homography_matrix)

            # Coordonnées transformées
            x_transformed = int(transformed_center[0][0][0])
            y_transformed = int(transformed_center[0][0][1])

            # Affichage des coordonnées
            print(f"Marqueur détecté : ID = {marker_id}, X = {x_transformed}, Y = {y_transformed}")

            # Affichage du centre transformé
            cv2.circle(undistorted_frame, tuple(center[0].astype(int)), 5, (0, 255, 0), -1)

    cv2.imshow("ArUco Detection (corrigé)", undistorted_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
