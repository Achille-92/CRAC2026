import cv2
import numpy as np

# Charger les paramètres de calibration
data = np.load('calibration_data.npz')
mtx, dist = data['mtx'], data['dist']

# Paramètres
CAMERA_INDEX = 0  # 0 = caméra intégrée, 1 = webcam USB

# Initialisation caméra
cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
if not cap.isOpened():
    print("Erreur : impossible d'ouvrir la caméra")
    exit()

# Configuration de la résolution (à adapter selon votre caméra)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Création du dictionnaire et du détecteur ArUco
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
detector = cv2.aruco.ArucoDetector(aruco_dict)

print("Appuyez sur 'q' pour quitter.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Impossible de lire l'image de la caméra")
        break

    # Correction de la distorsion
    h, w = frame.shape[:2]
    undistorted_frame = cv2.undistort(frame, mtx, dist, None, mtx)  # Utilisez mtx comme newcameramtx

    # Conversion en niveaux de gris
    gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)

    # Détection des marqueurs
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(undistorted_frame, corners, ids)
        for i, marker_id in enumerate(ids.flatten()):
            marker_corners = corners[i][0]
            center_x = int(sum([corner[0] for corner in marker_corners]) / 4)
            center_y = int(sum([corner[1] for corner in marker_corners]) / 4)

            # Transformation des coordonnées
            height, width = undistorted_frame.shape[:2]
            x_transformed = int((width - center_x) * (3000 / width))
            y_transformed = int(center_y * (2000 / height))
            print(f"Marqueur détecté : ID = {marker_id}, X = {x_transformed}, Y = {y_transformed}")

    cv2.imshow("ArUco Detection (corrigé)", undistorted_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
