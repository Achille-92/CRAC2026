import cv2
import numpy as np
import glob

# Taille du damier (nombre de coins intérieurs)
chessboard_size = (9, 6)  # À adapter selon votre damier, faire -1 dans chaque sens

# Préparation des points 3D du damier
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# Liste pour stocker les points 3D et 2D
objpoints = []
imgpoints = []

# Chargement des images du damier
images = glob.glob('calibration_images/*.jpg')  # À adapter selon l'extension

if not images:
    print("Aucune image trouvée dans le dossier 'calibration_images'.")
    exit()

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Détection des coins du damier
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)

        # Dessiner les coins détectés pour vérification
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
        cv2.imshow('Damier détecté', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Étalonnage de la caméra
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Sauvegarde des paramètres
np.savez('calibration_data.npz', mtx=mtx, dist=dist)
print("Étalonnage terminé. Matrice et coefficients sauvegardés dans 'calibration_data.npz'.")
