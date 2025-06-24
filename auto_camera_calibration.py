import cv2
import numpy as np
import os
import yaml
from datetime import datetime

# === Caminhos base ===
base_dir = os.path.dirname(os.path.abspath(__file__))  # Pasta Aruco_ROS_Noetic
calib_dir = os.path.join(base_dir, "camera_calibration")
captures_dir = os.path.join(calib_dir, "captures")

# === Parâmetros do tabuleiro ===
chessboard_size = (5, 8)  # cantos internos (colunas, linhas)
square_size = 0.02  # tamanho real dos quadrados em metros ou unidade desejada

# === Inicialização ===
os.makedirs(captures_dir, exist_ok=True)
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Erro: Não foi possível acessar a câmera")
    exit()

print("Pressione [ESPAÇO] para capturar uma imagem com o tabuleiro de xadrez.")
print("Pressione [ENTER] para iniciar a calibração.")
print("Pressione [ESC] para sair.")

captured_images = []

# === Etapa 1: Captura de imagens ===
while True:
    ret, frame = cap.read()
    if not ret:
        print("Erro ao capturar imagem.")
        break

    display_frame = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = 255 - gray  # inverte a imagem

    found, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if found:
        cv2.drawChessboardCorners(display_frame, chessboard_size, corners, found)
        cv2.putText(display_frame, "Tabuleiro detectado", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 0), 2)

    cv2.imshow("Captura de Calibracao", display_frame)

    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC
        break
    elif key == 32:  # ESPAÇO
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = os.path.join(captures_dir, f"capture_{timestamp}.png")
        cv2.imwrite(filename, frame)
        captured_images.append(filename)
        print(f"Imagem salva: {filename}")
    elif key == 13:  # ENTER
        print("Iniciando calibração...")
        break

cap.release()
cv2.destroyAllWindows()

# === Etapa 2: Calibração da câmera ===
if len(captured_images) < 5:
    print("Erro: Capture pelo menos 5 imagens válidas antes de calibrar.")
    exit()

objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []
imgpoints = []

for img_path in captured_images:
    img = cv2.imread(img_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = 255 - gray

    found, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if found:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                    (cv2.TermCriteria_EPS + cv2.TermCriteria_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)
    else:
        print(f"Tabuleiro não encontrado em {img_path}")

# Calibração com OpenCV
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

print("\n=== Resultados da Calibração ===")
print("Matriz da câmera:\n", camera_matrix)
print("Coeficientes de distorção:\n", dist_coeffs.ravel())
print("Erro médio RMS:", ret)

# === Etapa 3: Salvar YAML no formato ROS ===
image_height, image_width = gray.shape
camera_name = "my_webcam"

calib_data = {
    "image_width": image_width,
    "image_height": image_height,
    "camera_name": camera_name,
    "camera_matrix": {
        "rows": 3,
        "cols": 3,
        "data": camera_matrix.flatten().tolist()
    },
    "distortion_model": "plumb_bob",
    "distortion_coefficients": {
        "rows": 1,
        "cols": dist_coeffs.shape[1] if len(dist_coeffs.shape) > 1 else len(dist_coeffs),
        "data": dist_coeffs.flatten().tolist()
    },
    "rectification_matrix": {
        "rows": 3,
        "cols": 3,
        "data": [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]
    },
    "projection_matrix": {
        "rows": 3,
        "cols": 4,
        "data": [
            camera_matrix[0, 0], 0.0, camera_matrix[0, 2], 0.0,
            0.0, camera_matrix[1, 1], camera_matrix[1, 2], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
    }
}

yaml_path = os.path.join(calib_dir, "camera_calibration.yaml")
with open(yaml_path, 'w') as f:
    yaml.dump(calib_data, f, default_flow_style=False)

print(f"\nCalibração salva no formato ROS em: {yaml_path}")

