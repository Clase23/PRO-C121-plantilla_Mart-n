import cv2
import time
import numpy as np

# Guardar el output en un archivo
fourcc = cv2.VideoWriter_fourcc(*'XVID')
output_file = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))

# Iniciar la cámara web
cap = cv2.VideoCapture(0)

# Dar tiempo a la cámara para iniciarse
time.sleep(2)

# Capturar fondo por 60 cuadros
bg = 0
for i in range(60):
    ret, bg = cap.read()
    if not ret:
        continue
    bg = np.flip(bg, axis=1)

# Leer los cuadros de la cámara en tiempo real
while cap.isOpened():
    ret, img = cap.read()
    if not ret:
        break

    img = np.flip(img, axis=1)

    # Convertir de BGR a HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Crear máscaras para el color amarillo
    lower_yellow1 = np.array([20, 100, 100])
    upper_yellow1 = np.array([25, 255, 255])
    mask1 = cv2.inRange(hsv, lower_yellow1, upper_yellow1)

    lower_yellow2 = np.array([26, 100, 100])
    upper_yellow2 = np.array([32, 255, 255])
    mask2 = cv2.inRange(hsv, lower_yellow2, upper_yellow2)

    # Combinar ambas máscaras
    mask = mask1 + mask2
    cv2.imshow("mask_1", mask1)

    # Eliminar ruido de la máscara
    #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    #mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=1)

    # Invertir máscara
    #mask_inv = cv2.bitwise_not(mask)

    # Obtener sólo las partes de la imagen sin el color rojo
    #res1 = cv2.bitwise_and(img, img, mask=mask_inv)

    # Obtener sólo las partes del fondo donde estaba el color rojo
    #res2 = cv2.bitwise_and(bg, bg, mask=mask)

    # Combinar ambas imágenes
    #final_output = cv2.addWeighted(res1, 1, res2, 1, 0)

    #output_file.write(final_output)
    #cv2.imshow("Invisibility Cloak", final_output)

    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
output_file.release()
cv2.destroyAllWindows()