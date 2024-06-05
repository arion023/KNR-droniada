from CV.ball_finder import *
img_path  = "images/output_33.jpg"
find_colors(img_path)

"""
import cv2
import numpy as np

#po ustawienieu wartosci C rysuje prostokat
# q zabija program

# Zółta piłka
# H = <0,50>
# s = <100, 255>
# v = <190, 255> 

# "Ceglana" piłka
# H = <110,180>
# S = <0,240>
# V = <135,255>

# Niebieska pilka
# H = <95,150>
# S = <110,255>
# V = <175,230> 

# Fioletowa pilka 
# H = <105,120>
# S = <115,255> 
# V = <100,240>


# Funkcje ruchu (przykładowe)
def move_left():
    print("Moving left")

def move_right():
    print("Moving right")

def move_up():
    print("Moving up")

def move_down():
    print("Moving down")

# Funkcja nic nie robiąca dla suwaków trackbar
def nothing(x):
    pass

# Funkcja do wykrywania koloru i rysowania prostokąta
def detect_color(frame, lower_color, upper_color, draw_rectangle):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        if draw_rectangle:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return (x, y, w, h), mask
    return None, mask

# Funkcja do obliczania komendy sterującej na podstawie błędu
def compute_control_command(target, frame_center):
    x, y, w, h = target
    target_center = (x + w // 2, y + h // 2)
    
    error_x = frame_center[0] - target_center[0]
    error_y = frame_center[1] - target_center[1]
    
    return error_x, error_y

# Wczytanie obrazu z pliku
image_path = ''  # Zamień tę ścieżkę na rzeczywistą ścieżkę do obrazu
frame = cv2.imread(image_path)

# Sprawdzenie czy obraz został poprawnie wczytany
if frame is None:
    print(f"Error: Unable to load image at {image_path}")
    exit()

# Zmniejszenie rozmiaru obrazu
scale_percent = 30  # Skaluje obraz do 50% oryginalnego rozmiaru
width = int(frame.shape[1] * scale_percent / 100)
height = int(frame.shape[0] * scale_percent / 100)
dim = (width, height)

# Utworzenie okna z suwakami do kalibracji koloru
cv2.namedWindow('Trackbars')
cv2.createTrackbar('LH', 'Trackbars', 0, 180, nothing)
cv2.createTrackbar('LS', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('LV', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('UH', 'Trackbars', 180, 180, nothing)
cv2.createTrackbar('US', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('UV', 'Trackbars', 255, 255, nothing)

draw_rectangle = False

while True:
    # Tworzenie kopii obrazu do przetwarzania
    resized_frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
    frame_center = (resized_frame.shape[1] // 2, resized_frame.shape[0] // 2)

    # Pobieranie aktualnych wartości z suwaków
    lh = cv2.getTrackbarPos('LH', 'Trackbars')
    ls = cv2.getTrackbarPos('LS', 'Trackbars')
    lv = cv2.getTrackbarPos('LV', 'Trackbars')
    uh = cv2.getTrackbarPos('UH', 'Trackbars')
    us = cv2.getTrackbarPos('US', 'Trackbars')
    uv = cv2.getTrackbarPos('UV', 'Trackbars')

    lower_color = np.array([lh, ls, lv])
    upper_color = np.array([uh, us, uv])

    target, mask = detect_color(resized_frame, lower_color, upper_color, draw_rectangle)

    if target:
        error_x, error_y = compute_control_command(target, frame_center)
        
        # Proste sterowanie ruchem na podstawie błędu
        threshold = 10  # Próg martwej strefy dla błędu
        if abs(error_x) > threshold:
            if error_x > 0:
                move_right()
            else:
                move_left()
        
        if abs(error_y) > threshold:
            if error_y > 0:
                move_down()
            else:
                move_up()
        
        x, y, w, h = target
       # print(f"Detected object at X: {x}, Y: {y}, Width: {w}, Height: {h}")
       # print(f"Control Command X: {error_x}, Control Command Y: {error_y}")
    
    cv2.imshow('Frame', resized_frame)
    cv2.imshow('Mask', mask)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        draw_rectangle = not draw_rectangle

cv2.destroyAllWindows()

"""