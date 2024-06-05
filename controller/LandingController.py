import cv2
import numpy as np

# Zestawy wartości HSV dla różnych obiektów
hsv_values = {
    "Zolta pilka": {"lower": np.array([0, 100, 190]), "upper": np.array([50, 255, 255])},
    "Ceglana pilka": {"lower": np.array([145, 95, 150]), "upper": np.array([180, 255, 255])},
    "Niebieska pilka": {"lower": np.array([95, 110, 175]), "upper": np.array([150, 255, 230])},
    "Fioletowa pilka": {"lower": np.array([90, 90, 0]), "upper": np.array([130, 255, 255])}
}

# Funkcje ruchu (przykładowe)
def move_left():
    pass

def move_right():
    pass

def move_forward():
    pass

def move_back():
    pass

# Funkcja do wykrywania koloru
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
        return (x, y, w, h), mask, cv2.contourArea(c)
    return None, mask, 0

# Funkcja do obliczania komendy sterującej na podstawie błędu
def compute_control_command(target, frame_center):
    x, y, w, h = target
    target_center = (x + w // 2, y + h // 2)
    
    error_x = frame_center[0] - target_center[0]
    error_y = frame_center[1] - target_center[1]
    
    return error_x, error_y

# Funkcja do obliczania gradientu
def calculate_gradient(target, frame_center):
    if target is None:
        return float('inf')  # Jeśli nie wykryto piłki, gradient jest nieskończonością

    x, y, w, h = target
    target_center_x = x + w // 2
    target_center_y = y + h // 2

    # Oblicz euklidesową odległość od środka obrazu do środka piłki
    distance_to_center = np.sqrt((frame_center[0] - target_center_x)**2 + (frame_center[1] - target_center_y)**2)
    
    # Wielkość piłki
    size_of_ball = w + h
    
    # Oblicz gradient
    gradient = distance_to_center / size_of_ball if size_of_ball != 0 else float('inf')
    
    return gradient

# Wczytanie obrazu z pliku
image_path = r'C:\Users\hyper\OneDrive\Desktop\zdjecia z podlotu\4.jpg'  # Zamień tę ścieżkę na rzeczywistą ścieżkę do obrazu
frame = cv2.imread(image_path)

# Sprawdzenie czy obraz został poprawnie wczytany
if frame is None:
    print(f"Error: Unable to load image at {image_path}")
    exit()

# Zmniejszenie rozmiaru obrazu
scale_percent = 30  # Skaluje obraz do 30% oryginalnego rozmiaru
width = int(frame.shape[1] * scale_percent / 100)
height = int(frame.shape[0] * scale_percent / 100)
dim = (width, height)

draw_rectangle = False

def adjust_rectangle_position(target, frame_center):
    if target is None:
        return  # Nie wykryto prostokąta, nie podejmuj żadnych działań

    x, y, w, h = target
    target_center_x = x + w // 2
    target_center_y = y + h // 2

    # Oblicz błąd w pozycji X i Y
    error_x = frame_center[0] - target_center_x
    error_y = frame_center[1] - target_center_y

    # Dynamiczne przeliczanie threshold na podstawie wielkości piłki
    movement_threshold = int((w + h) / 2)  # Przykładowe skalowanie; można dostosować współczynnik

    # Rysowanie okręgu określającego threshold
    cv2.circle(frame, frame_center, movement_threshold, (255, 0, 0), 2)

    print(f"Threshold: {movement_threshold}, Error X: {error_x}, Error Y: {error_y}")

    # Regulacja ruchu wzdłuż osi X
    if abs(error_x) > movement_threshold:
        if error_x > 0:
            move_left()
            print('Przesuniecie w lewo')
        else:
            move_right()
            print('Przesuniecie w prawo')

    # Regulacja ruchu wzdłuż osi Y
    if abs(error_y) > movement_threshold:
        if error_y > 0:
            move_forward()
            print('Przesuniecie do przodu') 
        else:
            move_back()
            print('Przesuniecie do tyłu')

while True:
    # Skalowanie obrazu
    resized_frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
    frame_center = (resized_frame.shape[1] // 2, resized_frame.shape[0] // 2)

    best_area = 0
    best_target = None
    best_mask = None
    best_color_name = None

    # Przeszukiwanie wszystkich zestawów wartości HSV
    for color_name, hsv in hsv_values.items():
        lower_color = hsv["lower"]
        upper_color = hsv["upper"]
        target, mask, area = detect_color(resized_frame, lower_color, upper_color, draw_rectangle)
        
        if area > best_area:
            best_area = area
            best_target = target
            best_mask = mask
            best_color_name = color_name

    # Wykonanie regulacji ruchu
    if best_target:
        adjust_rectangle_position(best_target, frame_center)

    # Obliczanie gradientu
    gradient = calculate_gradient(best_target, frame_center)
    print(f"Gradient: {gradient}")

    # Wyświetlanie zmniejszonego obrazu
    if best_color_name:
        cv2.putText(resized_frame, best_color_name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    cv2.imshow('Resized Frame', resized_frame)
    cv2.imshow('Mask', best_mask)

    # Oczekiwanie na klawisz i zakończenie pętli w przypadku naciśnięcia klawisza 'q'
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        draw_rectangle = not draw_rectangle

# Zamknięcie wszystkich okien
cv2.destroyAllWindows()
