import cv2
import numpy as np

# Zestawy wartości HSV dla różnych obiektów
hsv_values = {
<<<<<<< HEAD
    "Zolta pilka": {"lower": np.array([20, 80, 0]), "upper": np.array([50, 255, 255]), "color": (0, 255, 255)},
    "Ceglana pilka": {"lower": np.array([40, 40, 50]), "upper": np.array([180, 50, 255]), "color": (0, 0, 255)},  # Czerwony kolor ramki
    "Niebieska pilka": {"lower": np.array([60, 60, 0]), "upper": np.array([110, 255, 255]), "color": (255, 0, 0)},
    "Fioletowa pilka": {"lower": np.array([120, 40, 0]), "upper": np.array([160, 255, 255]), "color": (255, 0, 255)},
    "Biala plachta": {"lower": np.array([0, 0, 180]), "upper": np.array([180, 50, 255]), "color": (0, 0, 255)}
=======
    "Zolta pilka": {"lower": np.array([0, 100, 190]), "upper": np.array([50, 255, 255])},
    "Ceglana pilka": {"lower": np.array([145, 95, 150]), "upper": np.array([180, 255, 255])},
    "Niebieska pilka": {"lower": np.array([95, 110, 175]), "upper": np.array([150, 255, 230])},
    "Fioletowa pilka": {"lower": np.array([90, 90, 0]), "upper": np.array([130, 255, 255])}
>>>>>>> 3900ed7579983b30392214576366acd1bfa2a47d
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
    
    targets = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if draw_rectangle:
<<<<<<< HEAD
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        targets.append((x, y, w, h))
    return targets, mask
=======
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return (x, y, w, h), mask, cv2.contourArea(c)
    return None, mask, 0
>>>>>>> 3900ed7579983b30392214576366acd1bfa2a47d

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
<<<<<<< HEAD
image_path = r''  # Zamień tę ścieżkę na rzeczywistą ścieżkę do obrazu
=======
image_path = r'C:\Users\hyper\OneDrive\Desktop\zdjecia z podlotu\czerwona1.jpg'  # Zamień tę ścieżkę na rzeczywistą ścieżkę do obrazu
>>>>>>> 3900ed7579983b30392214576366acd1bfa2a47d
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

<<<<<<< HEAD
    # Stałe do regulacji
    movement_threshold = 30  # Minimalna odległość, aby ruch był wykonywany 
                             # wartości dla threshold są podawane jako różnica bezwzględna współrzędnych x i y  
                             # Trzeba wziąć poprawkę na to, że kamera celuje sobą, a nie chwytakiem, czyli przesuwać wychylenie względem osi y o jakąś wartość
   
    # Regulacja ruchu wzdłuż osi X
    if abs(error_x) > movement_threshold:
        if error_x > 0:
            move_left()  # Prostokąt przesunięty w prawo zgodnie do zwrotu x
            print('Przesunięcie w lewo')
        else:
            move_right()  # Prostokąt przesunięty przeciwnie do zwrotu x
            print('Przesunięcie w prawo')
=======
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
>>>>>>> 3900ed7579983b30392214576366acd1bfa2a47d

    # Regulacja ruchu wzdłuż osi Y
    if abs(error_y) > movement_threshold:
        if error_y > 0:
<<<<<<< HEAD
            move_forward()  # Prostokąt przesunięty zgodnie do zwrotu y
            print('Przesunięcie w przód') 
        else:
            move_back()  # Prostokąt przesunięty przeciwnie do zwrotu y
            print('Przesunięcie do tyłu')
=======
            move_forward()
            print('Przesuniecie do przodu') 
        else:
            move_back()
            print('Przesuniecie do tyłu')
>>>>>>> 3900ed7579983b30392214576366acd1bfa2a47d

while True:
    # Skalowanie obrazu
    resized_frame = cv2.resize(frame, dim, interpolation=cv2.INTER_AREA)
    frame_center = (resized_frame.shape[1] // 2, resized_frame.shape[0] // 2)

    # Wykrywanie białych obszarów
    lower_white = hsv_values["Biala plachta"]["lower"]
    upper_white = hsv_values["Biala plachta"]["upper"]
    white_color = hsv_values["Biala plachta"]["color"]
    white_targets, white_mask = detect_color(resized_frame, lower_white, upper_white, white_color, draw_rectangle)

    best_area = 0
    best_target = None
    best_mask = None
    best_color_name = None

<<<<<<< HEAD
    for white_target in white_targets:
        x, y, w, h = white_target
        white_area_frame = resized_frame[y:y+h, x:x+w]

        # Przeszukiwanie wszystkich zestawów wartości HSV w białych obszarach
        for color_name, hsv in hsv_values.items():
            if color_name == "Biala plachta":
                continue  # Pomijanie detekcji białych obszarów ponownie

            lower_color = hsv["lower"]
            upper_color = hsv["upper"]
            color = hsv.get("color", (0, 0, 0))  # Default to black if color is not specified
            targets, mask = detect_color(white_area_frame, lower_color, upper_color, color, draw_rectangle)
            
            for target in targets:
                tx, ty, tw, th = target
                area = tw * th
                if area > best_area:
                    best_area = area
                    best_target = (x + tx, y + ty, tw, th)
                    best_mask = mask
                    best_color_name = color_name
                    best_color = color
=======
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
>>>>>>> 3900ed7579983b30392214576366acd1bfa2a47d

    # Wykonanie regulacji ruchu
    if best_target:
        adjust_rectangle_position(best_target, frame_center)

    # Obliczanie gradientu
    step = calculate_gradient(best_target, frame_center)
    print(f"step: {step}")

    # Wyświetlanie zmniejszonego obrazu
<<<<<<< HEAD
    cv2.imshow('Resized Frame', resized_frame)
    cv2.imshow('White Mask', white_mask)
=======
    if best_color_name:
        cv2.putText(resized_frame, best_color_name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

    cv2.imshow('Resized Frame', resized_frame)
    cv2.imshow('Mask', best_mask)
>>>>>>> 3900ed7579983b30392214576366acd1bfa2a47d

    # Oczekiwanie na klawisz i zakończenie pętli w przypadku naciśnięcia klawisza 'q'
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        draw_rectangle = not draw_rectangle

# Zamknięcie wszystkich okien
cv2.destroyAllWindows()
