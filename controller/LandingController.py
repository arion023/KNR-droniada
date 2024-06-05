import cv2
import numpy as np

"""
Najpierw program iteruje po wszystkich maskach i sprawdza ktorej uzyc
potem robi maske i rysuje prostokat dookola obszaru najwiekszego konturu dla danego
koloru nastepnie algorytm stara sie utrzymac dany prostokat w srodku obrazu kamery
przez wydawanie konkretnych komend do drona. 

"""

# Zestawy wartości HSV dla różnych obiektów
hsv_values = {
    "Zolta pilka": {"lower": np.array([0, 100, 190]), "upper": np.array([50, 255, 255])},
    "Ceglana pilka": {"lower": np.array([145, 95, 150]), "upper": np.array([180, 255, 255])},
    "Niebieska pilka": {"lower": np.array([95, 110, 175]), "upper": np.array([150, 255, 230])},
    "Fioletowa pilka": {"lower": np.array([90, 90, 0]), "upper": np.array([130, 255, 255])}
}

# Funkcje ruchu (przykładowe)
def move_left():
    #print("Moving left")
    pass

def move_right():
    #print("Moving right")
    pass

def move_forward():
    #print("Moving up")
    pass

def move_back():
    #print("Moving down")
    pass

# Funkcja nic nie robiąca dla suwaków trackbar
def nothing(x):
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

# Wczytanie obrazu z pliku
image_path = r'C:\Users\Turlaq\Desktop\zolta.jpg'  # Zamień tę ścieżkę na rzeczywistą ścieżkę do obrazu
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
#cv2.namedWindow('Trackbars')
#cv2.createTrackbar('LH', 'Trackbars', 0, 180, nothing)
#cv2.createTrackbar('LS', 'Trackbars', 0, 255, nothing)
#cv2.createTrackbar('LV', 'Trackbars', 0, 255, nothing)
#cv2.createTrackbar('UH', 'Trackbars', 180, 180, nothing)
#cv2.createTrackbar('US', 'Trackbars', 255, 255, nothing)
#cv2.createTrackbar('UV', 'Trackbars', 255, 255, nothing)

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

    # Stałe do regulacji
    movement_threshold = 30  # Minimalna odległość, aby ruch był wykonywany 
                             # wartosci dla threshold sa podawane jak roznica bezwgledna wspolrzednych x i y  
                             # Trzeba wziac poprawke na to ze kamera celuje soba a nie chwytakiem czyli przesuwac wychlenie wzgledem osi y o jakas wartosc
   
    # Regulacja ruchu wzdłuż osi X
    if abs(error_x) > movement_threshold:
        if error_x > 0:
            move_left()  # Prostokąt przesunięty w prawo zgodnie do zwrotu x
            print('Przesuniecie w lewo')
        else:
            move_right()  # Prostokąt przesunięty przeciwnie do zwrotu x
            print('Przesuniecie w prawo')
        
            

    # Regulacja ruchu wzdłuż osi Y
    if abs(error_y) > movement_threshold:
        if error_y > 0:
            move_forward()  # Prostokąt przesunięty zgodnie do zwrotu y
            print('Przesuniecie w przodu') 
        else:
            move_back()  # Prostokąt przesunięty przeciwnie do zwrotu y
            print('Przesuniecie do tyłu ')

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