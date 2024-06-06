import cv2
import numpy as np

# Zestawy wartości HSV dla różnych obiektów i kolory ramki
hsv_values = {
    "Zolta pilka": {"lower": np.array([20, 80, 0]), "upper": np.array([50, 255, 255]), "color": (0, 255, 255)},
    "Ceglana pilka": {"lower": np.array([40, 40, 50]), "upper": np.array([180, 50, 255]), "color": (0, 0, 255)},  # Czerwony kolor ramki
    "Niebieska pilka": {"lower": np.array([60, 60, 0]), "upper": np.array([110, 255, 255]), "color": (255, 0, 0)},
    "Fioletowa pilka": {"lower": np.array([120, 40, 0]), "upper": np.array([160, 255, 255]), "color": (255, 0, 255)},
    "Biala plachta": {"lower": np.array([0, 0, 180]), "upper": np.array([180, 50, 255]), "color": (0, 0, 255)}
}

# Wczytanie obrazu z pliku
image_path = 'images/test_img/czerwona1.jpg'  # Zamień tę ścieżkę na rzeczywistą ścieżkę do obrazu
frame = cv2.imread(image_path)

# Funkcje ruchu (przykładowe)
def move_left():
    print('W lewo')
    pass

def move_right():
    print('W prawo')
    pass

def move_forward():
    print('W przod')
    pass

def move_back():
    print('W tyl')
    pass

# Funkcja do wykrywania koloru
def detect_color(frame, lower_color, upper_color, color, draw_rectangle):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_color, upper_color)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    targets = []
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        if draw_rectangle:
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        targets.append((x, y, w, h))
    return targets, mask

# Funkcja do obliczania komendy sterującej na podstawie błędu
def compute_control_command(target, frame_center):
    x, y, w, h = target
    target_center = (x + w // 2, y + h // 2)
    
    error_x = frame_center[0] - target_center[0]
    error_y = frame_center[1] - target_center[1]
    
    return error_x, error_y


def process_image(image_path):
    # Zestawy wartości HSV dla różnych obiektów i kolory ramki
    hsv_values = {
        "Zolta pilka": {"lower": np.array([20, 80, 0]), "upper": np.array([50, 255, 255]), "color": (0, 255, 255)},
        "Ceglana pilka": {"lower": np.array([40, 40, 50]), "upper": np.array([180, 50, 255]), "color": (0, 0, 255)},
        "Niebieska pilka": {"lower": np.array([60, 60, 0]), "upper": np.array([110, 255, 255]), "color": (255, 0, 0)},
        "Fioletowa pilka": {"lower": np.array([120, 40, 0]), "upper": np.array([160, 255, 255]), "color": (255, 0, 255)},
        "Biala plachta": {"lower": np.array([0, 0, 180]), "upper": np.array([180, 50, 255]), "color": (0, 0, 255)}
    }

# Wczytanie obrazu z plikulower": np.array([0, 0, 180]), "upper": np.array([180, 50, 255]), "color": (0, 0, 255)}


    # Wczytanie obrazu
    frame = cv2.imread(image_path)

    # Sprawdzenie czy obraz został poprawnie wczytany
    if frame is None:
        print(f"Error: Unable to load image at {image_path}")
        return None

    # Zmniejszenie rozmiaru obrazu
    scale_percent = 50  # Skaluje obraz do 50% oryginalnego rozmiaru
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

        # Stałe do regulacji
        movement_threshold = 30  # Minimalna odległość, aby ruch był wykonywany
                                 # wartości dla threshold są podawane jako różnica bezwzględna współrzędnych x i y
                                 # Trzeba wziąć poprawkę na to, że kamera celuje sobą, a nie chwytakiem, czyli przesuwać wychylenie względem osi y o jakąś wartość

        # Sprawdzanie priorytetów kolorów
        if best_color_name == "Niebieska pilka":
            # Regulacja ruchu wzdłuż osi X
            if abs(error_x) > movement_threshold:
                if error_x > 0:
                    move_left()  # Prostokąt przesunięty w prawo zgodnie do zwrotu x
                    print('Przesunięcie w lewo')
                else:
                    move_right()  # Prostokąt przesunięty przeciwnie do zwrotu x
                    print('Przesunięcie w prawo')

            # Regulacja ruchu wzdłuż osi Y
            if abs(error_y) > movement_threshold:
                if error_y > 0:
                    move_forward()  # Prostokąt przesunięty zgodnie do zwrotu y
                    print('Przesunięcie w przód') 
                else:
                    move_back()  # Prostokąt przesunięty przeciwnie do zwrotu y
                    print('Przesunięcie do tyłu')
        elif best_color_name == "Ceglana pilka":
            # Działania dla obszaru ceglanego
            pass  # Tutaj można dodać odpowiednie operacje dla obszaru ceglanego
        elif best_color_name == "Fioletowa pilka":
            # Działania dla obszaru fioletowego
            pass  # Tutaj można dodać odpowiednie operacje dla obszaru fioletowego

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
        best_color = None

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

        # Wykonanie regulacji ruchu
        if best_target:
            adjust_rectangle_position(best_target, frame_center)


        # Wyświetlanie zmniejszonego obrazu
        cv2.imshow('Resized Frame', resized_frame)
        cv2.imshow('White Mask', white_mask)

        # Oczekiwanie na klawisz i zakończenie pętli w przypadku naciśnięcia klawisza 'q'
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            draw_rectangle = not draw_rectangle

    # Zamknięcie wszystkich okien
    cv2.destroyAllWindows()


    if __name__ == "__main__":
        zmienna = process_image('images/test_img/czerwona1.jpg')
        print(zmienna)
    


