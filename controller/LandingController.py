import cv2
import numpy as np

def process_image(image_path):
    # Zestawy wartości HSV dla różnych obiektów i kolory ramki
    hsv_values = {
        "Zolta pilka": {"lower": np.array([20, 80, 0]), "upper": np.array([50, 255, 255]), "color": (0, 255, 255)},
        "Ceglana pilka": {"lower": np.array([40, 40, 50]), "upper": np.array([180, 50, 255]), "color": (0, 0, 255)},  # Czerwony kolor ramki
        "Niebieska pilka": {"lower": np.array([60, 60, 0]), "upper": np.array([110, 255, 255]), "color": (255, 0, 0)},
        "Fioletowa pilka": {"lower": np.array([120, 40, 0]), "upper": np.array([160, 255, 255]), "color": (255, 0, 255)},
        "Biala plachta": {"lower": np.array([0, 0, 180]), "upper": np.array([180, 50, 255]), "color": (0, 0, 255)}
    }

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

    draw_rectangle = True

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
        return targets, frame

    # Wykrywanie białych obszarów
    lower_white = hsv_values["Biala plachta"]["lower"]
    upper_white = hsv_values["Biala plachta"]["upper"]
    white_color = hsv_values["Biala plachta"]["color"]
    white_targets, white_frame = detect_color(frame, lower_white, upper_white, white_color, draw_rectangle)

    best_area = 0
    best_target = None
    best_mask = None
    best_color_name = None
    best_color = None

    for white_target in white_targets:
        x, y, w, h = white_target
        white_area_frame = frame[y:y+h, x:x+w]

        # Przeszukiwanie wszystkich zestawów wartości HSV w białych obszarach
        for color_name, hsv in hsv_values.items():
            if color_name == "Biala plachta":
                continue  # Pomijanie detekcji białych obszarów ponownie

            lower_color = hsv["lower"]
            upper_color = hsv["upper"]
            color = hsv.get("color", (0, 0, 0))  # Default to black if color is not specified
            targets, masked_frame = detect_color(white_area_frame, lower_color, upper_color, color, draw_rectangle)

            for target in targets:
                tx, ty, tw, th = target
                area = tw * th
                if area > best_area:
                    best_area = area
                    best_target = (x + tx, y + ty, tw, th)
                    best_mask = masked_frame
                    best_color_name = color_name
                    best_color = color

    # Wyświetlanie zmniejszonego obrazu z zaznaczonymi obszarami
    cv2.imshow('Processed Frame', best)
    # Wyświetlanie zmniejszonego obrazu z zaznaczonymi obszarami
    cv2.imshow('Processed Frame', best_mask)

    # Zapisanie przetworzonego obrazu
    output_path = image_path.replace('.jpg', '_processed.jpg')  # Dodajemy '_processed' do nazwy pliku
    cv2.imwrite(output_path, best_mask)

    return output_path


