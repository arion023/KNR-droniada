from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

# Połącz się z dronem (zmień 'udp:127.0.0.1:14550' na odpowiedni adres i port)
vehicle = connect('udp:127.0.0.1:14550', wait_ready=True)

# Ustaw prędkość poziomą w metrach na sekundę
def set_velocity_body(vehicle, vx, vy, vz):
    # Stwórz wiadomość komendy ruchu (COMMAND_LONG)
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # Wysłane do kontrolera
        mavutil.mavlink.MAV_CMD_DO_SET_VELOCITY_BODY,  # Numer komendy
        0,  # konfirmacja
        0,  # Zerowanie parametrów ustawionych na 0
        0, 0, 0,  # kolejne parametry ustawione na 0
        vx, vy, vz)  # Prędkości w osiach x, y, z
    # Wyślij wiadomość
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Przykładowe użycie: Ustawienie prędkości do przodu 1 m/s (reszta na 0)
set_velocity_body(vehicle, 1, 0, 0)

# Czekaj 5 sekund
time.sleep(5)

# Zatrzymaj drona
set_velocity_body(vehicle, 0, 0, 0)

# Zakończ połączenie z dronem
vehicle.close()
