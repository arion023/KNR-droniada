from gpiozero import PWMOutputDevice, DigitalOutputDevice
import time
# Zadeklarowanie pinów dla mostka H
pinForward = 17  # Pin do przodu (GPIO 17)
pinBackward = 27 # Pin do tylu (GPIO 27)
pinPWM = 22      # Pin PWM (GPIO 12)
wartoscPWM = 0.2

# Ustawienie pinów
forward = DigitalOutputDevice(pinForward)
backward = DigitalOutputDevice(pinBackward)
speed = PWMOutputDevice(pinPWM)

def motor_forward(speed_value):
    """Jedź do przodu z określoną prędkością."""
    backward.off()
    forward.on()
    speed.value = speed_value  # Ustaw prędkość (0.0 do 1.0)

def motor_backward(speed_value):
    """Jedź do tyłu z określoną prędkością."""
    forward.off()
    backward.on()
    speed.value = speed_value  # Ustaw prędkość (0.0 do 1.0)

def motor_stop():
    """Zatrzymaj silnik."""
    forward.off()
    backward.off()
    speed.value = 0

# Przykładowe użycie funkcji
motor_forward(wartoscPWM)  # Jedź do przodu z połową prędkości
time.sleep(2)  # Czekaj 2 sekundy
motor_backward(wartoscPWM)  # Jedź do tyłu z mniejszą prędkością
time.sleep(2)  # Czekaj 2 sekundy
motor_stop()  # Zatrzymaj silnik
