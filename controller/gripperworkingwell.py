from gpiozero import PWMOutputDevice, DigitalOutputDevice

# Zadeklarowanie pinów dla mostka H
pinForward = 27  # Pin do przodu (GPIO 17)
pinBackward = 17 # Pin do tylu (GPIO 27)
pinPWM = 22      # Pin PWM (GPIO 22)

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
motor_forward(0.4)  # Jedź do przodu z połową prędkości
import time
time.sleep(2)  # Czekaj 2 sekundy
motor_backward(0.4)  # Jedź do tyłu z mniejszą prędkością
time.sleep(2)  # Czekaj 2 sekundy
motor_stop()  # Zatrzymaj silnik
