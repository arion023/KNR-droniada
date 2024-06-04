import RPi.GPIO as GPIO
from time import sleep

class Gripper:
    pin1 = 27
    pin2 = 17
    pin_pwm = 12
    
    def __init__(self):
        self.gripped = False
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin1, GPIO.OUT)
        GPIO.setup(self.pin2, GPIO.OUT)
        GPIO.setup(self.pin_pwm, GPIO.OUT)

        # Konfiguracja PWM
        self.pwm = GPIO.PWM(self.pin_pwm, 1000)  # Ustawienie częstotliwości PWM na 1 kHz
        self.pwm.start(60)  # Uruchomienie PWM z wypełnieniem 100% (wartość 1)
    
    def open_grip(self):
        GPIO.output(self.pin1, GPIO.HIGH)
        GPIO.output(self.pin2, GPIO.LOW) 
        self.gripped = True
        print('elo')
        sleep(1)  # Obrót silnika o pół obrotu w jedną stronę
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.LOW)

    def close_grip(self):
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.HIGH)
        self.gripped = False
        print('elo1')
        sleep(1)  # Obrót silnika o pół obrotu w drugą stronę
        GPIO.output(self.pin1, GPIO.LOW)
        GPIO.output(self.pin2, GPIO.LOW)
    
    def test(self):
        while True:
            self.open_grip()
            sleep(1)  # Poczekaj chwilę
            self.close_grip()
            sleep(1)  # Poczekaj chwilę

    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup()

# Inicjalizacja obiektu klasy Gripper
gripper = Gripper()

try:
    # Wywołanie testu
    gripper.test()
except KeyboardInterrupt:
    # Czyszczenie pinów GPIO po zakończeniu programu
    gripper.cleanup()
