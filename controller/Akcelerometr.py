import serial
import time

# Konfiguracja połączenia szeregowego
ser = serial.Serial(
    port='/dev/ttyS0',    # Nazwa portu szeregowego
    baudrate=57600,       # Prędkość transmisji
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1             # Czas oczekiwania na dane
)

print("Odbieranie danych z akcelerometru...")

start_time = time.time()  # Zapisz czas rozpoczęcia

try:
    while time.time() - start_time < 10:  # Pętla działa przez 10 sekund
        if ser.in_waiting > 0:
            try:
                data = ser.readline().decode('utf-8', errors='ignore').rstrip()
                print(f"Dane z akcelerometru: {data}")
            except UnicodeDecodeError as e:
                print(f"Error decoding data: {e}")
except KeyboardInterrupt:
    print("Zakończono odbieranie danych.")
finally:
    ser.close()
    print("Połączenie szeregowe zamknięte.")
