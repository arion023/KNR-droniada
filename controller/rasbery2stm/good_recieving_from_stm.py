import serial
import time


# definiujemy mnożnik przez który jest mnożony float potem wysyłany i odbierany 
MULTIPLYER = 2**30
# Konfiguracja połączenia szeregowego
ser = serial.Serial(
    port='/dev/ttyS0',    # Nazwa portu szeregowego
    baudrate=57600,       # Prędkość transmisji
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1             # Czas oczekiwania na dane
)

print("Odbieranie danych z stm...")

start_time = time.time()  # Zapisz czas rozpoczęcia

try:
    while time.time() - start_time < 10:  # Pętla działa przez 10 sekund
        if ser.in_waiting > 0:
            try:
                data = ser.readline().decode('utf-8', errors='ignore').rstrip()
                print(f"raw dane z stm32: {data}")
                #odbieramy stuff np float : ``
                string_uart, int_uart = data.split(":")
                int_uart = int_uart[1:]
                print("string", string_uart)
                print("int",int_uart)
                resoult = float(int_uart)/MULTIPLYER
                print(resoult)
                # resoult_of_division = int(data)/multiplyer
                # print(f"Dane z akcelerometru: {resoult_of_division}")
            except UnicodeDecodeError as e:
                print(f"Error decoding data: {e}")
except KeyboardInterrupt:
    print("Zakończono odbieranie danych.")
finally:
    ser.close()
    print("Połączenie szeregowe zamknięte.")
