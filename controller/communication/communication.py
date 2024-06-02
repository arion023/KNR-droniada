import serial

Konfiguracja portu szeregowego
ser = serial.Serial(

    port='/dev/ttyAMA0',
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1,
)

try:
    while True:
        if ser.in_waiting > 0:
            message = ser.readline().decode('utf-8').rstrip()
            print(f'Odebrano: {message}')
except KeyboardInterrupt:
    print("Przerwano przez użytkownika")
finally:
    ser.close()
