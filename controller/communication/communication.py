import time
import serial

ser = serial.Serial(
    port='/dev/tty0',
    baudrate=57600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

if not ser.isOpen():
    ser.open()

ser.flushInput()
ser.flushOutput()

counter = 0

while True:
    # Odbieranie danych z STM32
    if ser.in_waiting > 0:
       data = ser.read_until(b'@')
       print(f"Odebrano z STM32: {data}")

    # Wysyłanie danych do STM32
    # counter += 1
    # message = f"counter: {counter}#"
    # message1 = 'flt4'
    # message2 = 'flt0'
    # ser.write(message1.encode('utf-8'))
    # print(f"Wysłano do STM32: {message1.strip('#')}")
    # time.sleep(4)
    # ser.write(message2.encode('utf-8'))
    # print(f"Wysłano do STM32: {message1.strip('#')}")
    # #ser.write(message2.encode('utf-8'))
    # #print(f"Wysłano do STM32: {message2.strip('#')}")
    time.sleep(2)
# try:
#     while True:
#         if ser.in_waiting > 0:
#             message = ser.readline().decode('utf-8').rstrip()
#             print(f'Odebrano: {message}')
# except KeyboardInterrupt:
#     print("Przerwano przez użytkownika")
# finally:
#     ser.close()