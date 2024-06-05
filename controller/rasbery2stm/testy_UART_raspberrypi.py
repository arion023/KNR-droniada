import serial
import time
import struct

# Configure the serial port
ser = serial.Serial(
    port='/dev/serial0',  # or '/dev/ttyAMA0'
    baudrate=57600,
    timeout=1
)

# Ensure the serial port is open
if ser.is_open:
    print("Serial port is open")

# Send data to STM32
def send_data(data):
    ser.write(data.encode())

# Receive data from STM32
def receive_data():
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting).decode()
        return data
    return None

def send_float(ser, value):
    # Pack the float into 4 bytes using struct
    packed_data = struct.pack('f', value)
    ser.write(packed_data)

# Example usage
# send_data("Hello STM32")
send_float(ser, 3.123)
time.sleep(1)
# response = receive_data()
# if response:
#     print(f"Received from STM32: {response}")

# Close the serial port
ser.close()
