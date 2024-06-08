import serial
import time

# Configuration for the serial port
ser = serial.Serial(
    port='/dev/serial0',  # or '/dev/ttyS0' depending on your setup
    baudrate=9600,
    timeout=1
)
MULTIPLYER = 2**20

def send_integer_to_stm32(three_chars,int1,int2,int3,checksum):
    # Multiply the float by the multiplier and convert to integer
    int_value_1 = int(int1 * MULTIPLYER)
    int_value_2 = int(int2 * MULTIPLYER)
    int_value_3 = int(int3 * MULTIPLYER)

    
    # Convert the integer to bytes
    data1 = int_value_1.to_bytes(4, byteorder='big', signed=True)
    data2 = int_value_2.to_bytes(4, byteorder='big', signed=True)
    data3 = int_value_3.to_bytes(4, byteorder='big', signed=True)
    
    checksum = checksum.to_bytes(4, byteorder='big', signed=True)
    
    three_chars = three_chars.encode('utf-8')

    # Send the byte data to STM32
    data = three_chars + data1 + data2 + data3 + checksum
    
    data += b'@'

    ser.write(data)
    ser.flush()

    print(f"Sent: {three_chars, int_value_1, int_value_2, int_value_3, checksum} as bytes: {data}")


if __name__ == "__main__":
    try:
        while True:
            # Example float value to send
            three_chars = "LND"
            float_value1 = 3.4567
            float_value2 = 9.1011
            float_value3 = 12.1314
            fake_checksum = 123
            send_integer_to_stm32(three_chars,float_value1,float_value2, float_value3,fake_checksum)
            time.sleep(1)  # Send data every second

    except KeyboardInterrupt:
        print("Stopped by User")
    finally:
        ser.close()
