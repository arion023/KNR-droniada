from queue import Queue
import queue
from threading import Thread
from enum import Enum
import struct
import serial
import time
import logging

#temporary
import os

SERIAL_MOCK_WRITER = './mock/serial_mock_writer.txt'
SERIAL_MOCK_READER = './mock/serial_mock_read.txt'

class CommunicationStatus(Enum):
    READY = 0
    IN_PROGRESS = 1


class MockSerial():
    def __init__(self, *values):
        self.mock_path = SERIAL_MOCK_WRITER
        self.mock_fp = None
        self.is_open = False

        self.response_que = Queue()


    def load_mock_responses(self, responses):
        for r in responses:
            self.response_que.put(r)

    def write(self, data):
        self.mock_fp.write(str(data) + '\n')

    def read_until(self, expected):
        response = self.response_que.get()
        return response.encode(encoding='utf-8')

    def open(self):
        self.mock_fp = open(self.mock_path, 'w')
        self.is_open = True

    def isOpen(self):
        return self.is_open

    def flush(self):
        pass


class FlightControllerInterface():
    def __init__(self, mock=False, mock_responses=[]):

        self.set_up_logger()
        self.logger.info("Controller Interface initizalization...")

        self.command_que = Queue()

        self.controller_handler= FlightControllerHandler(self.command_que, mock)
        if mock:
            self.controller_handler.load_mock_responses(mock_responses)


    def run_handler(self):
        self.handler_thread = Thread(target=self.controller_handler.run, args=())
        self.handler_thread.start()

    def get_controller_handler(self):
        return self.controller_handler

    def set_up_logger(self):
        self.logger = logging.getLogger("FlightControllerInterfaceLogger")
        self.logger.setLevel(logging.INFO)
        file_handler = logging.FileHandler('./log/FlightControllerInterface.log')
        formatter = logging.Formatter('%(asctime)s - %(message)s', datefmt='%H:%M:%S')
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

    def goto_point(self, x, y, z):
        cmd = 'DST'
        values = [x, y, z]

        if not self.__validate_values(values):
            return

        self.command_que.put_nowait((cmd, values))
        self.logger.info(f'Command send to handler. {cmd} {values}')

    def move(self, x_vel, y_vel, z_vel):
        cmd = 'VEL'

        values = [x_vel, y_vel, z_vel]

        if not self.__validate_values(values):
            return

        self.command_que.put_nowait((cmd, values))
        self.logger.info(f'Command send to handler. {cmd} {values}')


    def land(self):
        cmd = 'LND'

        self.command_que.put_nowait((cmd, []))
        self.logger.info(f'Command send to handler. {cmd}')


    def start(self):
        cmd = 'STR'

        self.command_que.put_nowait((cmd, []))
        self.logger.info(f'Command send to handler. {cmd}')


    def rotate(self, angle):
        cmd = 'ROT'
        values = [angle]

        if not self.__validate_values(values):
            return

        self.command_que.put_nowait((cmd, values))
        self.logger.info(f'Command send to handler. {cmd} {values}')


    def terminate_handler(self):
        cmd = 'END'

        self.command_que.put_nowait((cmd, []))
        self.logger.info(f'Command send to handler. {cmd}')


    def __validate_values(self, values):
        for f in values:
            if not isinstance(f, float):
                self.logger.info('Error: input values must be floats')
                return False
        return True

    def __del__(self):
        self.handler_thread.join()


class FlightControllerHandler:
    def __init__(self, command_que, mock=False):

        self.set_up_logger()
        self.logger.info('Handler initialization.')

        self.mock = mock
        self.command_que = command_que
        if mock:
            self.serial_bus = MockSerial()
        else:
            self.serial_bus = serial.Serial(port='/dev/ttyS0',
                                        baudrate=57600,
                                        parity=serial.PARITY_NONE,
                                        stopbits=serial.STOPBITS_ONE,
                                        bytesize=serial.EIGHTBITS,
                                        timeout=1)
        if not self.serial_bus.isOpen():
            self.serial_bus.open()

    def load_mock_responses(self, responses):
        if self.mock:
            self.serial_bus.load_mock_responses(responses)
        else:
            print("Error: controller is set as non-mock")

    def set_up_logger(self):
        self.logger = logging.getLogger("FlightControllerHandlerLogger")
        self.logger.setLevel(logging.INFO)
        file_handler = logging.FileHandler('./log/FlightControllerHandler.log')
        formatter = logging.Formatter('%(asctime)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

    def __pack(self, cmd, values):
        checksum=int(0)
        cmd_bytes = cmd.encode('utf-8')
        endcommand = '@'
        endcommand = endcommand.encode('utf-8')
        #command formats = @ CMD XYZ CHECKSUM
        return struct.pack('>bbbfffib', *cmd_bytes, *values, checksum, *endcommand)

    def __unpack(self, data):
        #command formats = CMD @
        response_bytes = struct.unpack('>bbbb', data)
        response = bytearray()
        response.append(response_bytes[0])
        response.append(response_bytes[1])
        response.append(response_bytes[2])
        return response.decode('utf-8')

    def __send_command(self, cmd, values):
        self.logger.info(f'Sending command: {cmd}, {values}')
        byte_frame = self.__pack(cmd, values)

        self.serial_bus.write(byte_frame)
        self.serial_bus.flush()

    def __get_response(self, timeout=-1):
        self.logger.info(f'Waiting for response...')
        response_frame = self.serial_bus.read_until('@'.encode('utf-8'))
        waiting_step = 0.1
        while(not response_frame and (timeout>0 or timeout==-1)):
            time.sleep(waiting_step)
            timeout-=waiting_step
            response_frame = self.serial_bus.read_until('@'.encode('utf-8'))
        if response_frame:
            response = self.__unpack(response_frame)
            self.logger.info(f'Recived response: {response}')
            return response
        else:
            self.logger.info(f'Doesn\'t recived response.')
            return None

    def __get_command(self):
        self.logger.info('Waiting for command from que...')
        command = self.command_que.get(block=True)
        self.logger.info(f'Command recived from que: {command}')
        return self.__command_creator(command)


    def __command_creator(self, command):
        cmd = command[0]
        values = [.0, .0, .0]
        if cmd == 'DST' or cmd == 'VEL':
            values = command[1]
        elif cmd == 'ROT':
            values[0] = command[1][0]

        return cmd, values

    def __handle_response(self, cmd):
        response = self.__get_response(timeout=1)
        if response=='ACK':
            if cmd != 'DST':
                return True
            else:
                response = self.__get_response() #blocking
                if response == 'FIN':
                    return True
        else:
            return False


    def run(self):
        status = False
        cmd=''
        values=[]
        while(True):
            cmd, values = self.__get_command()
            if cmd == 'END':
                break
            while(status==False):
                self.__send_command(cmd, values)
                status = self.__handle_response(cmd)
            status = False


# if __name__ == "__main__":

#     #example of how to use code, output will be logged in log directory
#     #run this code from communication directory
#     #if you have doesn't configured serial bus in code, use mock=True, and create mock directory, this will write to file in this directory
#     mock_responses = ['ACK@', 'ACK@', 'ACK@', 'ACK@', 'ACK@']

#     interface = FlightControllerInterface(mock=False, mock_responses=mock_responses)
#     interface.run_handler()

#     # interface.move(2., 2., 3.) #63 bytes
#     # interface.start() #60
#     interface.rotate(2) #0
    

#     # interface.move(3., 2., 3.)
#     # interface.move(4., 2., 3.)
#     # interface.move(5., 2., 3.)
#     # interface.move(6., 2., 3.)
    
#     # interface.goto_point(1,1,1)
#     for i in range (100):
#         # cmd = 'AAA'

#         # interface.command_que.put_nowait((cmd, []))


#         cmd = 'AAAAA'

#         interface.command_que.put_nowait((cmd, []))
            
#         cmd = 'AAAAAAAAAA'

#         interface.command_que.put_nowait((cmd, []))

#     # interface.terminate_handler()

import serial
import serial

def send_bytes_to_stm32(serial_port, num_bytes):
    """
    Sends a specified number of bytes to the STM32 microcontroller, 
    ending with '@' as the terminating character.

    Parameters:
    serial_port (serial.Serial): The serial port connected to the STM32.
    num_bytes (int): The number of bytes to send, including the '@' terminator.
    """
    if not serial_port.isOpen():
        raise Exception("Serial port is not open")

    # Example byte data to send (for illustration purposes)
    data = bytearray([i % 256 for i in range(num_bytes - 1)])
    
    # Append '@' as the terminating character
    data.append(ord('@'))

    # Send the byte data to STM32
    serial_port.write(data)
    serial_port.flush()

    print(f"Sent {num_bytes} bytes to STM32, ending with '@'")

# Example usage
if __name__ == "__main__":
    # Open the serial port
    try:
        serial_port = serial.Serial(port='/dev/ttyS0', baudrate=57600, timeout=1)
        send_bytes_to_stm32(serial_port, 10)  # Sends 9 bytes of data plus '@' as the 10th byte
        send_bytes_to_stm32(serial_port, 20)  # Sends 9 bytes of data plus '@' as the 10th byte
        send_bytes_to_stm32(serial_port, 30)  # Sends 9 bytes of data plus '@' as the 10th byte
        send_bytes_to_stm32(serial_port, 100)  # Sends 9 bytes of data plus '@' as the 10th byte
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    finally:
        if serial_port.isOpen():
            serial_port.close()

