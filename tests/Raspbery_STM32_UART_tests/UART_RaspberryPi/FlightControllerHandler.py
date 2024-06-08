from queue import Queue
from threading import Thread
from enum import Enum
import struct
import serial
import time
import logging

# temporary
import os

SERIAL_MOCK_WRITER = './mock/serial_mock_writer.txt'
SERIAL_MOCK_READER = './mock/serial_mock_read.txt'


class CommunicationStatus(Enum):
    READY = 0
    IN_PROGRESS = 1


class MockSerial:
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


class FlightControllerInterface:
    def __init__(self, mock=False, mock_responses=[]):
        self.set_up_logger()
        self.logger.info("Controller Interface initialization...")
        self.command_que = Queue()
        self.handler_thread = None

        self.controller_handler = FlightControllerHandler(self.command_que, mock)
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
        if self.__validate_values(values):
            self.command_que.put_nowait((cmd, values))
            self.logger.info(f'Command sent to handler. {cmd} {values}')

    def move(self, x_vel, y_vel, z_vel):
        cmd = 'VEL'
        values = [x_vel, y_vel, z_vel]
        if self.__validate_values(values):
            self.command_que.put_nowait((cmd, values))
            self.logger.info(f'Command sent to handler. {cmd} {values}')

    def land(self):
        cmd = 'LND'
        self.command_que.put_nowait((cmd, []))
        self.logger.info(f'Command sent to handler. {cmd}')

    def start(self):
        cmd = 'STR'
        self.command_que.put_nowait((cmd, []))
        self.logger.info(f'Command sent to handler. {cmd}')

    def rotate(self, angle):
        cmd = 'ROT'
        values = [angle]
        if self.__validate_values(values):
            self.command_que.put_nowait((cmd, values))
            self.logger.info(f'Command sent to handler. {cmd} {values}')

    def terminate_handler(self):
        cmd = 'END'
        self.command_que.put_nowait((cmd, []))
        self.logger.info(f'Command sent to handler. {cmd}')

    def __validate_values(self, values):
        for f in values:
            if not isinstance(f, float):
                self.logger.info('Error: input values must be floats')
                return False
        return True

    def __del__(self):
        if self.handler_thread and self.handler_thread.is_alive():
            self.handler_thread.join()


class FlightControllerHandler:
    def __init__(self, command_que, mock=False):
        self.set_up_logger()
        self.logger.info('Handler initialization.')
        self.mock = mock
        self.command_que = command_que
        self.serial_bus = MockSerial() if mock else serial.Serial(
            port='/dev/ttyS0', baudrate=57600, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1
        )
        if not self.serial_bus.isOpen():
            self.serial_bus.open()

    def load_mock_responses(self, responses):
        if self.mock:
            self.serial_bus.load_mock_responses(responses)
        else:
            self.logger.error("Controller is set as non-mock")

    def set_up_logger(self):
        self.logger = logging.getLogger("FlightControllerHandlerLogger")
        self.logger.setLevel(logging.INFO)
        file_handler = logging.FileHandler('./log/FlightControllerHandler.log')
        formatter = logging.Formatter('%(asctime)s - %(message)s', datefmt='%Y-%m-%d %H:%M:%S')
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)

    def __pack(self, cmd, values):
        checksum = int(0)
        cmd_bytes = cmd.encode('utf-8')
        endcommand = '@'.encode('utf-8')
        return struct.pack('>3s3fIb', cmd_bytes, *values, checksum, endcommand[0])

    def __unpack(self, data):
        response_bytes = struct.unpack('>3s', data[:3])
        return response_bytes.decode('utf-8')

    def __send_command(self, cmd, values):
        self.logger.info(f'Sending command: {cmd}, {values}')
        byte_frame = self.__pack(cmd, values)
        self.serial_bus.write(byte_frame)
        self.serial_bus.flush()

    def __get_response(self, timeout=-1):
        self.logger.info('Waiting for response...')
        endcommand = '@'.encode('utf-8')
        response_frame = self.serial_bus.read_until(endcommand)
        waiting_step = 0.1
        while not response_frame and (timeout > 0 or timeout == -1):
            time.sleep(waiting_step)
            timeout -= waiting_step
            response_frame = self.serial_bus.read_until(endcommand)
        if response_frame:
            response = self.__unpack(response_frame)
            self.logger.info(f'Received response: {response}')
            return response
        else:
            self.logger.info("Didn't receive response.")
            return None

    def __get_command(self):
        self.logger.info('Waiting for command from queue...')
        command = self.command_que.get(block=True)
        self.logger.info(f'Command received from queue: {command}')
        return self.__command_creator(command)

    def __command_creator(self, command):
        cmd = command[0]
        values = [.0, .0, .0]
        if cmd in ['DST', 'VEL']:
            values = command[1]
        elif cmd == 'ROT':
            values[0] = command[1][0]
        return cmd, values

    def __handle_response(self, cmd):
        response = self.__get_response(timeout=1)
        if response == 'ACK':
            if cmd != 'DST':
                return True
            else:
                response = self.__get_response()  # blocking
                if response == 'FIN':
                    return True
        return False

    def run(self):
        status = False
        while True:
            cmd, values = self.__get_command()
            if cmd == 'END':
                break
            while not status:
                self.__send_command(cmd, values)
                status = self.__handle_response(cmd)
            status = False


# Example usage
if __name__ == "__main__":
    mock_responses = ['ACK@', 'ACK@', 'ACK@', 'ACK@', 'ACK@']
    interface = FlightControllerInterface(mock=False, mock_responses=mock_responses)
    interface.run_handler()

    interface.goto_point(1.0, 2.0, 3.0)
    interface.move(0.5, 0.5, 0.5)
    interface.rotate(90.0)
    interface.start()
    interface.land()
    interface.terminate_handler()
