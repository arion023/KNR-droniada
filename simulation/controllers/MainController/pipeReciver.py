import os
import fcntl
import time
import pickle, struct

FIFO_PATH = "./pipes/main_to_controller"
WRITE_FIFO_PATH = "./pipes/controller_to_main"

def read_velocities_from_fifo_non_blocking():
    with open(FIFO_PATH, 'r', buffering=1) as fifo:
        # Set the file descriptor to non-blocking mode
        fd = fifo.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        while True:
            try:
                data = fifo.readline()
                if data:
                    unpacked = struct.unpack('ffff', eval(data))
                    print(f'Received: { unpacked }')
                else:
                    # No data available at the moment
                    print('Pipe is empty.\nWaiting...')
                    time.sleep(1)  # Sleep briefly before trying again
            except BlockingIOError:
                # No data was available for reading
                time.sleep(1)

def read_destinations_from_fifo_non_blocking():
    write_fifo = open(WRITE_FIFO_PATH, 'w')
    with open(FIFO_PATH, 'r', buffering=1) as fifo:
        # Set the file descriptor to non-blocking mode
        fd = fifo.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        while True:
            try:
                data = fifo.readline()
                if data:
                    unpacked = struct.unpack('fff', eval(data))
                    print(f'Received: { unpacked }')
                    time.sleep(2)
                    write_fifo.write("RECIVED\n")
                    write_fifo.flush()
                else:
                    # No data available at the moment
                    print('Pipe is empty.\nWaiting...')
                    time.sleep(1)  # Sleep briefly before trying again
            except BlockingIOError:
                # No data was available for reading
                time.sleep(1)

if __name__ == '__main__':
    if not os.path.exists(FIFO_PATH):
        os.mkfifo(FIFO_PATH)
    #read_velocities_from_fifo_non_blocking()
    read_destinations_from_fifo_non_blocking()