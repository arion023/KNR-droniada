import os
import fcntl
import time
import pickle, struct

FIFO_PATH = "./main_to_controller"

def read_from_fifo_non_blocking():
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

if __name__ == '__main__':
    if not os.path.exists(FIFO_PATH):
        os.mkfifo(FIFO_PATH)
    read_from_fifo_non_blocking()