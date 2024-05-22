import os
import fcntl
import time
import pickle

FIFO_PATH = "./main_to_controller"

def read_from_fifo_non_blocking():
    with open(FIFO_PATH, 'r') as fifo:
        # Set the file descriptor to non-blocking mode
        fd = fifo.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

        while True:
            try:
                data = fifo.read()
                if data:
                    print(f'Received: {pickle.loads(data)}')
                else:
                    # No data available at the moment
                    print('Pipe is empty.\nWaiting...')
                    time.sleep(1)  # Sleep briefly before trying again
            except BlockingIOError:
                # No data was available for reading
                time.sleep(1)

if __name__ == '__main__':
    read_from_fifo_non_blocking()