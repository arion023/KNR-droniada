import sys, os, time
import pickle, struct

FIFO_PATH = "./pipes/main_to_controller"

def generate_comands():
    with open(FIFO_PATH, 'w') as fifo:
        # Set the file descriptor to non-blocking mode
        while True:
            velo = [0., 0., 0., 2.]
            packed_struct = struct.pack('ffff', *velo)
            print(f'Packed stucture: {packed_struct}')
            fifo.write((str(packed_struct)))
            fifo.flush()
            time.sleep(10)


if __name__ == "__main__":
    if not os.path.exists(FIFO_PATH):
        os.mkfifo(FIFO_PATH)
    generate_comands()


