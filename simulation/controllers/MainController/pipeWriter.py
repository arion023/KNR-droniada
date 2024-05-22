import sys, os, time
import pickle

FIFO_PATH = "./main_to_controller"

def generate_comands():
    with open(FIFO_PATH, 'w') as fifo:
        # Set the file descriptor to non-blocking mode
        while True:
            velo = [0., 0., 0., 2.]
            fifo.write(pickle.dumps(velo))
            fifo.flush()
            time.sleep(10)


if __name__ == "__main__":
    if not os.path.exists(FIFO_PATH):
        os.mkfifo(FIFO_PATH)
    generate_comands()


