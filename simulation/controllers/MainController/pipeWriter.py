import sys, os, time
import pickle, struct

WRITE_FIFO_PATH = "./pipes/main_to_controller"
READ_FIFO_PATH = "./pipes/controller_to_main"

def generate_velocites():
    
    velocities = [[0., 0., 0., 2.], [0., 0., 0.5, 0.], [0., 0.3, 0., 0.] ]

    with open(WRITE_FIFO_PATH, 'w') as fifo:
        # Set the file descriptor to non-blocking mode
        i = 0
        while True:
            velo = velocities[i%len(velocities)]
            i+=1
            packed_struct = struct.pack('ffff', *velo)
            print(f'Velocity: {velo}')
            print(f'Packed struct: {packed_struct}')
            fifo.write((str(packed_struct)))
            fifo.flush()
            time.sleep(10)


def generate_destinations():
    
    destinations = [[0., 0., 0.], [0., 0., 2.], [2., 2., 2.], [0., 0., 2.] ]
    
    read_fifo = open(READ_FIFO_PATH, 'r')

    with open(WRITE_FIFO_PATH, 'w') as fifo:
        # Set the file descriptor to non-blocking mode
        i = 0
        while True:
            dest = destinations[i%len(destinations)]
            i+=1
            packed_struct = struct.pack('fff', *dest)
            print(f'Destination: {dest}')
            print(f'Packed struct: {packed_struct}')
            fifo.write((str(packed_struct)))
            fifo.flush()
            response = read_fifo.readline()
            print("RESPONSE: ", response)
            

    read_fifo.close()

if __name__ == "__main__":
    if not os.path.exists(READ_FIFO_PATH):
        os.mkfifo(READ_FIFO_PATH)
    if not os.path.exists(WRITE_FIFO_PATH):
        os.mkfifo(WRITE_FIFO_PATH)
    #generate_velocities()
    generate_destinations()


