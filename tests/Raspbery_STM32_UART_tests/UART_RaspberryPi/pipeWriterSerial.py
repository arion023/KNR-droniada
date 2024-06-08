import sys, os, time
import struct

WRITE_FIFO_PATH = "./pipes/main_to_controller"
READ_FIFO_PATH = "./pipes/controller_to_main"

def generate_velocites():

    read_fifo = open(READ_FIFO_PATH, 'r')

    velocities = [[0., 0., 0., 2.], [0., 0., 0.5, 0.], [0., 0.3, 0., 0.] ]

    with open(WRITE_FIFO_PATH, 'wb') as fifo:
        # Set the file descriptor to non-blocking mode
        i = 0
        while True:
            velo = velocities[i%len(velocities)]
            i+=1
            packed_struct = struct.pack('>ffff', velo[0], velo[1], velo[2], velo[3])
            print(f'Velocity: {velo}')
            print(f'Packed struct: {packed_struct}')
            fifo.write(packed_struct)
            fifo.flush()
            response = read_fifo.readline()
            print("RESPONSE: ", response)
            time.sleep(10)


def generate_destinations():

    # destinations_old = [[0., 0., 0.], [0., 0., 2.], [2., 2., 2.], [0., 0., 2.] ]
    destinations_kosz = [[7.5, 27, 2]]

    positions_balls = [
    #od górnego lewego rogu(najdalej od drona), potem kolejne w prawo, potem kolejne rzędy w dół
    [11.5, 11.5, 2.],
    [11.5, 7.5, 2.],
    [11.5, 3.5, 2.],

    [7.5, 11.5, 2.],
    [7.5, 7.5, 2.],
    [7.5, 3.5, 2.],

    [3.5, 11.5, 2.],
    [3.5, 7.5, 2.],
    [3.5, 3.5, 2.],
    ]


    # full misja
    destinations = [
    [0., 0., 0.],  #start
    [0., 0., 2],  # góra 2
    # debug
    # [0., 2., 2],  # lewo 2m patrzać tak że platforma jest w dolnym prawym rogu
    # [2., 0., 2],  # do przodu 2m

    [7.5, -3.5, 3.8], #kamera, lecimy na środek, wysoko i walimy fote
    [7.5, 0, 3.8],  # w przelocie do tego punktu


    [3.5, 3.5, 2.],  #pierwsza

    # BLUE
    [11.5, 11.5, 2.],  # BLUE
    [11.5, 11.5, 0.2],  # lądowanie
    [11.5, 11.5, 2],  # do góry
    #blue -> kosz
    [7.5, 27, 2],  # kosz
    [7.5, 27, 1.5],  # zawis nad koszem i zrzut
    [7.5, 27, 2],  # do góry


    #RED
    [11.5, 3.5, 2.],  # Red
    [11.5, 3.5, .2],  # lądowanie
    [11.5, 3.5, 2],  # do góry
    #red -> kosz
    [7.5, 27, 2],  # kosz
    [7.5, 27, 1.5],  # zawis nad koszem i zrzut
    [7.5, 27, 2],  # do góry


    #Purple
    [3.5, 7.5, 2.],  # Purple/black
    [3.5, 7.5, .2],  # lądowanie
    [3.5, 7.5, 2],  # do góry
    #Purple -> kosz
    [7.5, 27, 2],  # kosz
    [7.5, 27, 1.5],  # zawis nad koszem i zrzut
    [7.5, 27, 2],  # do góry


    [0, 0, 2],    #powrót
    [0, 0, 0]]  #lądowanie




    read_fifo = open(READ_FIFO_PATH, 'r')

    with open(WRITE_FIFO_PATH, 'wb') as fifo:
        # Set the file descriptor to non-blocking mode
        i = 0
        while True:
            dest = destinations[i%len(destinations)]
            i+=1
            packed_struct = struct.pack('fff', *dest)
            print(f'Destination: {dest}')
            print(f'Packed struct: {packed_struct}')
            fifo.write(packed_struct)
            fifo.flush()
            #waiting for OK
            response = read_fifo.readline()
            print("RESPONSE: ", response)

            #waiting for REACHED
            response = read_fifo.readline()
            print("RESPONSE: ", response)


    read_fifo.close()

if __name__ == "__main__":
    if not os.path.exists(READ_FIFO_PATH):
        os.mkfifo(READ_FIFO_PATH)
    if not os.path.exists(WRITE_FIFO_PATH):
        os.mkfifo(WRITE_FIFO_PATH)
    # generate_velocites()
    generate_destinations()


