from controller import Robot, Keyboard

'''
Controller onyl for camera
'''


# Inicjalizacja robota
robot = Robot()

# Pobierz dostęp do klawiatury
keyboard = Keyboard()
keyboard.enable(64)
camera=robot.getDevice('camera')
camera.enable(64)

# Pobierz dostęp do kamery i stawów obrotowych kamery
camera_yaw = robot.getDevice('camera yaw')
camera_pitch = robot.getDevice('camera pitch')
camera_roll = robot.getDevice('camera roll')
yaw_sensor = robot.getDevice('camera yaw sensor')
pitch_sensor = robot.getDevice('camera pitch sensor')
roll_sensor = robot.getDevice('camera roll sensor')

# Ustaw prędkości obrotowe dla poszczególnych stawów
max_speed = 1.0
yaw_sensor.enable(64)
pitch_sensor.enable(64)
roll_sensor.enable(64)


while robot.step(64) != -1:
    # Obsługa klawiatury
    key = keyboard.getKey()
    if key == Keyboard.LEFT:
        camera_yaw.setPosition(yaw_sensor.getValue() + 0.1)
    elif key == Keyboard.RIGHT:
        camera_yaw.setPosition(yaw_sensor.getValue() - 0.1)
    elif key == Keyboard.UP:
        camera_pitch.setPosition(pitch_sensor.getValue() + 0.1)
    elif key == Keyboard.DOWN:
        camera_pitch.setPosition(pitch_sensor.getValue() - 0.1)
    elif key == ord('A'):
        camera_roll.setPosition(roll_sensor.getValue() + 0.1)
    elif key == ord('D'):
        camera_roll.setPosition(roll_sensor.getValue() - 0.1)
