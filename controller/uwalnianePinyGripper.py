import lgpio
import time
from gpiozero import PWMOutputDevice, DigitalOutputDevice

def claim_and_release_pins(pins):
    # Open GPIO chip
    h = lgpio.gpiochip_open(0)

    # Claim and release the pins
    for pin in pins:
        try:
            lgpio.gpio_claim_output(h, 0, pin, 0)
            lgpio.gpio_free(h, pin)
        except Exception as e:
            print(f"Error releasing pin {pin}: {e}")

    # Close GPIO chip
    lgpio.gpiochip_close(h)

# List of GPIO pins to claim and release
pins_to_reset = [23, 24, 12]  # Replace with the pins you are using

# Claim and release the pins
claim_and_release_pins(pins_to_reset)

# Define the pins for the H-bridge
pinForward = 17  # Pin do przodu (GPIO 17)
pinBackward = 27 # Pin do tylu (GPIO 27)
pinPWM = 22      # Pin PWM (GPIO 12)
wartoscPWM = 0.8

# Set up the pins
forward = DigitalOutputDevice(pinForward)
backward = DigitalOutputDevice(pinBackward)
speed = PWMOutputDevice(pinPWM)

def motor_forward(speed_value):
    """Move forward with specified speed."""
    backward.off()
    forward.on()
    speed.value = speed_value  # Set speed (0.0 to 1.0)

def motor_backward(speed_value):
    """Move backward with specified speed."""
    forward.off()
    backward.on()
    speed.value = speed_value  # Set speed (0.0 to 1.0)

def motor_stop():
    """Stop the motor."""
    forward.off()
    backward.off()
    speed.value = 0

# Example usage of the functions
motor_backward(wartoscPWM)  # Move backward with the specified speed
time.sleep(1)  # Wait for 2 seconds
motor_forward(wartoscPWM)  # Move forward with the specified speed
time.sleep(1)  # Wait for 2 seconds
motor_stop()  # Stop the motor
