import RPi.GPIO as GPIO
import time

# Set up the GPIO mode and pin
GPIO.setmode(GPIO.BCM)  # Use physical pin numbers
GPIO.setup(8, GPIO.IN)  # Use internal pull-down resistor
GPIO.setup(7, GPIO.IN)

try:
    while True:
        if GPIO.input(8) == GPIO.HIGH or GPIO.input(7) == GPIO.HIGH:
            print("Switch Pressed (High)")
        else:
            print("Switch Released (Low)")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program stopped.")
    GPIO.cleanup()  # Clean up GPIO settings when exiting