import RPi.GPIO as GPIO
import time

# Set the GPIO mode to BCM or BOARD (BCM is recommended)
GPIO.setmode(GPIO.BCM)

# Define the GPIO pin number
input_pin = 8  # Trig PIN

# Set the input pin to be an input
GPIO.setup(input_pin, GPIO.IN)



try:
    # Continuously read the state of the input pin
    while True:
        # Read the state of the input pin (HIGH or LOW)
        input_state = GPIO.input(input_pin)
        
        if input_state == GPIO.HIGH:
            print("GPIO pin is HIGH")
        else:
            print("GPIO pin is LOW")
        
        # Sleep for a short time to avoid excessive CPU usage (e.g., 0.1 seconds)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program terminated by user.")

finally:
    # Clean up GPIO settings when the program is stopped
    GPIO.cleanup()

# Clean up GPIO settings
GPIO.cleanup()
