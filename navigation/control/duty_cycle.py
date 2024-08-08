import RPi.GPIO as GPIO
import time


def get_duty_cycle(pin):
    # Variables to keep track of timing
    high_time = 0
    low_time = 0
    
    # Wait for the pin to go high
    while GPIO.input(pin) == GPIO.LOW:
        pass  # Wait for the rising edge
    start_time = time.time()  # Record the start time

    # Measure high time
    while GPIO.input(pin) == GPIO.HIGH:
        pass  # Wait for the pin to go low
    high_time = time.time() - start_time  # Record the duration of high state

    # Measure low time
    start_time = time.time()  # Reset the start time
    while GPIO.input(pin) == GPIO.LOW:
        pass  # Wait for the pin to go high
    low_time = time.time() - start_time  # Record the duration of low state

    # Calculate the duty cycle
    total_time = high_time + low_time
    duty_cycle = (high_time / total_time) * 100  # Convert to percentage

    return duty_cycle