import RPi.GPIO as GPIO
import time
from enum import Enum

class ULTRASONIC_PINS(Enum):
    ECHO_PIN = 7 #echo pin 
    TRIG_PIN = 8 #trigger pin

class Ultrasonic:
    def __init__(self):
        self.trigPin = ULTRASONIC_PINS.TRIG_PIN.value
        self.echoPin = ULTRASONIC_PINS.ECHO_PIN.value
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trigPin,GPIO.OUT)
        GPIO.setup(self.echoPin, GPIO.IN)
        
    def single_distance(self):
        delay = 0.00002
        GPIO.output(self.trigPin, True)
        time.sleep(delay)
        GPIO.output(self.trigPin, False)

        while GPIO.input(self.echoPin) == 0:
            startTime = time.monotonic_ns()
        while GPIO.input(self.echoPin) == 1:
            stopTime = time.monotonic_ns()

        timeDiff = (stopTime - startTime)/1e9
        distance = (timeDiff*34300)/2
        return distance

    def averaged_distance(self, numReadings=5):
        distances = []
        for _ in range(numReadings):
            dist = self.single_distance()
            distances.append(dist)
            time.sleep(0.05)
        avgDist = sum(distances)/len(distances)
        return avgDist