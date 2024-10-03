from Ultrasonic import Ultrasonic
import time
import RPi.GPIO as GPIO

if __name__ == '__main__':
    try:
        ultrasonic = Ultrasonic()
        while True:
            dist = ultrasonic.averaged_distance(numReadings=5)
            print ("Measured Distance = %.2f cm" % dist)
            time.sleep(1)
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()