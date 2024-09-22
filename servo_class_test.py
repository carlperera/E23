from Servo import Flap, Claw
from time import sleep
import RPi.GPIO as GPIO

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    claw = Claw()
 

    try:

        # simulate flap release USE PIN 12 FOR FLAP
        while True: 
            claw.open()
            claw.close()

        # #simulate claw USE PIN 13 FOR CLAW
        # servo.rotate_to_180()
        # servo.rotate_from_180()

    except KeyboardInterrupt:
        claw.stop()
    # finally:
    #     # servo.stop()  # Ensure cleanup happens

