from Servo import Flap, Claw
from time import sleep
import RPi.GPIO as GPIO


if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    # claw = Claw()
    # claw.collect_ball()

    flap = Flap()
    # flap.deposit_balls()
    # flap.close()
 
    try:

        # simulate flap release USE PIN 12 FOR FLAP
        while True: 
            # claw.open()
            # sleep(2)
            # claw.close()
            # sleep(2)
            input1 = input("1 for open or 2 for close: ")
            if input1 == "1":
                flap.open()
            else:
                flap.close()
            sleep(2)

            # flap.deposit_balls()
            
        

        # #simulate claw USE PIN 13 FOR CLAW
        # servo.rotate_to_180()
        # servo.rotate_from_180()

    except KeyboardInterrupt:
        # claw.close()
        # claw.stop()
        flap.close()
        flap.stop()
    # # finally:
    # #     # servo.stop()  # Ensure cleanup happens

