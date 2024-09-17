from Servo import Servo
from time import sleep

if __name__ == "__main__":
    servo = Servo(pin=12)
 

    try:

        # simulate flap release USE PIN 12 FOR FLAP
        servo.rotate_from_90()
        sleep(5)
        servo.rotate_to_90()

        # #simulate claw USE PIN 13 FOR CLAW
        # servo.rotate_to_180()
        # servo.rotate_from_180()

    except KeyboardInterrupt:
        servo.stop()
    # finally:
    #     # servo.stop()  # Ensure cleanup happens

