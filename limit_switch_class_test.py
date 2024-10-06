from LimitSwitch import LimitSwitch
import time
import RPi.GPIO as GPIO

if __name__ == '__main__':
    try:

        GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
        GPIO.setwarnings(False)

        limit_switch = LimitSwitch()

        curr_state = 0 

        while True:
            if limit_switch.check_being_pressed():
                print("PRESSED")
            else:
                GPIO.cleanup()
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()



    

