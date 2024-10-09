from LimitSwitch import LimitSwitch
import time
import RPi.GPIO as GPIO

LIMIT_SWITCH_LEFT_PIN = 7 
LIMIT_SWITCH_LEFT_PIN = 8

if __name__ == '__main__':
    
    
    try:

        limit_switch_left = LimitSwitch(pin_number = LIMIT_SWITCH_LEFT_PIN)
        limit_switch_right = LimitSwitch(pin_number = LIMIT_SWITCH_LEFT_PIN)

        curr_state = 0 

        while True:
            if limit_switch_left.check_being_pressed():
                print("PRESSED left")
            if limit_switch_right.check_being_pressed():
                print("PRESSED right")

            else:
                GPIO.cleanup()
 
        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()



    

