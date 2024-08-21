import RPi.GPIO as GPIO
import time

prevA = 0
prevB = 0
count = 0

CPR = 48
GEAR_RATIO = 74.83

# MOTOR 1 (right)
PIN_MOTOR1_IN1 = 17 # LOW
PIN_MOTOR1_IN2 = 27 # LOW 
PIN_MOTOR1_PWM_ENABLE = 18 # LOW
PIN_MOTOR1_A_OUT = 21# LOW 
PIN_MOTOR1_B_OUT = 20 # LOW 


# CONSTANTS
LOW = GPIO.LOW
HIGH = GPIO.HIGH

def interrupt_service_routine(channel) -> None:
    """ISR for the motor encoder

    Args:
        channel (_type_): Idk it needs to have an argument like how tasks in RTOS have that *pdata
        channel is GPIO pin number
    """
    global prevA, prevB, count


    a = GPIO.input(PIN_MOTOR1_A_OUT)
    b = GPIO.input(PIN_MOTOR1_B_OUT)

    # do teh logic:
    # 00 --> 01 --> 11 --> 10 is anti clock wise
    # 11 --> 01 --> 00 --> 10 is clock wise
    # increment or decrement count variable
    if (prevA == LOW and prevB == LOW and a == LOW and b == LOW) or \
       (prevA == GPIO.LOW and prevB == GPIO.HIGH and a == GPIO.HIGH and b == GPIO.HIGH) or \
       (prevA == GPIO.HIGH and prevB == GPIO.HIGH and a == GPIO.HIGH and b == GPIO.LOW) or \
       (prevA == GPIO.HIGH and prevB == GPIO.LOW and a == GPIO.LOW and b == GPIO.LOW):
        count += 1  # Clockwise
        # print("plus")
    elif (prevA == GPIO.HIGH and prevB == GPIO.HIGH and a == GPIO.LOW and b == GPIO.HIGH) or \
        (prevA == GPIO.LOW and prevB == GPIO.HIGH and a == GPIO.LOW and b == GPIO.LOW) or \
        (prevA == GPIO.LOW and prevB == GPIO.LOW and a == 1 and b == GPIO.LOW) or \
        (prevA == GPIO.HIGH and prevB == GPIO.LOW and a == GPIO.HIGH and b == GPIO.HIGH):
        count -= 1  # Counterclockwise
        # print("minus")
    else:
        #do nothing
        pass

    # update previous state:
    prevA = a
    prevB = b


if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    
    # """ MOTOR 1 (left)"""

    GPIO.setup(PIN_MOTOR1_A_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(PIN_MOTOR1_B_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    GPIO.setup(PIN_MOTOR1_IN1, GPIO.OUT)
    GPIO.setup(PIN_MOTOR1_IN2, GPIO.OUT)
    GPIO.setup(PIN_MOTOR1_PWM_ENABLE, GPIO.OUT)

    motor1_enable_pwm = GPIO.PWM(PIN_MOTOR1_PWM_ENABLE, 1000)

    # motor1_in1 = GPIO.PWM(PIN_MOTOR1_IN1, 1000)
    # motor1_in2 = GPIO.PWM(PIN_MOTOR1_IN2, 1000)
    
    motor1_enable_pwm.start(100) 
    # motor1_in1.start(100)
    # motor1_in2.start(0)
    GPIO.output(PIN_MOTOR1_IN1, GPIO.HIGH)
    GPIO.output(PIN_MOTOR1_IN2, GPIO.LOW)



    
    # set the callback function to be called:
    GPIO.add_event_detect(
        PIN_MOTOR1_A_OUT, edge=GPIO.BOTH, callback=interrupt_service_routine
    )
    GPIO.add_event_detect(
        PIN_MOTOR1_B_OUT, edge=GPIO.BOTH, callback=interrupt_service_routine
    )


    timeInterval = 0.5  # calculate speed every 0.5 seconds

    
    while True:
        try:
            # calculate motor speed:
            # revs of the motor shaft:
            revs = count / (CPR * GEAR_RATIO)

            # convert revolutions to rpm:
            rpm = (revs / timeInterval) * 60

            print(f"RPM = {rpm:.2f} RPM, count = {count: .2f}")

            # reset the encoder count to zero (so the past encoder counts dont affect the calculation of the next rpm)
            count = 0

            # sleep
            time.sleep(timeInterval)
            
        except KeyboardInterrupt:
            # motor1_in1.stop()
            # motor1_in2.stop()
        

            GPIO.cleanup()
            print("Done")
            break