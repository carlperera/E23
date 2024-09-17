import RPi.GPIO as GPIO
import time

motor1_prevA = 0
motor1_prevB = 0
motor1_count = 0

motor2_prevA = 0
motor2_prevB = 0
motor2_count = 0

CPR = 48
GEAR_RATIO = 74.83

# MOTOR 1 (right)
PIN_MOTOR1_IN1 = 17 # LOW
PIN_MOTOR1_IN2 = 27 # LOW 
PIN_MOTOR1_PWM_ENABLE = 18 # LOW
PIN_MOTOR1_A_OUT = 21# LOW 
PIN_MOTOR1_B_OUT = 20 # LOW 

# MOTOR 2 (right)
PIN_MOTOR2_IN1 = 23 # LOW - good
PIN_MOTOR2_IN2 = 24 # LOW -good 
PIN_MOTOR2_PWM_ENABLE = 9 # LOW - good 
PIN_MOTOR2_A_OUT = 14# LOW - good 
PIN_MOTOR2_B_OUT = 15 # LOW - good

# CONSTANTS
LOW = GPIO.LOW
HIGH = GPIO.HIGH

def interrupt_service_routine1(channel) -> None:
    """ISR for the motor encoder

    Args:
        channel (_type_): Idk it needs to have an argument like how tasks in RTOS have that *pdata
        channel is GPIO pin number
    """
    global motor1_prevA, motor1_prevB, motor1_count


    a = GPIO.input(PIN_MOTOR1_A_OUT)
    b = GPIO.input(PIN_MOTOR1_B_OUT)

    # do teh logic:
    # 00 --> 01 --> 11 --> 10 is anti clock wise
    # 11 --> 01 --> 00 --> 10 is clock wise
    # increment or decrement count variable
    if (motor1_prevA == LOW and motor1_prevB == LOW and a == LOW and b == LOW) or \
       (motor1_prevA == LOW and motor1_prevB == GPIO.HIGH and a == GPIO.HIGH and b == GPIO.HIGH) or \
       (motor1_prevA == GPIO.HIGH and motor1_prevB == GPIO.HIGH and a == GPIO.HIGH and b == LOW) or \
       (motor1_prevA == LOW and motor1_prevB == LOW and a == LOW and b == LOW):
        motor1_count += 1  # Clockwise
        # print("plus")
    elif (motor1_prevA == GPIO.HIGH and motor1_prevB == GPIO.HIGH and a == GPIO.LOW and b == GPIO.HIGH) or \
        (motor1_prevA == GPIO.LOW and motor1_prevB == GPIO.HIGH and a == GPIO.LOW and b == GPIO.LOW) or \
        (motor1_prevA == GPIO.LOW and motor1_prevB == GPIO.LOW and a == 1 and b == GPIO.LOW) or \
        (motor1_prevA == GPIO.HIGH and motor1_prevB == GPIO.LOW and a == GPIO.HIGH and b == GPIO.HIGH):
        motor1_count -= 1  # Counterclockwise
        # print("minus")
    else:
        #do nothing
        pass

    # update previous state:
    motor1_prevA = a
    motor1_prevB = b

def interrupt_service_routine2(channel) -> None:
    """ISR for the motor encoder

    Args:
        channel (_type_): Idk it needs to have an argument like how tasks in RTOS have that *pdata
        channel is GPIO pin number
    """
    global motor2_prevA, motor2_prevB, motor2_count


    a = GPIO.input(PIN_MOTOR2_A_OUT)
    b = GPIO.input(PIN_MOTOR2_B_OUT)

    # do teh logic:
    # 00 --> 01 --> 11 --> 10 is anti clock wise
    # 11 --> 01 --> 00 --> 10 is clock wise
    # increment or decrement count variable
    if (motor2_prevA == LOW and motor2_prevB == LOW and a == LOW and b == LOW) or \
       (motor2_prevA == LOW and motor2_prevB == GPIO.HIGH and a == GPIO.HIGH and b == GPIO.HIGH) or \
       (motor2_prevA == GPIO.HIGH and motor2_prevB == GPIO.HIGH and a == GPIO.HIGH and b == LOW) or \
       (motor2_prevA == LOW and motor2_prevB == LOW and a == LOW and b == LOW):
        motor2_count += 1  # Clockwise
        # print("plus")
    elif (motor2_prevA == GPIO.HIGH and motor2_prevB == GPIO.HIGH and a == GPIO.LOW and b == GPIO.HIGH) or \
        (motor2_prevA == GPIO.LOW and motor2_prevB == GPIO.HIGH and a == GPIO.LOW and b == GPIO.LOW) or \
        (motor2_prevA == GPIO.LOW and motor2_prevB == GPIO.LOW and a == 1 and b == GPIO.LOW) or \
        (motor2_prevA == GPIO.HIGH and motor2_prevB == GPIO.LOW and a == GPIO.HIGH and b == GPIO.HIGH):
        motor2_count -= 1  # Counterclockwise
        # print("minus")
    else:
        #do nothing
        pass

    # update previous state:
    motor2_prevA = a
    motor2_prevB = b

def setup_motor1_pins():
    
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

def setup_motor2_pins():
    # """ MOTOR 2 (right)"""

    GPIO.setup(PIN_MOTOR2_A_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(PIN_MOTOR2_B_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    GPIO.setup(PIN_MOTOR2_IN1, GPIO.OUT)
    GPIO.setup(PIN_MOTOR2_IN2, GPIO.OUT)
    GPIO.setup(PIN_MOTOR2_PWM_ENABLE, GPIO.OUT)

    motor2_enable_pwm = GPIO.PWM(PIN_MOTOR2_PWM_ENABLE, 1000)

    motor2_enable_pwm.start(100) 
    # motor1_in1.start(100)
    # motor1_in2.start(0)
    GPIO.output(PIN_MOTOR2_IN1, GPIO.HIGH)
    GPIO.output(PIN_MOTOR2_IN2, GPIO.LOW)

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    setup_motor1_pins()
    setup_motor2_pins()
 
    GPIO.add_event_detect(
        PIN_MOTOR1_A_OUT, edge=GPIO.BOTH, callback=interrupt_service_routine1
    )
    GPIO.add_event_detect(
        PIN_MOTOR1_B_OUT, edge=GPIO.BOTH, callback=interrupt_service_routine1
    )

    GPIO.add_event_detect(
        PIN_MOTOR2_A_OUT, edge=GPIO.BOTH, callback=interrupt_service_routine2
    )
    GPIO.add_event_detect(
        PIN_MOTOR2_B_OUT, edge=GPIO.BOTH, callback=interrupt_service_routine2
    )


    timeInterval = 0.5  # calculate speed every 0.5 seconds
    
    while True:
        try:
            """MOTOR 1"""
            # calculate motor speed:
            # revs of the motor shaft:
            revs1 = motor1_count / (CPR * GEAR_RATIO)
            

            # convert revolutions to rpm:
            rpm1 = (revs1 / timeInterval) * 60

            print(f"RPM = {rpm1:.2f} RPM, count = {motor1_count: .2f}")

            # reset the encoder count to zero (so the past encoder counts dont affect the calculation of the next rpm)
            motor1_count = 0

            """MOTOR 2"""
            # calculate motor speed:
            # revs of the motor shaft:
            revs2 = motor2_count / (CPR * GEAR_RATIO)
            
            # convert revolutions to rpm:
            rpm2 = (revs2 / timeInterval) * 60

            print(f"RPM = {rpm2:.2f} RPM, count = {motor2_count: .2f}")

            # reset the encoder count to zero (so the past encoder counts dont affect the calculation of the next rpm)
            motor2_count = 0

            # sleep
            time.sleep(timeInterval)
            
        except KeyboardInterrupt:


            GPIO.cleanup()
            print("Done")
            break
