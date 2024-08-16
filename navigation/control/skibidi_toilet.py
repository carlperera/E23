import RPi.GPIO as GPIO
import time

prevA = 0
prevB = 0
count = 0

ENCODER_A_PIN = 69
ENCODER_B_PIN = 420
CPR = 48
GEAR_RATIO = 75


def interrupt_service_routine(channel) -> None:
    """ISR for the motor encoder

    Args:
        channel (_type_): Idk it needs to have an argument like how tasks in RTOS have that *pdata
        channel is GPIO pin number
    """
    global prevA, prevB, count

    a = GPIO.input(ENCODER_A_PIN)
    b = GPIO.input(ENCODER_B_PIN)

    # do teh logic:
    # 00 --> 01 --> 11 --> 10 is anti clock wise
    # 11 --> 01 --> 00 --> 10 is clock wise
    # increment or decrement count variable
    if (prevA == 0 and prevB == 0 and a == 0 and b == 1) or \
       (prevA == 0 and prevB == 1 and a == 1 and b == 1) or \
       (prevA == 1 and prevB == 1 and a == 1 and b == 0) or \
       (prevA == 1 and prevB == 0 and a == 0 and b == 0):
        count += 1  # Clockwise
    elif (prevA == 1 and prevB == 1 and a == 0 and b == 1) or \
        (prevA == 0 and prevB == 1 and a == 0 and b == 0) or \
        (prevA == 0 and prevB == 0 and a == 1 and b == 0) or \
        (prevA == 1 and prevB == 0 and a == 1 and b == 1):
        count -= 1  # Counterclockwise
    else:
        #do nothing
        pass

    # update previous state:
    prevA = a
    prevB = b


if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    # set up encoder pins as inputs:
    GPIO.setup(ENCODER_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(ENCODER_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    # set the callback function to be called:
    GPIO.add_event_detect(
        ENCODER_A_PIN, edge=GPIO.BOTH, callback=interrupt_service_routine
    )
    GPIO.add_event_detect(
        ENCODER_B_PIN, edge=GPIO.BOTH, callback=interrupt_service_routine
    )

    timeInterval = 0.5  # calculate speed every 0.5 seconds

    try:
        while True:
            # calculate motor speed:
            # revs of the motor shaft:
            revs = count / (CPR * GEAR_RATIO)

            # convert revolutions to rpm:
            rpm = (revs / timeInterval) * 60

            # reset the encoder count to zero (so the past encoder counts dont affect the calculation of the next rpm)
            count = 0

            print(f"The skibidi rizz is currently {rpm:.2f} RPM")

            # honk shoo honk shoo
            time.sleep(timeInterval)
    except KeyboardInterrupt:
        GPIO.cleanup()
