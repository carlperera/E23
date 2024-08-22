import multiprocessing
from multiprocessing import Process, Value
import RPi.GPIO as GPIO
from RobotController import RobotController
from DiffDriveRobot import DiffDriveRobot

print("Number of cpu : ", multiprocessing.cpu_count())

"""CONSTANTS """
CPR = 48
GEAR_RATIO = 74.83

"""PINS for Motor 1 (left )"""
PIN_MOTOR1_IN1 = 17 
PIN_MOTOR1_IN2 = 27 
PIN_MOTOR1_PWM_ENABLE = 18
PIN_MOTOR1_A_OUT = 21
PIN_MOTOR1_B_OUT = 20

"""PINS for Motor 2 (right)"""
PIN_MOTOR2_IN1 = 23 
PIN_MOTOR2_IN2 = 24 
PIN_MOTOR2_PWM_ENABLE = 9  
PIN_MOTOR2_A_OUT = 14
PIN_MOTOR2_B_OUT = 15 

def setup_pins():
    #------------ setup the robot and the controller --------
    robot = DiffDriveRobot(inertia=5, dt=0.1, drag=1, wheel_radius=WHEEL_RAD, wheel_sep=WHEEL_SEP)
    controller = RobotController(Kp=1,Ki=0.25,wheel_radius=WHEEL_RAD,wheel_sep=WHEEL_SEP)

    # ----------- setup pins ----------
    # Set GPIO modes
    GPIO.setmode(GPIO.BCM)

    """ MOTOR 1 (left) """
    GPIO.setup(PIN_MOTOR1_A_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(PIN_MOTOR1_B_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    GPIO.setup(PIN_MOTOR1_IN1, GPIO.OUT)
    GPIO.setup(PIN_MOTOR1_IN2, GPIO.OUT)
    GPIO.setup(PIN_MOTOR1_PWM_ENABLE, GPIO.OUT)

    motor1_enable_pwm = GPIO.PWM(PIN_MOTOR1_PWM_ENABLE, 1000)
    motor1_enable_pwm.start(100) # initial speed

    GPIO.output(PIN_MOTOR1_IN1, GPIO.HIGH)
    GPIO.output(PIN_MOTOR1_IN2, GPIO.LOW)

    """ MOTOR 2 (right) """
    GPIO.setup(PIN_MOTOR2_A_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(PIN_MOTOR2_B_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    GPIO.setup(PIN_MOTOR2_IN1, GPIO.OUT)
    GPIO.setup(PIN_MOTOR2_IN2, GPIO.OUT)
    GPIO.setup(PIN_MOTOR2_PWM_ENABLE, GPIO.OUT)

    motor2_enable_pwm = GPIO.PWM(PIN_MOTOR2_PWM_ENABLE, 1000)
    motor2_enable_pwm.start(100) # initial speed

    GPIO.output(PIN_MOTOR2_IN1, GPIO.HIGH)
    GPIO.output(PIN_MOTOR2_IN2, GPIO.LOW)

    return None 


def navigation():
    setup_pins()
    return None 

def vision():

    return None 



if __name__ == "__main__":

    counter = Value('i',0)

    proc1 = Process(target=navigation,args=(counter,))
    proc2 = Process(target=vision, args=(counter,))

    proc1.start() #Start the process
    proc2.start()

    proc1.join()  #Wait for the process to stop
    proc2.join()  #Wait for the process to stop

    # core 1 - dedicated to navigation/control
        # thread 1 - navigation 

        # thread 2 - query the counts for each encoder 

    # core 2 - vision

    # core 3 

    # core 4 

    # make into a while loop that awaits commands in CLI
    while True:
        try:



        except KeyboardInterrupt:

            print("Done\n")
            
    




    