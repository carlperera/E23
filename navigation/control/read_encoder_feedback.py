"""
TEST PLAN:
1. test at a specific speed (at 100% maybe)
2. if that works, try cycling through different duty cycles (speeds) to see if encoder A and B values are correct positioning


for each motor:
    to find speed - find the counts per second from the output of A and B combined 
    

"""
import RPi.GPIO as GPIO

CPR = 48
GEAR_RATIO = 74.83

PIN_MOTOR1_IN1 = 17 # LOW - good
PIN_MOTOR1_IN2 = 27 # LOW -good 
PIN_MOTOR1_PWM_ENABLE = 18 # LOW - good 
PIN_MOTOR1_A_OUT = 21# LOW - good 
PIN_MOTOR1_B_OUT = 20 # LOW - good

PIN_MOTOR2_IN1 = 23 # LOW - good
PIN_MOTOR2_IN2 = 24 # LOW -good 
PIN_MOTOR2_PWM_ENABLE = 9 # LOW - good 
PIN_MOTOR2_A_OUT = 14# LOW - good 
PIN_MOTOR2_B_OUT = 15 # LOW - good

# last_time1 = 0
# last_time2 = 0
# duty_cycle1 = 0
# duty_cycle2 = 0

counts_motor1 = 0
counts_motor2 = 0
dir_motor1 = -1
dir_motor2 = -1

# default A and B values for encoder output feedbacks are LOW 
last_state1_A = GPIO.LOW
last_state1_B = GPIO.LOW
last_state2_A = GPIO.LOW
last_state2_B = GPIO.LOW


GPIO.setmode(GPIO.BCM)

# Motor 1 (left)
GPIO.setup(PIN_MOTOR1_IN1, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_MOTOR1_IN2, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_MOTOR1_PWM_ENABLE, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)

motor1_enable_pwm = GPIO.PWM(PIN_MOTOR1_PWM_ENABLE, 1000)

GPIO.setup(PIN_MOTOR1_A_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_MOTOR1_B_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Motor 2 (right)
GPIO.setup(PIN_MOTOR2_IN1, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_MOTOR2_IN2, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_MOTOR2_PWM_ENABLE, GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)

motor2_enable_pwm = GPIO.PWM(PIN_MOTOR2_PWM_ENABLE, 1000)

GPIO.setup(PIN_MOTOR2_A_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(PIN_MOTOR2_B_OUT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


motor1_enable_pwm.start(100) 
motor2_enable_pwm.start(100) 

def motor1_A_callback():
    global counts_motor1, last_state1_A, dir_motor1
    if GPIO.input(PIN_MOTOR1_A_OUT) == GPIO.HIGH and last_state1_A == GPIO.LOW:



def motor1_B_callback():
    global counts_motor1, last_state1_B, dir_motor1


def motor2_A_callback():
    global counts_motor2, last_state2_A, dir_motor2

def motor2_B_callback():
    global counts_motor2, last_state2_B, dir_motor2


GPIO.add_event_detect(PIN_MOTOR1_A_OUT, GPIO.BOTH, callback=motor1_A_callback)
GPIO.add_event_detect(PIN_MOTOR1_B_OUT, GPIO.BOTH, callback=motor1_B_callback)
GPIO.add_event_detect(PIN_MOTOR2_A_OUT, GPIO.BOTH, callback=motor2_A_callback)
GPIO.add_event_detect(PIN_MOTOR2_B_OUT, GPIO.BOTH, callback=motor2_B_callback)

GPIO.cleanup()

    







    




    




