import gpiozero
from gpiozero import Motor
import time
import RPi.GPIO as GPIO

# Set GPIO modes
GPIO.setmode(GPIO.BCM)
motor_A_in1 = 17
motor_A_in2 = 27

motor_B_in1 = 23
motor_B_in2 = 24    

# motor_B_in2 = 24
# motor_B_in1 = 25
# motor_B_en = 19

motor_enable = 18
motor_enable2 = 10




GPIO.setup(motor_A_in1, GPIO.OUT)
GPIO.setup(motor_A_in2, GPIO.OUT)

GPIO.setup(motor_B_in1, GPIO.OUT)
GPIO.setup(motor_B_in2, GPIO.OUT)
GPIO.setup(motor_enable, GPIO.OUT)
GPIO.setup(motor_enable2, GPIO.OUT)


# GPIO.setup(motor_B_in 1, GPIO.OUT)
# GPIO.setup(motor_B_in2, GPIO.OUT)
# GPIO.setup(motor_B_en, GPIO.OUT)

m1_pwm1 = GPIO.PWM(motor_A_in1, 1000)
m1_pwm2 = GPIO.PWM(motor_A_in2, 1000)
m2_pwm1 = GPIO.PWM(motor_B_in1, 1000)
m2_pwm2 = GPIO.PWM(motor_B_in2, 1000)

enable_pwm = GPIO.PWM(motor_enable, 1000)
enable_pwm2 = GPIO.PWM(motor_enable2, 1000)

enable_pwm.start(100) # Turning on both enables
enable_pwm2.start(100)

#input 1 and input 2 pins used for motor A
#input 1 LOW and input 2 HIGH = forward
#input 1 HIGH and input 2 LOW = backwards
#input 3 and input 4 pins used for motor B
#input 3 LOW and input 4 HIGH = forward
#input 3 HIGH and input 4 LOW = backwards

def forwards(speed):
    m1_pwm1.start(speed)
    m2_pwm1.start(speed)

def backwards(speed):
    m1_pwm2.start(speed)
    m2_pwm2.start(speed)

def right(direction, speed):
    if direction == "f":
        m1_pwm1.start(speed/4) # 25 on the left motor
        m2_pwm1.start(speed) # 100 on right motor
    else:
        m1_pwm2.start(speed/4)
        m2_pwm2.start(speed)

def left(direction, speed):
    if direction == "f":
        m1_pwm1.start(speed)
        m2_pwm1.start(speed/4) # 100 on right motor
    else:
        m1_pwm2.start(speed)
        m2_pwm2.start(speed/4)


while True:
    try:
        directionFlag = input("Please state first letter of direction and speed: ")
        direction = directionFlag.split(" ")[0]
        speed =  int(directionFlag.split(" ")[1])
        print(direction)

        if direction == "b": # if user types "back" change direction of motor
            backwards(speed)
                
        if direction == "f":
            forwards(speed)

        if direction == "bl":
            left("b", speed)
        if direction == "br":
            right("b", speed)
        if direction == "fl": # front left
            left("f", speed)
        if direction == "fr": # Front right
            right("f", speed)
        if direction == "bl":
            left("b", speed)
        if direction == "br":
            right("b", speed)

        if direction == "s":
            m1_pwm1.stop()
            m1_pwm2.stop()
            m2_pwm1.stop()
            m2_pwm2.stop()

    except KeyboardInterrupt:
        m1_pwm1.stop()
        m1_pwm2.stop()
        m2_pwm1.stop()
        m2_pwm2.stop()

        GPIO.cleanup()
        print("Done")
        break




    # directionFlag = input("set motor direction or type end to stop: ")
    
    


# Rotate both motor L and R anticlockwise for 3 seconds



#Setup pins
# left_motor = Motor(18,22) # 18 forwards, 22 backwards

# left_motor.forward(1)
# time.sleep(3)
# left_motor.stop()


# Backward = gpiozero.OutputDevice(18) # On/Off output
# Forward = gpiozero.OutputDevice(22) #On/Off output

# SpeedPWM = gpiozero.PWMOutputDevice(13) # set up PWM pin

# while True:
#     directionFlag = input("set motor direction: ")
#     if directionFlag == "back": # if user types "back" change direction of motor
#         Backward.on() # Sets Backward Direction pin on
#         Forward.off() # Sets Backward Direction pin on
#     else:
#         Backward.off() # Sets Backward Direction off
#         Forward.on()   # Sets Backward Direction pin on
#     speedFlag = float(input("set speed (between 0-1000): ")) # Gets a number from the from the user
#     SpeedPWM.value = speedFlag/1000 # Sets the duty cycle of the PWM between 0-1