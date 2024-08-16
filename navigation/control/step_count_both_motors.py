import gpiozero
import time

"""
WILL NOT WORK TO ENGAGE BOTH MOTORS (for some reason I need to find out)

"""
# RIGHT MOTOR (2)
PIN_MOTOR2_IN1 = 23 # LOW - good
PIN_MOTOR2_IN2 = 24 # LOW -good 
PIN_MOTOR2_PWM_ENABLE = 9 # LOW - good 
PIN_MOTOR2_A_OUT = 14# LOW - good 
PIN_MOTOR2_B_OUT = 15 # LOW - good

pwm2 = gpiozero.PWMOutputDevice(pin=PIN_MOTOR2_PWM_ENABLE,active_high=True,initial_value=0,frequency=100)
dir3 = gpiozero.OutputDevice(pin=PIN_MOTOR2_IN1)
dir4 = gpiozero.OutputDevice(pin=PIN_MOTOR2_IN2)

encoder2 = gpiozero.RotaryEncoder(a=PIN_MOTOR2_A_OUT, b=PIN_MOTOR2_B_OUT,max_steps=100000) 


# LEFT MOTOR (1)
PIN_MOTOR1_IN1 = 17 # LOW - good
PIN_MOTOR1_IN2 = 27 # LOW -good 
PIN_MOTOR1_PWM_ENABLE = 18 # LOW - good 
PIN_MOTOR1_A_OUT = 21# LOW - good 
PIN_MOTOR1_B_OUT = 20 # LOW - good

pwm1 = gpiozero.PWMOutputDevice(pin=PIN_MOTOR1_PWM_ENABLE,active_high=True,initial_value=0,frequency=100)
dir1 = gpiozero.OutputDevice(pin=PIN_MOTOR1_IN1)
dir2 = gpiozero.OutputDevice(pin=PIN_MOTOR1_IN2)

encoder1 = gpiozero.RotaryEncoder(a=PIN_MOTOR1_A_OUT, b=PIN_MOTOR1_B_OUT,max_steps=100000) 


encoder2.steps = 0
encoder1.steps = 0
for j in range(10):
    pwm2.value = j/10
    dir3.value = not dir3.value
    dir4.value = not dir3.value
    print('2----Duty cycle:',pwm2.value,'Direction:',dir3.value)

    pwm1.value = j/10
    dir1.value = not dir1.value
    dir2.value = not dir2.value
    print('1----Duty cycle:',pwm1.value,'Direction:',dir1.value)


    time.sleep(5.0)
    print('2----Counter:',encoder2.steps,'Speed:',(encoder2.steps)/5.0,'steps per second\n')
    encoder2.steps = 0


    print('1----Counter:',encoder1.steps,'Speed:',(encoder1.steps)/5.0,'steps per second\n')
    encoder2.steps = 0

pwm2.value = 0 
pwm1.value = 0

