import gpiozero
import time

PIN_MOTOR1_IN1 = 17 # LOW - good
PIN_MOTOR1_IN2 = 27 # LOW -good 
PIN_MOTOR1_PWM_ENABLE = 18 # LOW - good 
PIN_MOTOR1_A_OUT = 21# LOW - good 
PIN_MOTOR1_B_OUT = 20 # LOW - good

pwm1 = gpiozero.PWMOutputDevice(pin=PIN_MOTOR1_PWM_ENABLE,active_high=True,initial_value=0,frequency=100)
dir1 = gpiozero.OutputDevice(pin=PIN_MOTOR1_IN1)
dir2 = gpiozero.OutputDevice(pin=PIN_MOTOR1_IN2)

encoder1 = gpiozero.RotaryEncoder(a=PIN_MOTOR1_A_OUT, b=PIN_MOTOR1_B_OUT,max_steps=100000) 
# This class has a lot more functionality,so worth reading up on it

#  Step through duty cycle values, slowly increasing the speed and changing the direction of motion
encoder1.steps = 0
for j in range(10):
    pwm1.value = j/10
    dir1.value = not dir1.value
    dir2.value = not dir1.value
    print('Duty cycle:',pwm1.value,'Direction:',dir1.value)
    time.sleep(5.0)
    print('Counter:',encoder1.steps,'Speed:',(encoder1.steps)/5.0,'steps per second\n')
    encoder1.steps = 0

pwm1.value =0 