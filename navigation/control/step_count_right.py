import gpiozero
import time

PIN_MOTOR2_IN1 = 23 # LOW - good
PIN_MOTOR2_IN2 = 24 # LOW -good 
PIN_MOTOR2_PWM_ENABLE = 9 # LOW - good 
PIN_MOTOR2_A_OUT = 14# LOW - good 
PIN_MOTOR2_B_OUT = 15 # LOW - good

pwm2 = gpiozero.PWMOutputDevice(pin=PIN_MOTOR2_PWM_ENABLE,active_high=True,initial_value=0,frequency=100)
dir3 = gpiozero.OutputDevice(pin=PIN_MOTOR2_IN1)
dir4 = gpiozero.OutputDevice(pin=PIN_MOTOR2_IN2)

encoder2 = gpiozero.RotaryEncoder(a=PIN_MOTOR2_A_OUT, b=PIN_MOTOR2_B_OUT,max_steps=100000) 
# This class has a lot more functionality,so worth reading up on it

#  Step through duty cycle values, slowly increasing the speed and changing the direction of motion
encoder2.steps = 0
for j in range(10):
    pwm2.value = j/10
    dir3.value = not dir3.value
    dir4.value = not dir3.value
    print('Duty cycle:',pwm2.value,'Direction:',dir3.value)
    time.sleep(5.0)
    print('Counter:',encoder2.steps,'Speed:',(encoder2.steps)/5.0,'steps per second\n')
    encoder2.steps = 0

pwm2.value = 0 

