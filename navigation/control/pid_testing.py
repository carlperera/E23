import gpiozero
import time

pwm = gpiozero.PWMOutputDevice(pin=19,active_high=True,initial_value=0,frequency=100)
dir1 = gpiozero.OutputDevice(pin=27)
dir2 = gpiozero.OutputDevice(pin=17)
encoder = gpiozero.RotaryEncoder(a=21, b=20,max_steps=100000) # encoder pins a = yellow, b = white
# This class has a lot more functionality,so worth reading up on it
# Step through duty cycle values, slowly increasing the speed and changing the direction of motion
encoder.steps = 0
for j in range(10):
    pwm.value = j/10
    dir1.value = not dir1.value
    dir2.value = not dir1.value
    print('Duty cycle:',pwm.value,'Direction:',dir1.value)
    time.sleep(5.0)
    print('Counter:',encoder.steps,'Speed:',(encoder.steps)/5.0,'steps per second\n')
    encoder.steps = 0

pwm.value =0  