import gpiozero
import time

SPR = 48
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


motor1_pwm = gpiozero.PWMOutputDevice(pin=PIN_MOTOR1_PWM_ENABLE,active_high=True,initial_value=0,frequency=100)
motor1_in1 = gpiozero.OutputDevice(pin=PIN_MOTOR1_IN1)
motor1_in2 = gpiozero.OutputDevice(pin=PIN_MOTOR1_IN2)
motor1_encoder = gpiozero.RotaryEncoder(a=PIN_MOTOR1_A_OUT, b=PIN_MOTOR1_B_OUT,max_steps=100000) 

motor2_pwm = gpiozero.PWMOutputDevice(pin=PIN_MOTOR2_PWM_ENABLE,active_high=True,initial_value=0,frequency=100)
motor2_in1 = gpiozero.OutputDevice(pin=PIN_MOTOR2_IN1)
motor2_in2 = gpiozero.OutputDevice(pin=PIN_MOTOR2_IN2)
motor2_encoder = gpiozero.RotaryEncoder(a=PIN_MOTOR2_A_OUT, b=PIN_MOTOR2_B_OUT,max_steps=100000) 

motor1_encoder.steps = 0
motor2_encoder.steps = 0

for j in range(10):
    motor1_pwm.value = j/10
    motor1_in1.value = not motor1_in1.value
    motor1_in2.value = not motor1_in1.value
    print('1-----Duty cycle:',motor1_pwm.value,'Direction:',motor1_in1.value)


    motor2_pwm.value = j/10
    motor2_in1.value = not motor2_in1.value
    motor2_in2.value = not motor2_in1.value
    print('2-----Duty cycle:',motor2_pwm.value,'Direction:',motor2_in1.value)

    time.sleep(5.0)

    
    print('1-----Counter:',motor1_encoder.steps,'Speed:',(motor1_encoder.steps)/5.0,'steps per second\n')
    motor1_steps = motor1_encoder.steps

    # Calculate the number of revolutions
    motor1_revs = motor1_steps / (SPR* GEAR_RATIO)

    # Convert to RPM
    motor1_rpms = (motor1_revs / 5) * 60

    print(f"RPM = {motor1_rpms:.2f} RPM, Steps = {motor1_steps}")
    
    motor1_encoder.steps = 0




    print('2-----Counter:',motor2_encoder.steps,'Speed:',(motor2_encoder.steps)/5.0,'steps per second\n')
    motor2_steps = motor2_encoder.steps

    # Calculate the number of revolutions
    motor2_revs = motor2_steps / (SPR* GEAR_RATIO)

    # Convert to RPM
    motor2_rpms = (motor2_revs / 5) * 60

    print(f"RPM = {motor2_rpms:.2f} RPM, Steps = {motor2_steps}")
    
    motor2_encoder.steps = 0

motor1_pwm.value =0 
motor2_pwm.value =0 