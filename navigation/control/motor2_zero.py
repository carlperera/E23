from gpiozero import RotaryEncoder, PWMOutputDevice
import time

count = 0

CPR = 48
SPR = 48
GEAR_RATIO = 74.83

# # MOTOR 1 (left)
# PIN_MOTOR1_IN1 = 17
# PIN_MOTOR1_IN2 = 27
# PIN_MOTOR1_PWM_ENABLE = 18
# PIN_MOTOR1_A_OUT = 21
# PIN_MOTOR1_B_OUT = 20

# MOTOR 2 (right)
PIN_MOTOR2_IN1 = 23 # LOW - good
PIN_MOTOR2_IN2 = 24 # LOW -good 
PIN_MOTOR2_PWM_ENABLE = 9 # LOW - good 
PIN_MOTOR2_A_OUT = 14# LOW - good 
PIN_MOTOR2_B_OUT = 15 # LOW - good


if __name__ == "__main__":
    # Set up the motor control pins
    motor2_in1 = PWMOutputDevice(PIN_MOTOR2_IN1, frequency=1000)
    motor2_in2 = PWMOutputDevice(PIN_MOTOR2_IN2, frequency=1000)
    motor2_enable = PWMOutputDevice(PIN_MOTOR2_PWM_ENABLE, frequency=1000)

    # Set up the rotary encoder
    encoder = RotaryEncoder(PIN_MOTOR2_A_OUT, PIN_MOTOR2_B_OUT)

    motor2_enable.value = 1  # Enable the motor
    motor2_in1.value = 1  # Set direction
    motor2_in2.value = 0  # Set opposite direction


    # Attach the callback function to the encoder
    # encoder.when_rotated = interrupt_service_routine(encoder)

    timeInterval = 0.5  # Calculate speed every 0.5 seconds

    start_time = time.time()

    try:
        encoder.steps = 0 
        while True:

            # Wait for the time interval
            time.sleep(timeInterval)
            
            # Calculate the time elapsed
            elapsed_time = time.time() - start_time

            # Get the current step count
            steps = encoder.steps

            # Calculate the number of revolutions
            revs = steps / (SPR)

            # Convert to RPM
            rpm = (revs / elapsed_time) * 60

            print(f"RPM = {rpm:.2f} RPM, Steps = {steps}")

            # Reset the encoder steps to zero for the next interval
            encoder.steps = 0

            # Update the start time
            start_time = time.time()
            
    except KeyboardInterrupt:
        motor2_in1.off()
        motor2_in2.off()
        motor2_enable.off()
        print("Done")
