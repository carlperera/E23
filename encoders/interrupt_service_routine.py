def ISR_motor2(channel) -> None:
    """The frequency of this interrupt service routine is dependent on the duty cycle of 
    A and B output feedbacks 
    
    """
    global moto

    motor2_A = GPIO.input(PIN_MOTOR2_A_OUT)
    motor2_B = GPIO.input(PIN_MOTOR2_A_OUT)

    if (motor2_prev_A == LOW and motor2_A == HIGH and motor2_prev_B == LOW and motor2_B == LOW) or \
        (motor2_prev_A == HIGH and motor2_A == HIGH and motor2_prev_B == LOW and motor2_B == HIGH) or \
        (motor2_prev_A == HIGH and motor2_A == LOW and motor2_prev_B == HIGH and motor2_B == HIGH) or \
        (motor2_prev_A == LOW and motor2_A == LOW and motor2_prev_B == HIGH and motor2_B == LOW):
            counts += 1  # Clockwise

    elif (motor2_prev_A == LOW and motor2_A == LOW and motor2_prev_B == LOW and motor2_B == HIGH) or \
         (motor2_prev_A == LOW and motor2_A == HIGH and motor2_prev_B == HIGH and motor2_B == HIGH) or \
         (motor2_prev_A == HIGH and motor2_A == HIGH and motor2_prev_B == HIGH and motor2_B == LOW) or \
         (motor2_prev_A == HIGH and motor2_A == LOW and motor2_prev_B == LOW and motor2_B == LOW):
         counts -= 1 #anticlockwise
    

    motor2_prev_A = motor2_A
    motor2_prev_B = motor2_B 

    return None 