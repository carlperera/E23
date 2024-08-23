def interrupt_service_routine(channel) -> None:
    """The frequency of this interrupt service routine is dependent on the duty cycle of 
    A and B output feedbacks 
    
    """
    global moto

    motor1_A = GPIO.input(pin_A_out)
    motor1_B = GPIO.input(pin_B_out)

    if (motor1_prev_A == LOW and motor1_A == HIGH and motor1_prev_B == LOW and motor1_B == LOW) or \
        (motor1_prev_A == HIGH and motor1_A == HIGH and motor1_prev_B == LOW and motor1_B == HIGH) or \
        (motor1_prev_A == HIGH and motor1_A == LOW and motor1_prev_B == HIGH and motor1_B == HIGH) or \
        (motor1_prev_A == LOW and motor1_A == LOW and motor1_prev_B == HIGH and motor1_B == LOW):
            counts += 1  # Clockwise

    elif (motor1_prev_A == LOW and motor1_A == LOW and motor1_prev_B == LOW and motor1_B == HIGH) or \
         (motor1_prev_A == LOW and motor1_A == HIGH and motor1_prev_B == HIGH and motor1_B == HIGH) or \
         (motor1_prev_A == HIGH and motor1_A == HIGH and motor1_prev_B == HIGH and motor1_B == LOW) or \
         (motor1_prev_A == HIGH and motor1_A == LOW and motor1_prev_B == LOW and motor1_B == LOW):
         counts -= 1 #anticlockwise
    

    motor1_prev_A = motor1_A
    motor1_prev_B = motor1_B 

    return None 

