from gpiozero import AngularServo
from time import sleep

servo = AngularServo(12, min_pulse_width=0.0006, max_pulse_width=0.0023)

while (True):
    servo.angle = 90
    sleep(0.02)
    servo.angle = 80
    sleep(0.02)
    servo.angle = 70
    sleep(0.02)
    servo.angle = 60
    sleep(0.02)
    servo.angle = 0
    sleep(0.02)
    servo.angle = -60
    sleep(0.02)
    servo.angle = -70
    sleep(0.02)
    servo.angle = -80
    sleep(0.02)
    servo.angle = -90
    sleep(0.02)




    # servo.angle = 90
    # sleep(0.02)
    # servo.angle = 80
    # sleep(0.02)
    # servo.angle = 70
    # sleep(0.02)
    # servo.angle = 60
    # sleep(0.02)
    # servo.angle = 0
    # sleep(0.02)
    # servo.angle = -60
    # sleep(0.02)
    # servo.angle = -70
    # sleep(0.02)
    # servo.angle = -80
    # sleep(0.02)
    # servo.angle = -90
    # sleep(0.02)