#include <stdio.h>
#include <pigpio.h>


int prevA = 0
int prevB = 0
int count = 0

#define CPR = 48
#define GEAR_RATIO = 74.83

#define PIN_MOTOR1_IN1 = 17
#define PIN_MOTOR1_IN2 = 27 
#define PIN_MOTOR1_PWM_ENABLE = 18 
#define PIN_MOTOR1_A_OUT = 21
#define PIN_MOTOR1_B_OUT = 20 


void interruptServiceRoutine(int channel) {


}

int main() {
    if (gpioInitialise < 0) {
        fprintf(stderr, "pigpio intialisation failed\n");
        return 1;
    }

    //set up encoder feedback pins 
    gpioSetMode(PIN_MOTOR1_A_OUT, PI_INPUT);
    gpioSetMode(PIN_MOTOR1_B_OUT, PI_INPUT);

    //set up in1 and in2
    gpioSetMode(PIN_MOTOR1_IN1, PI_OUTPUT)
    gpioSetMode(PIN_MOTOR1_IN2, PI_OUTPUT)

    //set up enable pwm
    gpioSetMode(PIN_MOTOR1_PWM_ENABLE, PI_OUTPUT)


    gpioSetPWMFrequency(PIN_MOTOR1_PWM_ENABLE, 1000);
    
    gpioPWM(PIN_MOTOR_IN1, 0);

    gpioPWM(PIN_MOTOR1_IN)


    gpioSetISRFUnc(PIN_MOTOR1_A_OUT, EITHER_EDGE)
    gpioSetISRFUnc(PIN_MOTOR1_B_OUT, EITHER_EDGE)
}