#include <stdio.h>
#include <pigpio.h>

#define ENCODER_A_PIN 17 // GPIO pin number for encoder A
#define ENCODER_B_PIN 18 // GPIO pin number for encoder B

volatile int encoder_a = 0;
volatile int encoder_b = 0;

void encoderA_callback(int gpio, int level, uint32_t tick) {
    encoder_a = level;
}

void encoderB_callback(int gpio, int level, uint32_t tick) {
    encoder_b = level;
}

int main() {
    if (gpioInitialise() < 0) {
        fprintf(stderr, "pigpio initialisation failed.\n");
        return 1;
    }

    // Set encoder pins as inputs
    gpioSetMode(ENCODER_A_PIN, PI_INPUT);
    gpioSetMode(ENCODER_B_PIN, PI_INPUT);

    // Set up callback functions for the encoder pins
    gpioSetAlertFunc(ENCODER_A_PIN, encoderA_callback);
    gpioSetAlertFunc(ENCODER_B_PIN, encoderB_callback);

    while (1) {
        // Read the current states of encoder pins
        int a = gpioRead(ENCODER_A_PIN);
        int b = gpioRead(ENCODER_B_PIN);
        
        printf("Encoder A: %d, Encoder B: %d\n", a, b);

        // Add your encoder processing logic here
        
        // Sleep for a short period to reduce CPU usage
        gpioDelay(100000); // 100 ms
    }

    gpioTerminate();
    return 0;
}
