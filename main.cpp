#include "mbed.h"
#include "RoverControl.hpp"

int main()
{
    stop();
    float period = 0.01;
    float duty = 0.25;
    powerLED.period(2);
    while (true) {
        powerLED.write(0.25);
        // ---- Move Forward 10 seconds ----
        forward(duty, period);  // one forward call

        // ---- Stop 5 seconds ----
        wait_us(5000000);
        stop();

        // ---- Move Reverse 10 seconds ----
        reverse(duty, period);  // one  reverse call

        // ---- Stop 5 seconds ----
        wait_us(5000000);
        stop();
    }
}