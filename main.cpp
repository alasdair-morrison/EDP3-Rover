#undef __ARM_FP
#include "mbed.h"
#include "RoverControl.hpp"

void rightDepart () {
    rightMotor.write(1.1 * 0.25);
}

void leftDepart () {
    leftMotor.write(1.1 * 0.25);
}

int main()
{
    stop();
    float period = 1.0/40000;
    float duty = 0.25;
    float dutyTurnRight = 0.5;
    float dutyTurnLeft = 0.5;
    powerLED.period(2);
    leftMotor.period(period);
    rightMotor.period(period);
    leftInterrupt.fall(&leftDepart);
    rightInterrupt.fall(&rightDepart);

    while (true) {
        powerLED.write(0.25);
        forward(duty);
    }
}