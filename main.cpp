#include "mbed.h"

// Pinout Definitions
PwmOut leftMotor(PTA2);
DigitalOut leftForwardControl(PTA1);
DigitalOut leftBackwardControl(PTD4);
PwmOut rightMotor(PTA12);
DigitalOut rightForwardControl(PTA4);
DigitalOut rightBackwardControl(PTA5);
Timer T;

// main() runs in its own thread in the OS
int main()
{
    leftMotor.period(0.01);
    rightMotor.period(0.01);
    leftForwardControl = 0;
    leftBackwardControl = 0;
    rightForwardControl = 0;
    rightBackwardControl = 0;
    T.start();
    while (true) {
        leftMotor = 0.5;
        rightMotor = 0.5;
        rightForwardControl = 1;
        leftForwardControl = 1;

        if ((T.elapsed_time().count() % 10000000) == 0) { //if timer is at a multiple 10s
            rightForwardControl = !rightForwardControl;
            leftForwardControl = !leftForwardControl;
            rightBackwardControl = !rightBackwardControl;
            leftBackwardControl = !leftBackwardControl;
        }

    }
}

