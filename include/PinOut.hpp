#include "mbed.h"

PwmOut leftMotor(PTA2);
DigitalOut leftForwardControl(PTA1);
DigitalOut leftBackwardControl(PTD4);

PwmOut rightMotor(PTA12);
DigitalOut rightForwardControl(PTA5);
DigitalOut rightBackwardControl(PTA4);