#include "mbed.h"

PwmOut leftMotor(PTA1);
DigitalOut leftForwardControl(PTA2);
DigitalOut leftBackwardControl(PTD4);

PwmOut rightMotor(PTA12);
DigitalOut rightForwardControl(PTA5);
DigitalOut rightBackwardControl(PTA4);

PwmOut powerLED(LED_RED);