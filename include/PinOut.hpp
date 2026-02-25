#undef __ARM_FP
#include "mbed.h"
//Left Motor Pins
PwmOut leftMotor(PTA1);
DigitalOut leftForwardControl(PTA2);
DigitalOut leftBackwardControl(PTD4);
//Right Motor Pins
PwmOut rightMotor(PTA12);
DigitalOut rightForwardControl(PTA5);
DigitalOut rightBackwardControl(PTA4);
//Detection LED
PwmOut powerLED(LED_RED);

//IR Sensor Pins
//DigitalIn leftIR(PTD1);
DigitalIn middleIR(PTD3);
//DigitalIn rightIR(PTD2);

//Interrupts
InterruptIn leftInterrupt(PTD1);
InterruptIn rightInterrupt(PTD2);
