#include "mbed.h"
#include "PinOut.hpp"

void forward(int time, float duty) {
    leftMotor.period(0.01);
    rightMotor.period(0.01);
    leftForwardControl = 1;
    rightForwardControl = 1;
    wait_us(time);
}

void reverse(int time, float duty) {
    leftMotor.period(0.01);
    rightMotor.period(0.01);
    leftBackwardControl = 1;
    rightBackwardControl = 1;
    wait_us(time);
}