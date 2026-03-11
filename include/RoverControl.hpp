#undef __ARM_FP
#include "mbed.h"
#include "PinOut.hpp"

void forward(float duty) {
    leftMotor.write(duty);
    rightMotor.write(duty);

    leftBackwardControl = 0;
    rightBackwardControl = 0;
    leftForwardControl  = 1;
    rightForwardControl = 1;
}

void reverse(float duty) {
    leftMotor.write(duty);
    rightMotor.write(duty);

    leftForwardControl  = 0;
    rightForwardControl = 0;
    leftBackwardControl = 1;
    rightBackwardControl = 1;
}

void turnLeft(float duty) {
    leftMotor.write(duty);
    rightMotor.write(duty);

    leftForwardControl  = 0;
    rightForwardControl = 1;
    leftBackwardControl = 1;
    rightBackwardControl = 0;
}

void turnRight(float duty) {
    leftMotor.write(duty);
    rightMotor.write(duty);

    leftForwardControl  = 1;
    rightForwardControl = 0;
    leftBackwardControl = 0;
    rightBackwardControl = 1;
}

void stop() {
    leftMotor.write(0.8f);
    rightMotor.write(0.8f);

    leftForwardControl  = 1;
    leftBackwardControl = 1;
    rightForwardControl = 1;
    rightBackwardControl = 1;
}

void cornerLeft(float duty) {
    turnLeft(duty);
    wait_us(18000);
}

void cornerRight(float duty) {
    turnRight(duty);
    wait_us(18000);
}