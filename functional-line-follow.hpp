#undef __ARM_FP
#include "mbed.h"
//Left Motor Pins
PwmOut leftMotor(PTD4);
DigitalOut leftForwardControl(PTA12);
DigitalOut leftBackwardControl(PTA4);
//Right Motor Pins
PwmOut rightMotor(PTA5);
DigitalOut rightForwardControl(PTC9);
DigitalOut rightBackwardControl(PTC8);

//IR Sensor Pins
DigitalIn leftIR(PTD0);
DigitalIn middleIR(PTD2);
DigitalIn rightIR(PTD3);
DigitalIn rightTurnIR(PTD1);
DigitalIn leftTurnIR(PTD5);

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
    leftMotor.write(0.0f);
    rightMotor.write(0.0f);

    leftForwardControl  = 0;
    leftBackwardControl = 0;
    rightForwardControl = 0;
    rightBackwardControl = 0;
}

void cornerLeft(float dutyTurnRight) {
    turnLeft(dutyTurnRight);
        wait_us(35000);

}

void cornerRight(float dutyTurnLeft) {
    turnRight(dutyTurnLeft);
        wait_us(35000);

}

int main()
{
    stop();
    float period = 1.0/40000;
    float duty = 0.6;
    float dutyTurnRight = 0.8;
    float dutyTurnLeft = 0.8;
    leftMotor.period(period);
    rightMotor.period(period);

    while (true) {
        int leftValue = leftIR.read();
        int rightValue = rightIR.read();
        int leftTurnValue = leftTurnIR.read();
        int rightTurnValue = rightTurnIR.read();
        int middleValue = middleIR.read();
        //printf("Far Right:%d, Right: %d, Middle: %d, Left: %d, Far Left: %d\r\n", rightTurnValue, rightValue, middleValue, leftValue, leftTurnValue);
 
        // If 90 degree right turn is needed
        if ((leftValue == 1 && rightValue == 1 && rightTurnValue == 1 && middleValue == 1 && leftTurnValue == 0) || (leftTurnValue == 0 && 
        leftValue == 0 && middleValue == 1 && rightValue == 1 && rightTurnValue == 1 )) {
            //printf("Cornering Right\r\n");
            cornerRight(dutyTurnRight);
        }
        // If 90 degree left turn is needed
        else if ((leftValue == 1 && rightValue == 1 && rightTurnValue == 0 && middleValue == 1 && leftTurnValue == 1) || (leftTurnValue == 1 && 
        leftValue == 0 && middleValue == 1 && rightValue == 0 && rightTurnValue == 0 )) {
            //printf("Cornering Left\r\n");
            cornerLeft(dutyTurnLeft);
        }
        // If both sensors are on BLACK, move forward
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1 && leftTurnValue == 0 && rightTurnValue == 0) {
            //printf("Forward\r\n");
            forward(duty - 0.1);
        }
        // Both sensors on WHITE, stop
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1 && rightTurnValue == 1 && leftTurnValue == 1) {
            //printf("Stopping\r\n");
            stop();
        }   
        // Left sensor on black line, turn left
        else if ( (leftTurnValue == 0 && leftValue == 1 &&  middleValue == 1 && rightValue == 0  &&  rightTurnValue == 0 )|| (leftTurnValue == 1 && leftValue == 1)) {
            //printf("Turning Left\r\n");
            turnLeft(duty);
        }
        // Right sensor on black line, turn right
        else if ( (leftTurnValue == 0 && leftValue == 0 &&  middleValue == 1 && rightValue == 1  &&  rightTurnValue == 0 )|| (rightTurnValue == 1 && rightValue == 1)) {
            //printf("Turning Right\r\n");
            turnRight(duty);
        }
    }
}