#undef __ARM_FP
#include "mbed.h"
#include "RoverControl.hpp"

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

    while (true) {
        powerLED.write(0.25);
        int leftValue = leftIR.read();
        int rightValue = rightIR.read();
        int leftTurnValue = leftTurnIR.read();
        int rightTurnValue = rightTurnIR.read();

        // If 90 degree right turn is needed
        if (leftTurnValue == 1 && rightValue == 1) {
            cornerRight(duty);
        }
        // If 90 degree left turn is needed
        else if (leftTurnValue == 1 && leftValue == 1) {
            cornerLeft(duty);
        }
        // If both sensors are on white, move forward
        else if (leftValue == 1 && rightValue == 1) {
            forward(duty);
        }
        // Left sensor on black line, turn left
        else if (leftValue == 0 && rightValue == 1) {
            turnLeft(duty);
        }
        // Right sensor on black line, turn right
        else if (leftValue == 1 && rightValue == 0) {
            turnRight(duty);
        }
        // Both sensors on black, stop
        else if (leftValue == 0 && rightValue == 0) {
            stop();
        }
    }
}