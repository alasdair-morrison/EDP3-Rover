#undef __ARM_FP
#include "mbed.h"
#include "RoverControl.hpp"

int main()
{
    stop();
    float period = 1.0/40000;
    float duty = 0.6;
    leftMotor.period(period);
    rightMotor.period(period);

    while (true) {
        int leftValue = leftIR.read();
        int rightValue = rightIR.read();
        int leftTurnValue = leftTurnIR.read();
        int rightTurnValue = rightTurnIR.read();
        int middleValue = middleIR.read();

        // If 90 degree right turn is needed
        if (rightTurnValue == 1 && rightValue == 1 && middleValue == 1 && leftValue == 1) {
            cornerRight(duty);
            
        }
        // If 90 degree left turn is needed
        else if (leftTurnValue == 1 && leftValue == 1 && middleValue == 1 && rightValue == 1) {
            cornerLeft(duty);
        }
        // If both sensors are on line, move forward
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1) {
            forward(duty);
        }
        // Left sensor on black line, turn left
        else if (leftValue == 0 && rightValue == 1 && middleValue == 1) {
            turnLeft(duty);
        }
        // Right sensor on black line, turn right
        else if (leftValue == 1 && rightValue == 0 && middleValue == 1) {
            turnRight(duty);
        }
        // Both sensors on tile, stop
        else if (leftValue == 0 && rightValue == 0 && middleValue == 0) {
            stop();
        }
    }
}