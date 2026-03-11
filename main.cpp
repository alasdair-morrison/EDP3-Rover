#undef __ARM_FP
#include "mbed.h"
#include "RoverControl.hpp"
int mode = 0;
int prevMode = 0;

void waitControl() {
    if (mode != prevMode) {
            stop();
            ThisThread::sleep_for(5ms);
            fullStop();
            ThisThread::sleep_for(75ms);
            stop();
            ThisThread::sleep_for(5ms);
        }
}

int main()
{
    stop();
    float period = 1.0/40000;
    float duty = 0.6;
    float dutyTurn = 0.6767;
    float forwardDuty = duty - 0.3;
    int wait_delay = 5000;

    leftMotor.period(period);
    rightMotor.period(period);

    while (true) {
        int leftValue = leftIR.read();
        int rightValue = rightIR.read();
        int leftTurnValue = leftTurnIR.read();
        int rightTurnValue = rightTurnIR.read();
        int middleValue = middleIR.read();
        
        // If 90 degree right turn is needed
        if ((leftValue == 1 && rightValue == 1 && rightTurnValue == 1 && middleValue == 1 && leftTurnValue == 0) || (leftTurnValue == 0 && leftValue == 0 && middleValue == 1 && rightValue == 1 && rightTurnValue == 1 )) {
            prevMode = mode;
            mode = 1;
            waitControl();
            cornerRight(dutyTurn);
        }
        // If 90 degree left turn is needed
        if ((leftValue == 1 && rightValue == 1 && rightTurnValue == 0 && middleValue == 1 && leftTurnValue == 1) || (leftTurnValue == 1 && leftValue == 0 && middleValue == 1 && rightValue == 0 && rightTurnValue == 0 )) {
            prevMode = mode;
            mode = 2;
            waitControl();
            cornerRight(dutyTurn);
        }
        // If both sensors are on BLACK, move forward
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1 && leftTurnValue == 0 && rightTurnValue == 0) {
            prevMode = mode;
            mode = 3;
            waitControl();
            cornerRight(dutyTurn);
            forward(forwardDuty);
        }
        // Left sensor on black line, turn left
        else if ( (leftTurnValue == 0 && leftValue == 0 &&  middleValue == 1 && rightValue == 1  &&  rightTurnValue == 0 )|| (leftTurnValue == 1 && leftValue == 1)) {
            prevMode = mode;
            mode = 4;
            waitControl();
            cornerRight(dutyTurn);
            turnLeft(duty);
        }
        // Right sensor on black line, turn right
        else if ( (leftTurnValue == 0 && leftValue == 0 &&  middleValue == 1 && rightValue == 1  &&  rightTurnValue == 0 )|| (rightTurnValue == 1 && rightValue == 1)) {
            prevMode = mode;
            mode = 5;
            waitControl();
            cornerRight(dutyTurn);
            turnRight(duty);
        }
            // Both sensors on WHITE, stop
        else if (leftValue == 1 && rightValue == 1 && middleValue == 1 && rightTurnValue == 1 && leftTurnValue == 1) {
            prevMode = mode;
            mode = 0;
            waitControl();
            cornerRight(dutyTurn);
            fullStop();
        }        
    }
}