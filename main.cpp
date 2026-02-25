#undef __ARM_FP
#include "mbed.h"
#include "RoverControl.hpp"

void rightDepart () {
    rightMotor.write(1.1 * 0.25);
}

void leftDepart () {
    leftMotor.write(1.1 * 0.25);
}

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
    leftInterrupt.fall(&leftDepart);
    rightInterrupt.fall(&rightDepart);

    while (true) {
        powerLED.write(0.25);
        forward(duty);
    }

    /*while (true) {
      int leftValue = leftIR.read();
      int rightValue = rightIR.read();

      // If both sensors are on white, move forward
      if (leftValue == 1 && rightValue == 1) {
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
    }*/
}